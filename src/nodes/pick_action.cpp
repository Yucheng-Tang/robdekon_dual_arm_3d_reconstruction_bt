#include "nodes/pick_action.hpp"
#include <one_arm_nbv_bt/param_utils.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <chrono>
#include <sstream>
#include <iomanip>

using nbv_param::get_or_default;

// helper: read current gripper opening (meters) from <ns>/joint_states
static bool read_gripper_width_once(
    const rclcpp::Node::SharedPtr& node,
    const std::string& gripper_ns,
    double& out_width,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(500))
{
  std::string topic = gripper_ns;
  if (!topic.empty() && topic.front() != '/')
    topic.insert(topic.begin(), '/');
  topic += "/joint_states";

  std::mutex m;
  std::condition_variable cv;
  bool got = false;

  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
      topic, rclcpp::SensorDataQoS(),
      [&](const sensor_msgs::msg::JointState::SharedPtr msg)
      {
        if (msg->position.empty()) return;

        // Franka hand publishes 1 or 2 finger joints. Opening ~ sum of both (or 2x first).
        double w = 0.0;
        if (msg->position.size() >= 2)       w = msg->position[0] + msg->position[1];
        else /* size()==1 */                 w = 2.0 * msg->position.front();

        {
          std::lock_guard<std::mutex> lk(m);
          out_width = w;
          got = true;
        }
        cv.notify_one();
      });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    exec.spin_some();
    {
      std::lock_guard<std::mutex> lk(m);
      if (got) return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

namespace bt_nodes {

using namespace std::chrono_literals;

PickAction::PickAction(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Read parameters (no declare; you start node with automatically_declare_parameters_from_overrides)
  get_or_default(node_, "one_arm_nbv_bt.ee_link",      ee_link_,     std::string("fr3_hand"));
  get_or_default(node_, "one_arm_nbv_bt.gripper_ns",   gripper_ns_,  std::string("/fr3_gripper"));
  get_or_default(node_, "one_arm_nbv_bt.attach_proxy", attach_proxy_, false);
  get_or_default(node_, "one_arm_nbv_bt.proxy_radius", proxy_radius_, 0.03);

  get_or_default(node_, "one_arm_nbv_bt.grasp.width",  grasp_width_, 0.03);
  get_or_default(node_, "one_arm_nbv_bt.grasp.speed",  grasp_speed_, 0.1);
  get_or_default(node_, "one_arm_nbv_bt.grasp.force",  grasp_force_, 20.0);
  get_or_default(node_, "one_arm_nbv_bt.grasp.eps_inner", eps_inner_, 0.005);
  get_or_default(node_, "one_arm_nbv_bt.grasp.eps_outer", eps_outer_, 0.005);

  // Action clients
  grasp_client_ = rclcpp_action::create_client<Grasp>(node_, gripper_ns_ + "/grasp");
  move_client_  = rclcpp_action::create_client<Move>( node_, gripper_ns_ + "/move");
}

bool PickAction::ensureClients()
{
  if (!grasp_client_->wait_for_action_server(1s))
  {
    RCLCPP_WARN(node_->get_logger(), "Pick: action server %s/grasp not available", gripper_ns_.c_str());
    return false;
  }
  // move action is optional here; not strictly needed for pick
  (void)move_client_;
  return true;
}

BT::NodeStatus PickAction::tick()
{
  if (!ensureClients())
    return BT::NodeStatus::FAILURE;

  // Build Grasp goal
  Grasp::Goal goal;
  goal.width = grasp_width_;
  goal.speed = grasp_speed_;
  goal.force = grasp_force_;
  goal.epsilon.inner = eps_inner_;
  goal.epsilon.outer = eps_outer_;

  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "Pick: sending Grasp to " << gripper_ns_
        << " (width=" << goal.width << " m, speed=" << goal.speed
        << " m/s, force=" << goal.force << " N, eps=[" << eps_inner_ << "," << eps_outer_ << "])";
    RCLCPP_INFO_STREAM(node_->get_logger(), oss.str());
  }

  auto goal_future = grasp_client_->async_send_goal(goal);
  auto rc = rclcpp::spin_until_future_complete(node_, goal_future, 5s);
  if (rc != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Pick: failed to send grasp goal");
    return BT::NodeStatus::FAILURE;
  }
  auto gh = goal_future.get();
  if (!gh)
  {
    RCLCPP_ERROR(node_->get_logger(), "Pick: null goal handle");
    return BT::NodeStatus::FAILURE;
  }

  auto result_future = grasp_client_->async_get_result(gh);
  rc = rclcpp::spin_until_future_complete(node_, result_future, 30s);
  if (rc != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Pick: grasp result wait failed");
    return BT::NodeStatus::FAILURE;
  }
  auto res = result_future.get();
  if (!res.result || !res.result->success)
  {
    double measured = -1.0;
    if (read_gripper_width_once(node_, gripper_ns_, measured, std::chrono::milliseconds(800)))
    {
      const double lower = goal.width - goal.epsilon.inner;
      const double upper = goal.width + goal.epsilon.outer;
      RCLCPP_WARN(node_->get_logger(),
        "Pick: grasp reported failure. measured_width=%.3f m, expected in [%.3f, %.3f] (width=%.3f, eps=[%.3f, %.3f])",
        measured, lower, upper, goal.width, goal.epsilon.inner, goal.epsilon.outer);
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(),
        "Pick: grasp reported failure. (couldn't read %s/joint_states in time)",
        gripper_ns_.c_str());
    }
    RCLCPP_WARN(node_->get_logger(), "Pick: grasp reported failure");
    return BT::NodeStatus::FAILURE;
  }

  // Optional: attach proxy after successful grasp
  if (attach_proxy_)
  {
    moveit::planning_interface::PlanningSceneInterface psi;

    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = ee_link_;
    aco.object.id = "grasp_object_proxy";
    aco.object.header.frame_id = ee_link_;

    shape_msgs::msg::SolidPrimitive sph;
    sph.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    sph.dimensions = { proxy_radius_ };  // radius

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0; // at TCP origin

    aco.object.primitives = { sph };
    aco.object.primitive_poses = { pose };
    aco.object.operation = aco.object.ADD;

    psi.applyAttachedCollisionObject(aco);
    RCLCPP_INFO(node_->get_logger(), "Pick: attached proxy sphere (r=%.3f) to %s",
                proxy_radius_, ee_link_.c_str());
  }

  RCLCPP_INFO(node_->get_logger(), "Pick: SUCCESS");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nodes