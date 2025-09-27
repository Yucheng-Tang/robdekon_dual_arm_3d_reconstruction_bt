#include "nodes/place_action.hpp"
#include <one_arm_nbv_bt/param_utils.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <chrono>
#include <sstream>
#include <iomanip>

using nbv_param::get_or_default;

namespace bt_nodes {

using namespace std::chrono_literals;

PlaceAction::PlaceAction(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Parameters
  get_or_default(node_, "one_arm_nbv_bt.ee_link",       ee_link_,     std::string("panda_hand_tcp"));
  get_or_default(node_, "one_arm_nbv_bt.gripper_ns",    gripper_ns_,  std::string("/franka_gripper"));
  get_or_default(node_, "one_arm_nbv_bt.detach_proxy",  detach_proxy_, false);

  get_or_default(node_, "one_arm_nbv_bt.open.width",    open_width_,  0.08);
  get_or_default(node_, "one_arm_nbv_bt.open.speed",    open_speed_,  0.1);

  move_client_ = rclcpp_action::create_client<Move>(node_, gripper_ns_ + "/move");
}

bool PlaceAction::ensureClients()
{
  if (!move_client_->wait_for_action_server(1s))
  {
    RCLCPP_WARN(node_->get_logger(), "Place: action server %s/move not available", gripper_ns_.c_str());
    return false;
  }
  return true;
}

BT::NodeStatus PlaceAction::tick()
{
  if (!ensureClients())
    return BT::NodeStatus::FAILURE;

  // Open command
  Move::Goal goal;
  goal.width = open_width_;
  goal.speed = open_speed_;

  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "Place: sending Move(open) to " << gripper_ns_
        << " (width=" << goal.width << " m, speed=" << goal.speed << " m/s)";
    RCLCPP_INFO_STREAM(node_->get_logger(), oss.str());
  }

  auto gh_future = move_client_->async_send_goal(goal);
  auto rc = rclcpp::spin_until_future_complete(node_, gh_future, 5s);
  if (rc != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Place: failed to send move goal");
    return BT::NodeStatus::FAILURE;
  }
  auto gh = gh_future.get();
  if (!gh)
  {
    RCLCPP_ERROR(node_->get_logger(), "Place: null goal handle");
    return BT::NodeStatus::FAILURE;
  }

  auto res_future = move_client_->async_get_result(gh);
  rc = rclcpp::spin_until_future_complete(node_, res_future, 30s);
  if (rc != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Place: move result wait failed");
    return BT::NodeStatus::FAILURE;
  }

  auto res = res_future.get();
  if (!res.result || !res.result->success)
  {
    RCLCPP_WARN(node_->get_logger(), "Place: gripper move/open reported failure");
    return BT::NodeStatus::FAILURE;
  }

  // Optional: detach the proxy
  if (detach_proxy_)
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.object.id = "grasp_object_proxy";
    aco.object.operation = aco.object.REMOVE;
    psi.applyAttachedCollisionObject(aco);
    RCLCPP_INFO(node_->get_logger(), "Place: detached proxy");
  }

  RCLCPP_INFO(node_->get_logger(), "Place: SUCCESS");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nodes