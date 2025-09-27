#include "move_to_joint_action.hpp"
#include <one_arm_nbv_bt/param_utils.hpp>                 // get_or_default()
#include <moveit_msgs/msg/constraints.hpp>
#include <chrono>
#include <sstream>
#include <iomanip>

using nbv_param::get_or_default;

namespace bt_nodes {

using namespace std::chrono_literals;

MoveToJointsAction::MoveToJointsAction(const std::string& name,
                                       const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Planning settings (same style as PlanArmMoveAction)
  get_or_default(node_, "one_arm_nbv_bt.group_name",              group_,          std::string("panda_manipulator"));
  get_or_default(node_, "one_arm_nbv_bt.pipeline_id",             pipeline_,       std::string(""));  // "" -> use default
  get_or_default(node_, "one_arm_nbv_bt.allowed_planning_time",   planning_time_,  2.0);
  get_or_default(node_, "one_arm_nbv_bt.planning_attempts",       planning_attempts_, 1);
  get_or_default(node_, "one_arm_nbv_bt.max_velocity_scaling",    vel_scale_,      0.7);
  get_or_default(node_, "one_arm_nbv_bt.max_acceleration_scaling",acc_scale_,      0.7);
  get_or_default(node_, "one_arm_nbv_bt.joint_goal_tolerance",    joint_tol_,      0.005);

  exec_client_ = rclcpp_action::create_client<ExecTraj>(node_, "/execute_trajectory");
}

bool MoveToJointsAction::pickPlanningService()
{
  if (plan_client_ && !plan_service_name_.empty())
    return true;

  static const std::vector<std::string> candidates = {
    "/plan_kinematic_path",
    "/compute_motion_plan",
    "/plan"
  };

  for (const auto& name : candidates)
  {
    auto client = node_->create_client<GetMotionPlan>(name);
    if (client->wait_for_service(200ms))
    {
      plan_client_ = client;
      plan_service_name_ = name;
      RCLCPP_INFO(node_->get_logger(), "MoveToJoints: using planning service: %s", name.c_str());
      return true;
    }
  }

  auto fallback = node_->create_client<GetMotionPlan>(candidates.front());
  if (fallback->wait_for_service(1500ms))
  {
    plan_client_ = fallback;
    plan_service_name_ = candidates.front();
    RCLCPP_INFO(node_->get_logger(), "MoveToJoints: using planning service: %s", plan_service_name_.c_str());
    return true;
  }

  RCLCPP_WARN(node_->get_logger(), "MoveToJoints: planning service not available yet");
  return false;
}

bool MoveToJointsAction::ensureClients()
{
  if (!pickPlanningService())
    return false;

  if (!exec_client_->wait_for_action_server(1s))
  {
    RCLCPP_WARN(node_->get_logger(), "MoveToJoints: /execute_trajectory action not available");
    return false;
  }
  return true;
}

BT::NodeStatus MoveToJointsAction::tick()
{
  if (!ensureClients())
    return BT::NodeStatus::FAILURE;

  // Read param namespace (default: one_arm_nbv_bt.home)
  std::string ns;
  if (!getInput("ns", ns) || ns.empty())
    ns = "one_arm_nbv_bt.home";

  // Expect:
  //   <ns>.names:     [string...]
  //   <ns>.positions: [double...] (same size)
  // Provide sane defaults for Franka "ready" (radians)
  std::vector<std::string> names_def = {
    "panda_joint1","panda_joint2","panda_joint3","panda_joint4",
    "panda_joint5","panda_joint6","panda_joint7"
  };
  // Classic "ready" pose
  std::vector<double> pos_def = {0.0, -0.785398, 0.0, -2.35619, 0.0, 1.5708, 0.785398};

  std::vector<std::string> names;
  std::vector<double> positions;
  get_or_default(node_, ns + ".names",     names,     names_def);
  get_or_default(node_, ns + ".positions", positions, pos_def);

  if (names.size() != positions.size() || names.empty())
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "MoveToJoints: invalid joint target under '%s' (names=%zu, positions=%zu)",
                 ns.c_str(), names.size(), positions.size());
    return BT::NodeStatus::FAILURE;
  }

  // Build joint goal constraints
  moveit_msgs::msg::Constraints goal;
  goal.name = "bt_joint_goal";
  goal.joint_constraints.reserve(names.size());

  for (size_t i = 0; i < names.size(); ++i)
  {
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = names[i];
    jc.position   = positions[i];
    jc.tolerance_above = joint_tol_;
    jc.tolerance_below = joint_tol_;
    jc.weight = 1.0;
    goal.joint_constraints.push_back(jc);
  }

  // Build MotionPlanRequest
  moveit_msgs::msg::MotionPlanRequest req;
  req.group_name = group_;
  if (!pipeline_.empty()) {
    req.pipeline_id = pipeline_;
  }
  req.allowed_planning_time = planning_time_;
  req.num_planning_attempts = planning_attempts_;
  req.max_velocity_scaling_factor     = vel_scale_;
  req.max_acceleration_scaling_factor = acc_scale_;
  req.start_state.is_diff = true;
  req.goal_constraints = {goal};

  // Nice log
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "MoveToJoints: ns='" << ns << "' group='" << group_ << "' pipeline='"
        << (pipeline_.empty() ? "(default)" : pipeline_) << "'\n  [";
    for (size_t i = 0; i < names.size(); ++i)
    {
      oss << names[i] << "=" << positions[i];
      if (i + 1 < names.size()) oss << ", ";
    }
    oss << "]";
    RCLCPP_INFO_STREAM(node_->get_logger(), oss.str());
  }

  // ---- plan
  auto sreq = std::make_shared<GetMotionPlan::Request>();
  sreq->motion_plan_request = req;

  auto plan_future = plan_client_->async_send_request(sreq);
  auto rc = rclcpp::spin_until_future_complete(node_, plan_future, 10s);
  if (rc != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveToJoints: planning service wait failed");
    return BT::NodeStatus::FAILURE;
  }
  auto sres = plan_future.get();
  const auto& mpr = sres->motion_plan_response;

  if (mpr.error_code.val != mpr.error_code.SUCCESS)
  {
    RCLCPP_WARN(node_->get_logger(), "MoveToJoints: planning failed (code %d)", mpr.error_code.val);
    return BT::NodeStatus::FAILURE;
  }

  // ---- execute
  ExecTraj::Goal exec_goal;
  exec_goal.trajectory = mpr.trajectory;

  auto gh_future = exec_client_->async_send_goal(exec_goal);
  rc = rclcpp::spin_until_future_complete(node_, gh_future, 5s);
  if (rc != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveToJoints: send goal wait failed");
    return BT::NodeStatus::FAILURE;
  }
  auto gh = gh_future.get();
  if (!gh)
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveToJoints: null goal handle");
    return BT::NodeStatus::FAILURE;
  }

  auto result_future = exec_client_->async_get_result(gh);
  rc = rclcpp::spin_until_future_complete(node_, result_future, 120s);
  if (rc != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "MoveToJoints: execute_trajectory wait failed");
    return BT::NodeStatus::FAILURE;
  }

  auto res = result_future.get();
  if (res.result->error_code.val == res.result->error_code.SUCCESS)
  {
    RCLCPP_INFO(node_->get_logger(), "MoveToJoints: execution SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(node_->get_logger(), "MoveToJoints: execution failed (code %d)",
              res.result->error_code.val);
  return BT::NodeStatus::FAILURE;
}

} // namespace bt_nodes