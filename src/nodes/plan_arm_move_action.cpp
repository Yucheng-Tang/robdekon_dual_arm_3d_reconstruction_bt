#include "nodes/plan_arm_move_action.hpp"
#include <one_arm_nbv_bt/param_utils.hpp> 
#include "utils/bt_conversion.hpp"

#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/constraints.hpp>

#include <chrono>

using nbv_param::get_or_default;    

namespace bt_nodes {

using namespace std::chrono_literals;

PlanArmMoveAction::PlanArmMoveAction(const std::string& name,
                                     const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Read parameters with fallbacks (do NOT declare here)
  get_or_default(node_, "one_arm_nbv_bt.group_name",              group_,          std::string("fr3_arm"));
  get_or_default(node_, "one_arm_nbv_bt.ee_link",                 ee_link_,        std::string("fr3_hand"));
  get_or_default(node_, "one_arm_nbv_bt.pipeline_id",             pipeline_,       std::string("")); // empty -> let move_group choose
  get_or_default(node_, "one_arm_nbv_bt.planner_id",              planner_id_,     std::string(""));
  get_or_default(node_, "one_arm_nbv_bt.allowed_planning_time",   planning_time_,  2.0);
  get_or_default(node_, "one_arm_nbv_bt.planning_attempts",       planning_attempts_, 1);
  get_or_default(node_, "one_arm_nbv_bt.max_velocity_scaling",    vel_scale_,      0.3);
  get_or_default(node_, "one_arm_nbv_bt.max_acceleration_scaling",acc_scale_,      0.3);


  // Create execute_trajectory action client; planning service is chosen lazily
  exec_client_ = rclcpp_action::create_client<ExecTraj>(node_, "/execute_trajectory");
}

bool PlanArmMoveAction::ensureRobotModel_()
{
  if (robot_model_) return true;
  try {
    if (!rml_) rml_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description");
    robot_model_ = rml_->getModel();
  } catch (...) {
    robot_model_.reset();
  }
  if (!robot_model_) {
    RCLCPP_WARN(node_->get_logger(), "PlanArmMove: robot_description / model not available");
    return false;
  }
  return true;
}

bool PlanArmMoveAction::linkInGroup_(const std::string& link, const std::string& group) 
{
  if (!ensureRobotModel_()) return false;
  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
  if (!jmg) return false;
  return jmg->hasLinkModel(link);
}

// Transform a target pose that constrains `from_link` into an equivalent
// pose that constrains `to_link`, using the fixed transform between links.
//
// If the goal says "put the CAMERA here", and CAMERA is rigidly attached
// to HAND by ^HAND T_CAMERA, then the HAND pose we need is:
//   T_world_hand = T_world_camera * inv(^HAND T_CAMERA)
bool PlanArmMoveAction::transformTargetToLink_(const geometry_msgs::msg::PoseStamped& target_for_from_link,
                            const std::string& from_link,
                            const std::string& to_link,
                            geometry_msgs::msg::PoseStamped& target_for_to_link)
{
  if (!tf_buffer_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Pose given in world frame F:
  const std::string F = target_for_from_link.header.frame_id;
  if (F.empty()) {
    RCLCPP_WARN(node_->get_logger(), "PlanArmMove: target PoseStamped has empty frame_id");
    return false;
  }

  // lookup ^to_link T from_link  (i.e., transform of 'from_link' expressed in 'to_link')
  geometry_msgs::msg::TransformStamped T_to_from;
  try {
    // time 0 = latest; these are fixed links if attached in URDF/TF
    T_to_from = tf_buffer_->lookupTransform(to_link, from_link, tf2::TimePointZero, std::chrono::milliseconds(200));
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(),
                "PlanArmMove: TF lookup '%s' <- '%s' failed: %s",
                to_link.c_str(), from_link.c_str(), e.what());
    return false;
  }

  // Convert to Eigen for composition
  Eigen::Isometry3d T_F_from = Eigen::Isometry3d::Identity();
  tf2::fromMsg(target_for_from_link.pose, T_F_from);

  Eigen::Isometry3d T_to_from_eig = tf2::transformToEigen(T_to_from);

  // T_F_to = T_F_from * inv(^to T from)
  Eigen::Isometry3d T_F_to = T_F_from * T_to_from_eig.inverse();

  target_for_to_link = target_for_from_link;
  target_for_to_link.pose = tf2::toMsg(T_F_to);
  // Keep same world frame F
  return true;
}

bool PlanArmMoveAction::pickPlanningService()
{
  if (plan_client_ && !plan_service_name_.empty())
    return true;

  // Common service names exposed by move_group
  static const std::vector<std::string> candidates = {
    "/plan_kinematic_path",   // classic MoveIt
    "/compute_motion_plan",   // some configs
    "/plan"                   // rare/fallback
  };

  for (const auto& name : candidates)
  {
    auto client = node_->create_client<GetMotionPlan>(name);
    if (client->wait_for_service(200ms))
    {
      plan_client_ = client;
      plan_service_name_ = name;
      RCLCPP_INFO(node_->get_logger(), "Using MoveIt planning service: %s",
                  plan_service_name_.c_str());
      return true;
    }
  }

  // Last attempt with a slightly longer wait on the first candidate
  auto fallback = node_->create_client<GetMotionPlan>(candidates.front());
  if (fallback->wait_for_service(1500ms))
  {
    plan_client_ = fallback;
    plan_service_name_ = candidates.front();
    RCLCPP_INFO(node_->get_logger(), "Using MoveIt planning service: %s",
                plan_service_name_.c_str());
    return true;
  }

  RCLCPP_WARN(node_->get_logger(), "No MoveIt planning service is available yet");
  return false;
}

bool PlanArmMoveAction::ensureClients()
{
  if (!pickPlanningService())
    return false;

  if (!exec_client_->wait_for_action_server(1s))
  {
    RCLCPP_WARN(node_->get_logger(), "Action server /execute_trajectory not available");
    return false;
  }
  return true;
}

BT::NodeStatus PlanArmMoveAction::tick()
{
  // Requires move_group + controllers already running
  if (!ensureClients())
  {
    RCLCPP_WARN(node_->get_logger(), "PlanArmMove: servers not ready");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped target;
  // (A) Prefer PoseStamped directly from port
  if (auto ps = getInput<geometry_msgs::msg::PoseStamped>("target")){
    target = *ps;
    RCLCPP_INFO(node_->get_logger(), "PlanArmMove: using PoseStamped from 'target' port");
  }
  // (B) Otherwise accept Pose + frame_id (default "base")
  else if (auto p = getInput<geometry_msgs::msg::Pose>("target_pose")){
    std::string frame = "base";
    (void)getInput("frame_id", frame);  // optional override
    target.header.stamp = node_->now();
    target.header.frame_id = frame;
    target.pose = *p;
    RCLCPP_INFO(node_->get_logger(), "Pose: %s", poseToString(target.pose).c_str());
    RCLCPP_INFO(node_->get_logger(), "PlanArmMove: using Pose from 'target_pose' with frame_id='%s'", frame.c_str());
  }
  // (C) Fallback to YAML params under ns
  else {
    std::string ns = "bt_node";
    (void)getInput("ns", ns);
    if (!bt_nodes::get_pose_param(node_, target, ns)){
      RCLCPP_ERROR(node_->get_logger(),
                   "PlanArmMove: no 'target' or 'target_pose' provided, and params under '%s' not found",
                   ns.c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "PlanArmMove: using target from parameters namespace '%s'", ns.c_str());
  }

  std::string ee_link = ee_link_; 
  std::string ee_override;
  if (getInput("ee_link", ee_override) && !ee_override.empty()) {
    ee_link = ee_override;
    RCLCPP_INFO(node_->get_logger(),
                "PlanArmMove: overriding ee_link with BT port: '%s'", ee_link.c_str());
  }

  if (!linkInGroup_(ee_link, group_))
  {
    // Read or default fallback list
    if (ee_fallback_links_.empty()) {
      // Optionally read from YAML once:
      // get_or_default(node_, "one_arm_nbv_bt.ee_fallback_links", ee_fallback_links_,
      //                std::vector<std::string>{});
      ee_fallback_links_ = {"fr3_hand", "fr3_link8"}; // sensible defaults for Franka
    }

    bool switched = false;
    for (const auto& fb : ee_fallback_links_)
    {
      if (!linkInGroup_(fb, group_)) continue;

      geometry_msgs::msg::PoseStamped target_fb;
      if (transformTargetToLink_(target, /*from_link=*/ee_link, /*to_link=*/fb, target_fb))
      {
        RCLCPP_WARN(node_->get_logger(),
          "PlanArmMove: link '%s' is not in group '%s'. "
          "Transformed goal to fallback link '%s'.",
          ee_link.c_str(), group_.c_str(), fb.c_str());

        target  = target_fb; // use transformed pose
        ee_link = fb;        // use fallback link for constraints
        switched = true;
        break;
      }
    }

    if (!switched) {
      RCLCPP_ERROR(node_->get_logger(),
        "PlanArmMove: ee_link '%s' not in group '%s' and no usable fallback link found.",
        ee_link.c_str(), group_.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  // Build MotionPlanRequest
  moveit_msgs::msg::MotionPlanRequest req;
  req.group_name            = group_;
  req.allowed_planning_time = planning_time_;
  req.num_planning_attempts = planning_attempts_;
  req.max_velocity_scaling_factor     = vel_scale_;
  req.max_acceleration_scaling_factor = acc_scale_;
  req.start_state.is_diff = true;

  if (!pipeline_.empty())   req.pipeline_id = pipeline_;
  if (!planner_id_.empty()) req.planner_id  = planner_id_;

  // Goal constraints from pose
  auto goal = kinematic_constraints::constructGoalConstraints(
                ee_link, target, /*pos_tol=*/0.005, /*ang_tol=*/0.01);
  req.goal_constraints.push_back(goal);

  // Call planning service
  auto sreq = std::make_shared<GetMotionPlan::Request>();
  sreq->motion_plan_request = req;

  RCLCPP_INFO(node_->get_logger(),
              "PlanArmMove: calling %s for group '%s', pipeline '%s'",
              plan_service_name_.c_str(), group_.c_str(), pipeline_.c_str());

  auto sres_future = plan_client_->async_send_request(sreq);
  auto rc = rclcpp::spin_until_future_complete(node_, sres_future, 10s);

  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "PlanArmMove: planning service wait failed (%d)", (int)rc);
    return BT::NodeStatus::FAILURE;
  } 
  
  auto sres = sres_future.get();
  const auto& mpr = sres->motion_plan_response;

  if (mpr.error_code.val != mpr.error_code.SUCCESS)
  {
    RCLCPP_WARN(node_->get_logger(), "PlanArmMove: planning failed (code %d)",
                mpr.error_code.val);
    return BT::NodeStatus::FAILURE;
  }

  // Execute via /execute_trajectory
  ExecTraj::Goal exec_goal;
  exec_goal.trajectory = mpr.trajectory;

  auto gh_future = exec_client_->async_send_goal(exec_goal);
  rc = rclcpp::spin_until_future_complete(node_, gh_future, 5s);
  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "PlanArmMove: send goal wait failed");
    return BT::NodeStatus::FAILURE;
  }

  auto gh = gh_future.get();
  if (!gh)
  {
    RCLCPP_ERROR(node_->get_logger(), "PlanArmMove: null goal handle from execute_trajectory");
    return BT::NodeStatus::FAILURE;
  }

  auto result_future = exec_client_->async_get_result(gh);
  rc = rclcpp::spin_until_future_complete(node_, result_future, 120s);
  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "PlanArmMove: execute_trajectory wait failed");
    return BT::NodeStatus::FAILURE;
  }

  auto res = result_future.get();
  if (res.result->error_code.val == res.result->error_code.SUCCESS)
  {
    RCLCPP_INFO(node_->get_logger(), "PlanArmMove: execution SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(node_->get_logger(), "PlanArmMove: execution failed (code %d)",
              res.result->error_code.val);
  return BT::NodeStatus::FAILURE;
}

} // namespace bt_nodes