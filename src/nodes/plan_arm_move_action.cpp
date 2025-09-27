#include "nodes/plan_arm_move_action.hpp"
#include <one_arm_nbv_bt/param_utils.hpp> 
#include "utils/bt_conversion.hpp"

#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>

using nbv_param::get_or_default;    

namespace bt_nodes {

using namespace std::chrono_literals;

PlanArmMoveAction::PlanArmMoveAction(const std::string& name,
                                     const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Read default parameters
  get_or_default(node_, "one_arm_nbv_bt.group_name", default_group_, std::string("left_fr3_arm"));
  get_or_default(node_, "one_arm_nbv_bt.ee_link", default_ee_link_, std::string("left_fr3_hand"));

  // Initialize group-specific configurations
  initializeGroupConfigurations();

  // Create execute_trajectory action client; planning service is chosen lazily
  exec_client_ = rclcpp_action::create_client<ExecTraj>(node_, "/execute_trajectory");
}

void PlanArmMoveAction::initializeGroupConfigurations()
{
  // Configure single arm groups to use OMPL with TRAC-IK
  PlanningConfig single_arm_config;
  get_or_default(node_, "one_arm_nbv_bt.single_arm.pipeline_id", single_arm_config.pipeline_id, std::string("ompl"));
  get_or_default(node_, "one_arm_nbv_bt.single_arm.planner_id", single_arm_config.planner_id, std::string("RRTConnectkConfigDefault"));
  get_or_default(node_, "one_arm_nbv_bt.single_arm.planning_time", single_arm_config.planning_time, 5.0);
  get_or_default(node_, "one_arm_nbv_bt.single_arm.planning_attempts", single_arm_config.planning_attempts, 3);
  get_or_default(node_, "one_arm_nbv_bt.single_arm.max_velocity_scaling", single_arm_config.vel_scale, 0.3);
  get_or_default(node_, "one_arm_nbv_bt.single_arm.max_acceleration_scaling", single_arm_config.acc_scale, 0.3);

  // Configure dual arm group to use cuMotion
  PlanningConfig dual_arm_config;
  get_or_default(node_, "one_arm_nbv_bt.dual_arm.pipeline_id", dual_arm_config.pipeline_id, std::string("isaac_ros_cumotion"));
  get_or_default(node_, "one_arm_nbv_bt.dual_arm.planner_id", dual_arm_config.planner_id, std::string(""));
  get_or_default(node_, "one_arm_nbv_bt.dual_arm.planning_time", dual_arm_config.planning_time, 8.0);
  get_or_default(node_, "one_arm_nbv_bt.dual_arm.planning_attempts", dual_arm_config.planning_attempts, 2);
  get_or_default(node_, "one_arm_nbv_bt.dual_arm.max_velocity_scaling", dual_arm_config.vel_scale, 0.2);
  get_or_default(node_, "one_arm_nbv_bt.dual_arm.max_acceleration_scaling", dual_arm_config.acc_scale, 0.2);

  // Hand groups use OMPL but with faster settings for simple motions
  PlanningConfig hand_config;
  get_or_default(node_, "one_arm_nbv_bt.hand.pipeline_id", hand_config.pipeline_id, std::string("ompl"));
  get_or_default(node_, "one_arm_nbv_bt.hand.planner_id", hand_config.planner_id, std::string("RRTConnectkConfigDefault"));
  get_or_default(node_, "one_arm_nbv_bt.hand.planning_time", hand_config.planning_time, 2.0);
  get_or_default(node_, "one_arm_nbv_bt.hand.planning_attempts", hand_config.planning_attempts, 2);
  get_or_default(node_, "one_arm_nbv_bt.hand.max_velocity_scaling", hand_config.vel_scale, 0.5);
  get_or_default(node_, "one_arm_nbv_bt.hand.max_acceleration_scaling", hand_config.acc_scale, 0.5);

  // Map group types to configurations
  group_configs_["single_arm"] = single_arm_config;
  group_configs_["dual_arm"] = dual_arm_config;
  group_configs_["hand"] = hand_config;

  // Load end-effector links from YAML with defaults
  std::string left_arm_ee, right_arm_ee, left_hand_ee, right_hand_ee, both_arm_ee;
  get_or_default(node_, "one_arm_nbv_bt.group_ee_links.left_fr3_arm", left_arm_ee, std::string("left_fr3_hand"));
  get_or_default(node_, "one_arm_nbv_bt.group_ee_links.right_fr3_arm", right_arm_ee, std::string("right_fr3_hand"));
  get_or_default(node_, "one_arm_nbv_bt.group_ee_links.left_fr3_hand", left_hand_ee, std::string("left_fr3_hand"));
  get_or_default(node_, "one_arm_nbv_bt.group_ee_links.right_fr3_hand", right_hand_ee, std::string("right_fr3_hand"));
  get_or_default(node_, "one_arm_nbv_bt.group_ee_links.both_arms", both_arm_ee, std::string("left_fr3_hand"));

  // Map groups to their default end-effector links
  group_ee_links_["left_fr3_arm"] = left_arm_ee;
  group_ee_links_["right_fr3_arm"] = right_arm_ee;
  group_ee_links_["left_fr3_hand"] = left_hand_ee;
  group_ee_links_["right_fr3_hand"] = right_hand_ee;
  group_ee_links_["both_arms"] = both_arm_ee;

  // Load fallback links from YAML with defaults
  std::vector<std::string> left_arm_fallbacks, right_arm_fallbacks, left_hand_fallbacks, right_hand_fallbacks, both_arm_fallbacks;
  get_or_default(node_, "one_arm_nbv_bt.group_fallbacks.left_fr3_arm", left_arm_fallbacks, 
                 std::vector<std::string>{"left_fr3_hand", "left_fr3_link8"});
  get_or_default(node_, "one_arm_nbv_bt.group_fallbacks.right_fr3_arm", right_arm_fallbacks, 
                 std::vector<std::string>{"right_fr3_hand", "right_fr3_link8"});
  get_or_default(node_, "one_arm_nbv_bt.group_fallbacks.left_fr3_hand", left_hand_fallbacks, 
                 std::vector<std::string>{"left_fr3_hand"});
  get_or_default(node_, "one_arm_nbv_bt.group_fallbacks.right_fr3_hand", right_hand_fallbacks, 
                 std::vector<std::string>{"right_fr3_hand"});
  get_or_default(node_, "one_arm_nbv_bt.group_fallbacks.both_arms", both_arm_fallbacks, 
                 std::vector<std::string>{"left_fr3_hand", "right_fr3_hand", "left_fr3_link8", "right_fr3_link8"});

  // Setup fallback links for each group
  group_fallback_links_["left_fr3_arm"] = left_arm_fallbacks;
  group_fallback_links_["right_fr3_arm"] = right_arm_fallbacks;
  group_fallback_links_["left_fr3_hand"] = left_hand_fallbacks;
  group_fallback_links_["right_fr3_hand"] = right_hand_fallbacks;
  group_fallback_links_["both_arms"] = both_arm_fallbacks;

  // Log the loaded configuration for debugging
  RCLCPP_INFO(node_->get_logger(), "Loaded planning configurations:");
  RCLCPP_INFO(node_->get_logger(), "  Single arm: %s pipeline, %.1fs planning time", 
              single_arm_config.pipeline_id.c_str(), single_arm_config.planning_time);
  RCLCPP_INFO(node_->get_logger(), "  Dual arm: %s pipeline, %.1fs planning time", 
              dual_arm_config.pipeline_id.c_str(), dual_arm_config.planning_time);
  RCLCPP_INFO(node_->get_logger(), "  Hand: %s pipeline, %.1fs planning time", 
              hand_config.pipeline_id.c_str(), hand_config.planning_time);
}

std::string PlanArmMoveAction::getGroupType(const std::string& group_name)
{
  if (group_name == "both_arms") {
    return "dual_arm";
  } else if (group_name == "left_fr3_hand" || group_name == "right_fr3_hand") {
    return "hand";
  } else if (group_name == "left_fr3_arm" || group_name == "right_fr3_arm") {
    return "single_arm";
  }
  
  RCLCPP_WARN(node_->get_logger(), 
              "Unknown group '%s', defaulting to single_arm configuration", 
              group_name.c_str());
  return "single_arm";
}

PlanArmMoveAction::PlanningConfig PlanArmMoveAction::getPlanningConfig(const std::string& group_type)
{
  auto it = group_configs_.find(group_type);
  if (it != group_configs_.end()) {
    return it->second;
  }
  
  // Fallback to single arm config
  return group_configs_["single_arm"];
}

std::string PlanArmMoveAction::resolveEndEffectorLink(const std::string& group_name, 
                                                     const std::string& requested_ee_link)
{
  if (!requested_ee_link.empty()) {
    return requested_ee_link;
  }
  
  auto it = group_ee_links_.find(group_name);
  if (it != group_ee_links_.end()) {
    return it->second;
  }
  
  RCLCPP_WARN(node_->get_logger(), 
              "No default ee_link for group '%s', using fallback", 
              group_name.c_str());
  return default_ee_link_;
}

bool PlanArmMoveAction::getDualArmPosesFromParams(const std::string& ns,
                                                 geometry_msgs::msg::PoseStamped& left_target,
                                                 geometry_msgs::msg::PoseStamped& right_target)
{
  // Load left arm pose
  std::string left_frame, right_frame;
  std::vector<double> left_xyz, left_rpy, right_xyz, right_rpy;
  
  get_or_default(node_, ns + ".left_pose.frame_id", left_frame, std::string("base"));
  get_or_default(node_, ns + ".left_pose.xyz", left_xyz, std::vector<double>{0.4, 0.2, 0.3});
  get_or_default(node_, ns + ".left_pose.rpy", left_rpy, std::vector<double>{M_PI, 0.0, 0.0});
  
  get_or_default(node_, ns + ".right_pose.frame_id", right_frame, std::string("base"));
  get_or_default(node_, ns + ".right_pose.xyz", right_xyz, std::vector<double>{0.4, -0.2, 0.3});
  get_or_default(node_, ns + ".right_pose.rpy", right_rpy, std::vector<double>{M_PI, 0.0, 0.0});
  
  if (left_xyz.size() != 3 || left_rpy.size() != 3 || right_xyz.size() != 3 || right_rpy.size() != 3) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid dual-arm pose parameters under '%s'", ns.c_str());
    return false;
  }
  
  // Build left target
  left_target.header.frame_id = left_frame;
  left_target.header.stamp = node_->now();
  left_target.pose.position.x = left_xyz[0];
  left_target.pose.position.y = left_xyz[1];
  left_target.pose.position.z = left_xyz[2];
  
  tf2::Quaternion left_q;
  left_q.setRPY(left_rpy[0], left_rpy[1], left_rpy[2]);
  left_target.pose.orientation.x = left_q.x();
  left_target.pose.orientation.y = left_q.y();
  left_target.pose.orientation.z = left_q.z();
  left_target.pose.orientation.w = left_q.w();
  
  // Build right target
  right_target.header.frame_id = right_frame;
  right_target.header.stamp = node_->now();
  right_target.pose.position.x = right_xyz[0];
  right_target.pose.position.y = right_xyz[1];
  right_target.pose.position.z = right_xyz[2];
  
  tf2::Quaternion right_q;
  right_q.setRPY(right_rpy[0], right_rpy[1], right_rpy[2]);
  right_target.pose.orientation.x = right_q.x();
  right_target.pose.orientation.y = right_q.y();
  right_target.pose.orientation.z = right_q.z();
  right_target.pose.orientation.w = right_q.w();
  
  return true;
}

bool PlanArmMoveAction::parseDualArmTargets(const std::string& group_name,
                                           geometry_msgs::msg::PoseStamped& left_target,
                                           geometry_msgs::msg::PoseStamped& right_target)
{
  // Check for explicit dual-arm targets from BT ports
  if (auto left_ps = getInput<geometry_msgs::msg::PoseStamped>("left_target")) {
    if (auto right_ps = getInput<geometry_msgs::msg::PoseStamped>("right_target")) {
      left_target = *left_ps;
      right_target = *right_ps;
      RCLCPP_INFO(node_->get_logger(), "Using explicit dual-arm targets from BT ports");
      return true;
    }
  }
  
  // Try to load from parameters
  std::string ns = "bt_node";
  (void)getInput("ns", ns);
  
  if (getDualArmPosesFromParams(ns, left_target, right_target)) {
    RCLCPP_INFO(node_->get_logger(), "Using dual-arm targets from parameters '%s'", ns.c_str());
    return true;
  }
  
  RCLCPP_ERROR(node_->get_logger(), 
               "No dual-arm targets found for group '%s'. Provide 'left_target'/'right_target' ports or configure '%s.left_pose'/'%s.right_pose' parameters",
               group_name.c_str(), ns.c_str(), ns.c_str());
  return false;
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
    T_to_from = tf_buffer_->lookupTransform(to_link, from_link, tf2::TimePointZero, std::chrono::milliseconds(1000));
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

  // Determine which group to use
  std::string group_name = default_group_;
  if (auto g = getInput<std::string>("group_name")) {
    group_name = *g;
  }

  // Validate group exists
  if (std::find(available_groups_.begin(), available_groups_.end(), group_name) == available_groups_.end()) {
    RCLCPP_ERROR(node_->get_logger(), 
                 "PlanArmMove: Unknown group '%s'. Available groups: both_arm, left_fr3_arm, right_fr3_arm, left_fr3_hand, right_fr3_hand", 
                 group_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Get planning configuration for this group type
  std::string group_type = getGroupType(group_name);
  PlanningConfig config = getPlanningConfig(group_type);

  // Allow planner override from BT port
  if (auto planner_override = getInput<std::string>("planner")) {
    if (*planner_override == "ompl") {
      config.pipeline_id = "ompl";
      config.planner_id = "RRTConnectkConfigDefault";
    } else if (*planner_override == "cumotion") {
      config.pipeline_id = "isaac_ros_cumotion";
      config.planner_id = "";
    }
    RCLCPP_INFO(node_->get_logger(), "PlanArmMove: planner override to %s", planner_override->c_str());
  }

  // Build MotionPlanRequest with group-specific configuration
  moveit_msgs::msg::MotionPlanRequest req;
  req.group_name = group_name;
  req.allowed_planning_time = config.planning_time;
  req.num_planning_attempts = config.planning_attempts;
  req.max_velocity_scaling_factor = config.vel_scale;
  req.max_acceleration_scaling_factor = config.acc_scale;
  req.start_state.is_diff = true;

  if (!config.pipeline_id.empty()) req.pipeline_id = config.pipeline_id;
  if (!config.planner_id.empty())  req.planner_id = config.planner_id;

  // Parse target pose(s) based on group type
  if (group_name == "both_arms") {
    // Dual-arm planning: need two targets
    geometry_msgs::msg::PoseStamped left_target, right_target;
    if (!parseDualArmTargets(group_name, left_target, right_target)) {
      return BT::NodeStatus::FAILURE;
    }

    // Resolve end-effector links for both arms with separate overrides
    std::string left_ee_override, right_ee_override;
    (void)getInput("left_ee_link", left_ee_override);
    (void)getInput("right_ee_link", right_ee_override);
    
    std::string left_ee_link = resolveEndEffectorLink("left_fr3_arm", left_ee_override);
    std::string right_ee_link = resolveEndEffectorLink("right_fr3_arm", right_ee_override);
    
    // Validate and handle fallbacks for left arm
    if (!linkInGroup_(left_ee_link, group_name))
    {
        auto fallback_it = group_fallback_links_.find("left_fr3_arm");
        if (fallback_it != group_fallback_links_.end()) {
            bool switched = false;
            for (const auto& fb : fallback_it->second)
            {
                if (!linkInGroup_(fb, group_name)) continue;

                geometry_msgs::msg::PoseStamped left_target_fb;
                if (transformTargetToLink_(left_target, left_ee_link, fb, left_target_fb))
                {
                    RCLCPP_WARN(node_->get_logger(),
                      "PlanArmMove: left ee_link '%s' is not in group '%s'. "
                      "Transformed left target to fallback link '%s'.",
                      left_ee_link.c_str(), group_name.c_str(), fb.c_str());

                    left_target = left_target_fb;
                    left_ee_link = fb;
                    switched = true;
                    break;
                }
            }

            if (!switched) {
                RCLCPP_ERROR(node_->get_logger(),
                  "PlanArmMove: left ee_link '%s' not in group '%s' and no usable fallback link found.",
                  left_ee_link.c_str(), group_name.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        else {
            RCLCPP_ERROR(node_->get_logger(),
              "PlanArmMove: left ee_link '%s' not in group '%s' and no fallbacks configured.",
              left_ee_link.c_str(), group_name.c_str());
            return BT::NodeStatus::FAILURE;
        }
    }
    
    // Validate and handle fallbacks for right arm
    if (!linkInGroup_(right_ee_link, group_name))
    {
        auto fallback_it = group_fallback_links_.find("right_fr3_arm");
        if (fallback_it != group_fallback_links_.end()) {
            bool switched = false;
            for (const auto& fb : fallback_it->second)
            {
                if (!linkInGroup_(fb, group_name)) continue;

                geometry_msgs::msg::PoseStamped right_target_fb;
                if (transformTargetToLink_(right_target, right_ee_link, fb, right_target_fb))
                {
                    RCLCPP_WARN(node_->get_logger(),
                      "PlanArmMove: right ee_link '%s' is not in group '%s'. "
                      "Transformed right target to fallback link '%s'.",
                      right_ee_link.c_str(), group_name.c_str(), fb.c_str());

                    right_target = right_target_fb;
                    right_ee_link = fb;
                    switched = true;
                    break;
                }
            }

            if (!switched) {
                RCLCPP_ERROR(node_->get_logger(),
                  "PlanArmMove: right ee_link '%s' not in group '%s' and no usable fallback link found.",
                  right_ee_link.c_str(), group_name.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        else {
            RCLCPP_ERROR(node_->get_logger(),
              "PlanArmMove: right ee_link '%s' not in group '%s' and no fallbacks configured.",
              right_ee_link.c_str(), group_name.c_str());
            return BT::NodeStatus::FAILURE;
        }
    }
    
    // Build dual-arm goal constraints
    auto left_goal = kinematic_constraints::constructGoalConstraints(
                      left_ee_link, left_target, /*pos_tol=*/0.005, /*ang_tol=*/0.01);
    auto right_goal = kinematic_constraints::constructGoalConstraints(
                      right_ee_link, right_target, /*pos_tol=*/0.005, /*ang_tol=*/0.01);
    
    req.goal_constraints.push_back(left_goal);
    req.goal_constraints.push_back(right_goal);
    
    RCLCPP_INFO(node_->get_logger(), 
                "Planning dual-arm motion: left[%.3f,%.3f,%.3f] right[%.3f,%.3f,%.3f]",
                left_target.pose.position.x, left_target.pose.position.y, left_target.pose.position.z,
                right_target.pose.position.x, right_target.pose.position.y, right_target.pose.position.z);
  } 
  else {
    // Single-arm planning: parse single target
    geometry_msgs::msg::PoseStamped target;
    if (auto ps = getInput<geometry_msgs::msg::PoseStamped>("target")){
      target = *ps;
      RCLCPP_INFO(node_->get_logger(), "PlanArmMove: using PoseStamped from 'target' port");
    }
    else if (auto p = getInput<geometry_msgs::msg::Pose>("target_pose")){
      std::string frame = "base";
      (void)getInput("frame_id", frame);
      target.header.stamp = node_->now();
      target.header.frame_id = frame;
      target.pose = *p;
      RCLCPP_INFO(node_->get_logger(), "Pose: %s", poseToString(target.pose).c_str());
      RCLCPP_INFO(node_->get_logger(), "PlanArmMove: using Pose from 'target_pose' with frame_id='%s'", frame.c_str());
    }
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

    // Resolve end-effector link for single arm
    std::string ee_link_override;
    (void)getInput("ee_link", ee_link_override);
    std::string ee_link = resolveEndEffectorLink(group_name, ee_link_override);

    // Handle end-effector link validation and transformation
    if (!linkInGroup_(ee_link, group_name))
    {
      auto fallback_it = group_fallback_links_.find(group_name);
      if (fallback_it != group_fallback_links_.end()) {
        bool switched = false;
        for (const auto& fb : fallback_it->second)
        {
          if (!linkInGroup_(fb, group_name)) continue;

          geometry_msgs::msg::PoseStamped target_fb;
          if (transformTargetToLink_(target, ee_link, fb, target_fb))
          {
            RCLCPP_WARN(node_->get_logger(),
              "PlanArmMove: link '%s' is not in group '%s'. "
              "Transformed goal to fallback link '%s'.",
              ee_link.c_str(), group_name.c_str(), fb.c_str());

            target = target_fb;
            ee_link = fb;
            switched = true;
            break;
          }
        }

        if (!switched) {
          RCLCPP_ERROR(node_->get_logger(),
            "PlanArmMove: ee_link '%s' not in group '%s' and no usable fallback link found.",
            ee_link.c_str(), group_name.c_str());
          return BT::NodeStatus::FAILURE;
        }
      }
    }
  auto goal = kinematic_constraints::constructGoalConstraints(
              ee_link, target, /*pos_tol=*/0.005, /*ang_tol=*/0.01);
  req.goal_constraints.push_back(goal);
  }

  // Call planning service
  auto sreq = std::make_shared<GetMotionPlan::Request>();
  sreq->motion_plan_request = req;

  RCLCPP_INFO(node_->get_logger(),
              "PlanArmMove: calling %s for group '%s' (type: %s), pipeline '%s', planner '%s'",
              plan_service_name_.c_str(), group_name.c_str(), group_type.c_str(), 
              config.pipeline_id.c_str(), config.planner_id.c_str());


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