#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <string>
#include <memory>
#include <map>
#include <vector>

namespace bt_nodes {

class PlanArmMoveAction : public BT::SyncActionNode
{
public:
  explicit PlanArmMoveAction(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target"),
      BT::InputPort<geometry_msgs::msg::Pose>("target_pose"),     // NEW: plain Pose
      BT::InputPort<geometry_msgs::msg::PoseStamped>("left_target"),   // NEW: for dual arm
      BT::InputPort<geometry_msgs::msg::PoseStamped>("right_target"),  // NEW: for dual arm
      BT::InputPort<std::string>("frame_id"),   
      BT::InputPort<std::string>("ns", "bt_node"),
      BT::InputPort<std::string>("ee_link"),
      BT::InputPort<std::string>("right_ee_link"),
      BT::InputPort<std::string>("left_ee_link"),
      BT::InputPort<std::string>("group_name"),  // NEW: override group selection
      BT::InputPort<std::string>("planner")               
    }; 
  }

  BT::NodeStatus tick() override;

private:
  // Planning configuration for different group types
  struct PlanningConfig {
    std::string pipeline_id;
    std::string planner_id;
    double planning_time;
    int planning_attempts;
    double vel_scale;
    double acc_scale;
  };

  bool ensureClients();
  bool pickPlanningService();

  // Group type detection and configuration
  std::string getGroupType(const std::string& group_name);
  PlanningConfig getPlanningConfig(const std::string& group_type);
  
  // End-effector link resolution for different groups
  std::string resolveEndEffectorLink(const std::string& group_name, const std::string& requested_ee_link);

  rclcpp::Node::SharedPtr node_;

  // MoveIt thin-client interfaces
  using GetMotionPlan = moveit_msgs::srv::GetMotionPlan;
  using ExecTraj      = moveit_msgs::action::ExecuteTrajectory;

  // MoveIt thin-client interfaces
  using GetMotionPlan = moveit_msgs::srv::GetMotionPlan;
  using ExecTraj      = moveit_msgs::action::ExecuteTrajectory;

  rclcpp::Client<GetMotionPlan>::SharedPtr plan_client_;
  std::string plan_service_name_;

  rclcpp_action::Client<ExecTraj>::SharedPtr exec_client_;

  // Default parameters (can be overridden by group-specific configs)
  std::string default_group_;
  std::string default_ee_link_;
  
  // Group-specific configurations
  std::map<std::string, PlanningConfig> group_configs_;
  std::map<std::string, std::string> group_ee_links_;  // group -> default ee_link mapping
  
  // Available groups in your dual-arm setup
  std::vector<std::string> available_groups_ = {
    "both_arm", 
    "left_fr3_arm", 
    "left_fr3_hand", 
    "right_fr3_arm", 
    "right_fr3_hand"
  };

  // Robot model / group for membership checks
  moveit::core::RobotModelPtr robot_model_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> rml_;

  // TF for link-to-link fixed transform lookups
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // // Optional fallback list (read from YAML if you like)
  // std::vector<std::string> ee_fallback_links_; // e.g. {"fr3_hand", "fr3_link8"}

  // Fallback links for different arms
  std::map<std::string, std::vector<std::string>> group_fallback_links_;

  bool ensureRobotModel_();
  bool linkInGroup_(const std::string& link, const std::string& group); 

  bool transformTargetToLink_(const geometry_msgs::msg::PoseStamped& target_for_from_link,
                            const std::string& from_link,
                            const std::string& to_link,
                            geometry_msgs::msg::PoseStamped& target_for_to_link);
  
  bool getDualArmPosesFromParams(const std::string& ns,
                                geometry_msgs::msg::PoseStamped& left_target,
                                geometry_msgs::msg::PoseStamped& right_target);

  bool parseDualArmTargets(const std::string& group_name,
                          geometry_msgs::msg::PoseStamped& left_target,
                          geometry_msgs::msg::PoseStamped& right_target);

  void initializeGroupConfigurations();
};

} // namespace bt_nodes