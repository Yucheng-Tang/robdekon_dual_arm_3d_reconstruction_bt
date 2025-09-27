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
      BT::InputPort<std::string>("frame_id"),   
      BT::InputPort<std::string>("ns", "bt_node"),
      BT::InputPort<std::string>("ee_link")                
    }; 
  }

  BT::NodeStatus tick() override;

private:
  bool ensureClients();
  bool pickPlanningService();

  rclcpp::Node::SharedPtr node_;

  // MoveIt thin-client interfaces
  using GetMotionPlan = moveit_msgs::srv::GetMotionPlan;
  using ExecTraj      = moveit_msgs::action::ExecuteTrajectory;

  rclcpp::Client<GetMotionPlan>::SharedPtr plan_client_;
  std::string plan_service_name_;

  rclcpp_action::Client<ExecTraj>::SharedPtr exec_client_;

  // parameters
  std::string group_;
  std::string ee_link_;
  std::string pipeline_;     // "ompl" or "isaac_ros_cumotion"
  std::string planner_id_;
  double planning_time_;
  int    planning_attempts_;
  double vel_scale_;
  double acc_scale_;

  // Robot model / group for membership checks
  moveit::core::RobotModelPtr robot_model_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> rml_;

  // TF for link-to-link fixed transform lookups
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Optional fallback list (read from YAML if you like)
  std::vector<std::string> ee_fallback_links_; // e.g. {"fr3_hand", "fr3_link8"}

  bool ensureRobotModel_();
  bool linkInGroup_(const std::string& link, const std::string& group); 

  bool transformTargetToLink_(const geometry_msgs::msg::PoseStamped& target_for_from_link,
                            const std::string& from_link,
                            const std::string& to_link,
                            geometry_msgs::msg::PoseStamped& target_for_to_link);
};

} // namespace bt_nodes