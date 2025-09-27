#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>

#include <string>
#include <memory>
#include <vector>

namespace bt_nodes {

class MoveToJointsAction : public BT::SyncActionNode
{
public:
  explicit MoveToJointsAction(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    // Read the joint target from params under this namespace (default in YAML: one_arm_nbv_bt.home)
    return { BT::InputPort<std::string>("ns", "one_arm_nbv_bt.home", "Param namespace for joint goal") };
  }

  BT::NodeStatus tick() override;

private:
  bool ensureClients();
  bool pickPlanningService();

  rclcpp::Node::SharedPtr node_;

  using GetMotionPlan = moveit_msgs::srv::GetMotionPlan;
  using ExecTraj      = moveit_msgs::action::ExecuteTrajectory;

  rclcpp::Client<GetMotionPlan>::SharedPtr plan_client_;
  std::string plan_service_name_;
  rclcpp_action::Client<ExecTraj>::SharedPtr exec_client_;

  // planning params (read from YAML with defaults)
  std::string group_;
  std::string pipeline_;    // empty = let move_group choose default
  double planning_time_;
  int    planning_attempts_;
  double vel_scale_;
  double acc_scale_;
  double joint_tol_;        // abs tolerance for each joint (rad)
};

} // namespace bt_nodes
