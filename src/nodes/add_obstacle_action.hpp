#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "utils/bt_conversion.hpp"  // make sure this returns ROS 2 types: geometry_msgs::msg::Pose, etc.

namespace bt_nodes
{
class AddObstacle : public BT::SyncActionNode
{
public:
  // Pass in an rclcpp::Node to let the action create a service client & log
  AddObstacle(const std::string& name,
              const BT::NodeConfiguration& cfg);

  ~AddObstacle() override = default;

  // BT ports
  static BT::PortsList providedPorts();

  // Main tick
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_client_;
};

}  // namespace Turntable_NBV