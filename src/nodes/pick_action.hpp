#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>

#include <string>
#include <memory>

namespace bt_nodes {

class PickAction : public BT::SyncActionNode
{
public:
  explicit PickAction(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    // No ports needed; all params come from YAML
    return {};
  }

  BT::NodeStatus tick() override;

private:
  bool ensureClients();

  rclcpp::Node::SharedPtr node_;

  using Grasp = franka_msgs::action::Grasp;
  using Move  = franka_msgs::action::Move;

  rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;
  rclcpp_action::Client<Move>::SharedPtr  move_client_; // not used here, but handy

  // Params
  std::string ee_link_;
  std::string gripper_ns_;
  bool attach_proxy_;
  double proxy_radius_;

  // Grasp parameters
  double grasp_width_;
  double grasp_speed_;
  double grasp_force_;
  double eps_inner_;
  double eps_outer_;
};

}  // namespace bt_nodes