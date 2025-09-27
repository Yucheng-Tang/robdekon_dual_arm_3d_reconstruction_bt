#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <franka_msgs/action/move.hpp>

#include <string>
#include <memory>

namespace bt_nodes {

class PlaceAction : public BT::SyncActionNode
{
public:
  explicit PlaceAction(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    // No ports; uses params for width/speed and detach flag
    return {};
  }

  BT::NodeStatus tick() override;

private:
  bool ensureClients();

  rclcpp::Node::SharedPtr node_;

  using Move = franka_msgs::action::Move;
  rclcpp_action::Client<Move>::SharedPtr move_client_;

  // Params
  std::string ee_link_;
  std::string gripper_ns_;
  bool detach_proxy_;
  double open_width_;
  double open_speed_;
};

}  // namespace bt_nodes