#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <memory>
#include <chrono>

namespace bt_nodes {

class UserGateWait : public BT::SyncActionNode
{
public:
  UserGateWait(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("topic"),       // default: /nbv/next
      BT::InputPort<std::string>("expect"),      // default: next
      BT::InputPort<int>("timeout_ms")           // default: 15000
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  std::string expect_;
  int timeout_ms_{15000};
};

} // namespace bt_nodes
