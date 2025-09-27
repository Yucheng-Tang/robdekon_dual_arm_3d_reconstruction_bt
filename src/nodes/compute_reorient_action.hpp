#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace bt_nodes {

class ComputeReorientAction : public BT::SyncActionNode {
public:
  ComputeReorientAction(const std::string& name, const BT::NodeConfiguration& cfg);
  static BT::PortsList providedPorts() {
    return { BT::OutputPort<geometry_msgs::msg::PoseStamped>("target") };
  }
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace bt_nodes