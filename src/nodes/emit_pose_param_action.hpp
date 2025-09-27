#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace bt_nodes {

class EmitPoseParamAction : public BT::SyncActionNode {
public:
  // Input port "ns": parameter namespace containing frame_id, xyz, rpy
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("ns"),
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("target") };
  }

  EmitPoseParamAction(const std::string& name, const BT::NodeConfiguration& cfg);

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

} // namespace bt_nodes