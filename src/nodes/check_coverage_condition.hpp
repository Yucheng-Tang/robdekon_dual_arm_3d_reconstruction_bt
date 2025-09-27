#pragma once
#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>

namespace bt_nodes {

class CheckCoverageCondition : public BT::ConditionNode {
public:
  CheckCoverageCondition(const std::string& name, const BT::NodeConfiguration& cfg);
  
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("coverage"),                 
      BT::InputPort<double>("threshold")           
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  double threshold_{0.9};
};

}  // namespace bt_nodes