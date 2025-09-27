#pragma once
#include <behaviortree_cpp_v3/condition_node.h>

namespace bt_nodes {

class IsTrueCondition : public BT::ConditionNode
{
public:
  IsTrueCondition(const std::string& name,
                  const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace bt_nodes