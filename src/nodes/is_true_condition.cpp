#include "nodes/is_true_condition.hpp"

namespace bt_nodes {

IsTrueCondition::IsTrueCondition(const std::string& name,
                                 const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config)
{}

BT::PortsList IsTrueCondition::providedPorts()
{
  // SUCCESS if true, FAILURE otherwise. Missing value -> FAILURE.
  return { BT::InputPort<bool>("value", "Boolean to test; SUCCESS if true") };
}

BT::NodeStatus IsTrueCondition::tick()
{
  bool v = false;
  if (!getInput<bool>("value", v))
  {
    // No input? Treat as false.
    return BT::NodeStatus::FAILURE;
  }
  return v ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace bt_nodes