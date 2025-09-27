#include "compute_nbv_action.hpp"
using geometry_msgs::msg::PoseStamped;
namespace bt_nodes {
ComputeNBVAction::ComputeNBVAction(const std::string& n, const BT::NodeConfiguration& c)
: BT::SyncActionNode(n,c){ node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node"); }
BT::NodeStatus ComputeNBVAction::tick(){
  // TODO: call your NBV service; stub:
  PoseStamped p; p.header.frame_id="world"; p.pose.orientation.w=1.0;
  p.pose.position.x=0.45; p.pose.position.y=0.0; p.pose.position.z=0.5;
  setOutput("target", p);
  return BT::NodeStatus::SUCCESS;
}
}