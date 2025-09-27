#include "compute_reorient_action.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

using geometry_msgs::msg::PoseStamped;

namespace bt_nodes {

ComputeReorientAction::ComputeReorientAction(const std::string& name,
                                             const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus ComputeReorientAction::tick()
{
  // TODO: compute a new desired EE pose; here we rotate yaw by +30 deg
  PoseStamped p;
  p.header.frame_id = "world";
  p.pose.position.x = 0.50;
  p.pose.position.y = 0.00;
  p.pose.position.z = 0.25;

  tf2::Quaternion q; q.setRPY(0.0, 0.0, M_PI/6.0);
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();

  setOutput("target", p);
  RCLCPP_INFO(node_->get_logger(), "ComputeReorient: produced new orientation");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nodes