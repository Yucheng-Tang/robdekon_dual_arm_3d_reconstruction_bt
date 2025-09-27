#include "emit_pose_param_action.hpp"
#include <one_arm_nbv_bt/param_utils.hpp> 
#include <tf2/LinearMath/Quaternion.h>

using geometry_msgs::msg::PoseStamped;
using nbv_param::get_or_default;

// namespace {
// template <typename T>
// void get_or_default(const rclcpp::Node::SharedPtr& node,
//                     const std::string& name,
//                     T& out, const T& def)
// {
//   if (!node->get_parameter(name, out)) {
//     out = def;
//   }
// }
// } // namespace

namespace bt_nodes {

EmitPoseParamAction::EmitPoseParamAction(const std::string& name,
                                         const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus EmitPoseParamAction::tick()
{
  RCLCPP_INFO(node_->get_logger(), "EmitPoseParamAction triggered!");

  std::string ns;
  if (!getInput("ns", ns) || ns.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "EmitPoseParam: missing 'ns' input");
    return BT::NodeStatus::FAILURE;
  }

  // Defaults
  std::string frame_id_def = "base";
  std::vector<double> xyz_def{0.4, 0.0, 0.35};
  std::vector<double> rpy_def{3.14159265359, 0.0, 0.0};

  // Read or use defaults (no declaring here)
  std::string frame_id;
  std::vector<double> xyz, rpy;
  get_or_default(node_, ns + ".frame_id", frame_id, frame_id_def);
  get_or_default(node_, ns + ".xyz",      xyz,      xyz_def);
  get_or_default(node_, ns + ".rpy",      rpy,      rpy_def);

  if (xyz.size() != 3 || rpy.size() != 3) {
    RCLCPP_ERROR(node_->get_logger(), "EmitPoseParam: bad xyz/rpy sizes under ns=%s", ns.c_str());
    return BT::NodeStatus::FAILURE;
  }

  PoseStamped target;
  target.header.frame_id = frame_id;
  target.pose.position.x = xyz[0];
  target.pose.position.y = xyz[1];
  target.pose.position.z = xyz[2];

  tf2::Quaternion q; q.setRPY(rpy[0], rpy[1], rpy[2]);
  target.pose.orientation.x = q.x();
  target.pose.orientation.y = q.y();
  target.pose.orientation.z = q.z();
  target.pose.orientation.w = q.w();

  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "EmitPoseParam: emitted from ns='" << ns << "'"
        << " frame='" << frame_id << "'\n"
        << "  pos = [" << target.pose.position.x << ", "
                       << target.pose.position.y << ", "
                       << target.pose.position.z << "] (m)\n"
        << "  quat= [" << target.pose.orientation.x << ", "
                       << target.pose.orientation.y << ", "
                       << target.pose.orientation.z << ", "
                       << target.pose.orientation.w << "]\n"
        << "  rpy  = [" << rpy[0] << ", "
                       << rpy[1] << ", "
                       << rpy[2] << "] (rad)";
    RCLCPP_INFO_STREAM(node_->get_logger(), oss.str());
  }

  setOutput("target", target);
  RCLCPP_INFO(node_->get_logger(), "EmitPoseParam: emitted pose from ns='%s'", ns.c_str());
  return BT::NodeStatus::SUCCESS;
}

} // namespace bt_nodes