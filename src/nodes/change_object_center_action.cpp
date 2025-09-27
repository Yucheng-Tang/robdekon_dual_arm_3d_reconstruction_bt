#include "nodes/change_object_center_action.hpp"

#include <chrono>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>           


#include <one_arm_nbv_bt/param_utils.hpp>

using nbv_param::get_or_default;

namespace bt_nodes {

ChangeObjectCenterAction::ChangeObjectCenterAction(
    const std::string& name, const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  // Defaults for object_center
  get_or_default(node_, "one_arm_nbv_bt.object_center.frame",
                 child_frame, std::string {"nbv_object_center"});
  get_or_default(node_, "one_arm_nbv_bt.object_center.rebroadcast_ms",
                 rebroadcast_ms, int{200});

  // ---- fixed_object_center_pose ----
  // frame_id
  get_or_default(node_, "one_arm_nbv_bt.object_center.fixed_object_center_pose.frame_id",
                 parent_frame, std::string("base"));

  // position & orientation
  std::vector<double> xyz, rpy, quat;
  get_or_default(node_, "one_arm_nbv_bt.object_center.object_center_pose.xyz",  xyz,  std::vector<double>{});
  get_or_default(node_, "one_arm_nbv_bt.object_center.object_center_pose.rpy",  rpy,  std::vector<double>{});
  get_or_default(node_, "one_arm_nbv_bt.object_center.object_center_pose.quat", quat, std::vector<double>{});

  if (xyz.size() == 3 && (rpy.size() == 3 || quat.size() == 4))
  {
    param_pose_.position.x = xyz[0];
    param_pose_.position.y = xyz[1];
    param_pose_.position.z = xyz[2];

    if (quat.size() == 4) {
      param_pose_.orientation.x = quat[0];
      param_pose_.orientation.y = quat[1];
      param_pose_.orientation.z = quat[2];
      param_pose_.orientation.w = quat[3];
    } else {
      tf2::Quaternion q;
      q.setRPY(rpy[0], rpy[1], rpy[2]);  // radians
      param_pose_.orientation.x = q.x();
      param_pose_.orientation.y = q.y();
      param_pose_.orientation.z = q.z();
      param_pose_.orientation.w = q.w();
    }

    have_param_pose_ = true;
  }
}

BT::NodeStatus ChangeObjectCenterAction::tick()
{
  // 0) Read optional overrides from ports (highest priority)
  bool has_parent_port = false;
  if (auto v = getInput<std::string>("parent_frame")) { parent_frame = v.value(); has_parent_port = true; }
  if (auto v = getInput<std::string>("child_frame"))  { child_frame  = v.value(); }
  if (auto v = getInput<int>("rebroadcast_ms"))       { rebroadcast_ms = v.value(); }

  geometry_msgs::msg::PoseStamped ps;

  // 1) Select pose: pose_stamped > pose > YAML fallback
  if (auto ps_in = getInput<geometry_msgs::msg::PoseStamped>("pose_stamped"))
  {
    ps = *ps_in;

    // If parent_frame port was given, it OVERRIDES pose_stamped.header.frame_id.
    // Otherwise, only fill header if the incoming is empty.
    if (has_parent_port || ps.header.frame_id.empty()) {
      ps.header.frame_id = parent_frame;
    }
    // else: keep pose_stamped's own header.frame_id as-is
  }
  else if (auto p_in = getInput<geometry_msgs::msg::Pose>("pose"))
  {
    // For raw Pose, we must supply a frame_id; use (possibly overridden) parent_frame.
    ps.header.frame_id = parent_frame;
    ps.header.stamp    = node_->now();
    ps.pose            = *p_in;
  }
  else if (have_param_pose_)
  {
    // YAML fallback: use (possibly overridden) parent_frame
    ps.header.frame_id = parent_frame;
    ps.header.stamp    = node_->now();
    ps.pose            = param_pose_;
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(),
      "ChangeObjectCenter: no pose provided and no YAML fallback configured.");
    return BT::NodeStatus::FAILURE;
  }

  // 2) Sanitize orientation
  // Ensure orientation is valid (identity if unset/invalid)
  auto& q = ps.pose.orientation;
  if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
      (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0))
  {
    q.x = q.y = q.z = 0.0; q.w = 1.0;
  }

  // 3) Broadcast TF
  geometry_msgs::msg::TransformStamped T;
  T.header.stamp = node_->now();            // publish with current time
  T.header.frame_id = parent_frame;
  T.child_frame_id  = child_frame;
  T.transform.translation.x = ps.pose.position.x;
  T.transform.translation.y = ps.pose.position.y;
  T.transform.translation.z = ps.pose.position.z;
  T.transform.rotation      = ps.pose.orientation;

//   // Publish once or for a short window (to avoid race conditions)
//   if (rebroadcast_ms <= 0) {
//     tf_broadcaster_->sendTransform(T);
//   } else {
//     const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(rebroadcast_ms);
//     rclcpp::Rate rate(20.0); // ~20 Hz rebroadcast
//     while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
//       T.header.stamp = node_->now();
//       tf_broadcaster_->sendTransform(T);
//       rate.sleep();
//     }
//   }
  tf_broadcaster_->sendTransform(T);

  RCLCPP_INFO(node_->get_logger(),
    "ChangeObjectCenter: %s -> %s  pos=[%.3f, %.3f, %.3f] quat=[%.3f, %.3f, %.3f, %.3f]",
    parent_frame.c_str(), child_frame.c_str(),
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z,
    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w
  );

  return BT::NodeStatus::SUCCESS;
}

} // namespace bt_nodes