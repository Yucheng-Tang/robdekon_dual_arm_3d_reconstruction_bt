#pragma once
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <behaviortree_cpp_v3/action_node.h>

namespace bt_nodes {

class ChangeObjectCenterAction : public BT::SyncActionNode
{
public:
  explicit ChangeObjectCenterAction(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts(){
    return{
        BT::InputPort<geometry_msgs::msg::PoseStamped>("pose_stamped"),
        BT::InputPort<geometry_msgs::msg::Pose>("pose"),
        BT::InputPort<std::string>("parent_frame"),
        BT::InputPort<std::string>("child_frame"),
        BT::InputPort<int>("rebroadcast_ms")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string parent_frame;
  std::string child_frame;
  int rebroadcast_ms;

  // Optional fixed pose from YAML (either RPY or QUAT)
  bool                  have_param_pose_{false};
  geometry_msgs::msg::Pose param_pose_;
  std::string param_pose_frame_id_;
};

} // namespace bt_nodes