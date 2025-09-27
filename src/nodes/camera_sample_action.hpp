#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <pcl/io/pcd_io.h>
#include <std_msgs/msg/string.hpp>  
#include <filesystem>
#include <utils_msgs/srv/nbv_trigger.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <array>
#include <vector>
#include <string>

namespace moveit { namespace core {
  class RobotModel;
  class RobotState;
}}
namespace planning_scene {
  class PlanningScene;
}
namespace shapes {
  class Shape;
}

namespace collision_detection {
  class AllowedCollisionMatrix;
}

namespace bt_nodes {

/**
 * CameraSampleAction
 *
 * - Subscribes one PointCloud2 from RealSense (or any source)
 * - Transforms to working_frame (e.g., "base")
 * - Filters: desk plane (z cutoff), ROI box, ROI sphere
 * - Masks robot with either:
 *     (A) fast spherical link masks, or
 *     (B) exact-ish mesh mask via MoveIt/FCL
 * - Optionally masks attached object near ee_link
 * - Publishes filtered cloud to a topic
 *
 * Parameters (names under "one_arm_nbv_bt.*") are loaded from your YAML.
 */
class CameraSampleAction : public BT::SyncActionNode
{
public:
  CameraSampleAction(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts() {
    return {
      // BT::OutputPort<geometry_msgs::msg::Pose>("nbv_camera_pose"),
      // BT::OutputPort<bool>("is_terminated"),
      // BT::OutputPort<int64_t>("voxel_map_size"),
      // BT::OutputPort<int64_t>("ellipsoids_size"),
      // BT::OutputPort<int64_t>("nbv_result"),
      BT::OutputPort<sensor_msgs::msg::PointCloud2::SharedPtr>("cloud"),
    };  
  }

  BT::NodeStatus tick() override;

private:
  // ROS
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // IO
  std::string cloud_topic_;
  std::string working_frame_;
  std::string publish_topic_;
  double voxel_leaf_{0.0};

  // Desk filtering
  bool   desk_enable_{true};
  double desk_height_{0.0};
  double desk_margin_{0.01};

  // ROI (box)
  bool roi_box_enable_{false};
  std::array<double,3> roi_min_{ {-1e9,-1e9,-1e9} };
  std::array<double,3> roi_max_{ {+1e9,+1e9,+1e9} };

  // ROI (sphere)
  bool        roi_sphere_enable_{false};
  std::string roi_sphere_frame_;
  double      roi_sphere_radius_{0.25};

  // Robot masking (mode: "spheres" or "mesh")
  bool   robot_mask_enable_{false};
  std::string robot_mask_mode_{"spheres"};

  // (A) spheres
  std::vector<std::string> robot_mask_frames_;
  std::vector<double>      robot_mask_radii_; // meters

  // Attached object masking (sphere around ee_link)
  bool        attached_mask_enable_{true};
  std::string attached_mask_frame_; // typically ee_link
  double      attached_mask_radius_{0.05};

  // (B) mesh via MoveIt/FCL
  bool        mesh_mask_enable_{false};
  double      mesh_probe_radius_{0.01};
  std::string joint_states_topic_{"/joint_states"};
  // Pointers (lazy-inited in ctor if mesh mode)
  std::shared_ptr<moveit::core::RobotModel>         robot_model_;
  std::shared_ptr<planning_scene::PlanningScene>    planning_scene_;
  std::shared_ptr<moveit::core::RobotState>         robot_state_;
  std::shared_ptr<collision_detection::AllowedCollisionMatrix> acm_;
  std::string probe_id_{"__nbv_probe__"};
  std::shared_ptr<const shapes::Shape>              probe_shape_; // sphere shape

  // Pcd file saving
  bool save_enable_{false};
  std::string save_dir_{"/tmp"};
  std::string save_prefix_{"nbv_filtered"};
  bool save_ascii_{false};

  // --- helpers ---
  bool waitOneCloud(sensor_msgs::msg::PointCloud2::SharedPtr& out,
                    std::chrono::milliseconds timeout);

  bool lookupFrame(const std::string& target, const std::string& source,
                   geometry_msgs::msg::TransformStamped& T,
                   const rclcpp::Duration& timeout);

  void publishFiltered(const sensor_msgs::msg::PointCloud2& cloud_msg);

  bool read_joint_state_once(sensor_msgs::msg::JointState& out,
                             std::chrono::milliseconds timeout);

  void apply_joint_state(const sensor_msgs::msg::JointState& js);

  std::string makePcdFilename(const std::string& frame);
  bool saveCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                  const std::string& frame,
                  std::string& out_path);
  // bool callNbvService(sensor_msgs::msg::PointCloud2& out_pc,
  //                      const geometry_msgs::msg::Pose& current_camera_pose,
  //                      utils_msgs::srv::NBVTrigger::Response::SharedPtr& out_res);

};

} // namespace bt_nodes