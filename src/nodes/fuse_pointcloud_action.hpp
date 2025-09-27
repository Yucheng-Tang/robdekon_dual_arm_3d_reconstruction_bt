#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <utils_msgs/srv/nbv_trigger.hpp>  // <-- your service

#include <Eigen/Core>
#include <memory>
#include <string>
#include <chrono>

namespace bt_nodes {

/**
 * FusePointCloudAction
 *
 * Subscribes one filtered point cloud (already in working_frame),
 * registers it against a persistent global map using ICP/GICP/NDT,
 * fuses it, and republishes the updated map.
 *
 * Parameters (under "one_arm_nbv_bt.fusion.*"):
 *   input_topic            (string)  default: "/nbv/filtered_cloud"
 *   publish_topic          (string)  default: "/nbv/map_cloud"
 *   method                 (string)  "icp" | "gicp" | "ndt" (default: "gicp")
 *   scan_voxel_leaf        (double)  leaf for downsampling incoming scan (0=off)
 *   map_voxel_leaf         (double)  periodic voxel leaf for map (0=off)
 *   clear_on_start         (bool)    clear map on first tick (default: false)
 *   max_correspondence     (double)  ICP/GICP max correspondence dist (m)
 *   max_iterations         (int)     max iterations (default: 50)
 *   transform_epsilon      (double)  convergence epsilon (default: 1e-6)
 *   fitness_epsilon        (double)  ICP euclidean fitness epsilon (default: 1e-5)
 *   ndt_resolution         (double)  NDT voxel grid resolution (m)
 *   ndt_step_size          (double)  NDT step size
 *   accept_fitness_max     (double)  if fitness > this, reject (<=0 disables)
 *   initial_guess_mode     (string)  "identity" | "last" (default: "last")
 *   save_pcd_path          (string)  if non-empty, save PCD after each fusion
 *   save_every_n           (int)     save every N scans (default: 0 -> every)
 *   timeout_ms             (int)     subscription wait timeout (default: 1500)
 *
 * On success publishes the updated map; returns FAILURE if no cloud or registration fails.
 */

class FusePointCloudAction : public BT::SyncActionNode
{
public:
  explicit FusePointCloudAction(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts() {
    // Optional: ask BT to reset the map on this tick
    return { 
      BT::InputPort<sensor_msgs::msg::PointCloud2::SharedPtr>("cloud"),
      BT::InputPort<bool>("reset", false, "Reset accumulated map before fusing"),
   
      // NBV outputs (since you set them later)
      BT::OutputPort<geometry_msgs::msg::Pose>("nbv_camera_pose"),
      BT::OutputPort<bool>("is_terminated"),
      BT::OutputPort<int64_t>("voxel_map_size"),
      BT::OutputPort<int64_t>("ellipsoids_size"),
      BT::OutputPort<int64_t>("nbv_result"),
      BT::OutputPort<bool>("map_ready", "True once a map already exists"),
      BT::OutputPort<sensor_msgs::msg::PointCloud2::SharedPtr>(
      "map_cloud", "Latest global map as PointCloud2")
    };
  }

  BT::NodeStatus tick() override;

private:
  using CloudT      = pcl::PointCloud<pcl::PointXYZ>;
  using CloudPtr    = CloudT::Ptr;
  using CloudConstPtr = CloudT::ConstPtr;

  // Node
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Topics / frames
  std::string input_topic_;
  std::string map_topic_;
  std::string aligned_topic_;
  std::string map_frame_;

  // Downsampling
  double input_leaf_{0.01};
  double map_leaf_{0.01};

  // ICP / GICP params
  bool   use_gicp_{false};
  int    icp_max_iter_{50};
  double icp_max_corr_dist_{0.05};
  double icp_trans_eps_{1e-6};
  double icp_fitness_eps_{1e-6};
  double icp_max_fitness_{0.01};

  // Behavior toggles
  bool        reset_on_empty_{false};
  bool        save_pcd_on_success_{false};
  std::string save_pcd_path_{"/tmp/nbv_global_map.pcd"};

  // State
  CloudPtr map_;
  std::mutex map_mtx_;

  // Publishers (lazy)
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aligned_;

  // Call NBV service
  std::string nbv_service_name_{"/nbv_ros2_node_trigger"};  
  rclcpp::Client<utils_msgs::srv::NBVTrigger>::SharedPtr nbv_client_;

  // Helpers
  bool waitOneCloud(sensor_msgs::msg::PointCloud2::SharedPtr& out,
                    std::chrono::milliseconds timeout);

  CloudPtr toPCL(const sensor_msgs::msg::PointCloud2& msg);
  sensor_msgs::msg::PointCloud2 toROS(const CloudT& pcl_cloud, const std::string& frame);

  void voxelDownsample(CloudPtr& cloud, double leaf);

  bool runICP(const CloudConstPtr& source,
              const CloudConstPtr& target,
              Eigen::Matrix4f& T_out,
              double& fitness_out);

  void publishMapLocked(const std::string& frame);
  void publishAligned(const CloudT& aligned, const std::string& frame);
  void saveMapIfRequestedLocked();

  // timeouts (ms)
  int nbv_wait_ms_{20000};       // wait for service to appear
  int nbv_call_timeout_ms_{60000}; // wait for a response

  // NBV call
  bool callNbvService(const sensor_msgs::msg::PointCloud2& cloud,
                      const geometry_msgs::msg::Pose& current_camera_pose,
                      utils_msgs::srv::NBVTrigger::Response::SharedPtr& out_res);

  // ----- Debug helpers (member versions) -----
  struct CloudStats {
    size_t size{0}, finite{0}, nan_pts{0};
    Eigen::Vector3f min{ Eigen::Vector3f::Constant( std::numeric_limits<float>::infinity()) };
    Eigen::Vector3f max{ Eigen::Vector3f::Constant(-std::numeric_limits<float>::infinity()) };
  };

  CloudStats calcStats(const CloudT& c) const;
  void logStats(const char* name, const CloudT& c) const;
  double quickRMSE(const CloudT& src, const CloudT& tgt,
                   const Eigen::Matrix4f& T,
                   size_t max_samples = 5000,
                   float max_range = std::numeric_limits<float>::infinity()) const;
  std::string matToStr(const Eigen::Matrix4f& M) const;
  double rotationAngleRad(const Eigen::Matrix3f& R) const;
};

} // namespace bt_nodes