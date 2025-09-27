#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <string>
#include <mutex>
#include <chrono>

namespace bt_nodes {

class EvaluateReconstructionAction : public BT::SyncActionNode
{
public:
  explicit EvaluateReconstructionAction(const std::string& name,
                                        const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return {
      // Preferred: pass the global map cloud via BT
      BT::InputPort<sensor_msgs::msg::PointCloud2::SharedPtr>("map_cloud",
        "If provided, use this map cloud directly; otherwise subscribe to map_topic."),
      // Toggle evaluation (BT has priority). If false -> node returns SUCCESS without work.
      BT::InputPort<bool>("do_eval", false, "Run evaluation or skip."),
      // Optional overrides (else read from YAML)
      BT::InputPort<std::string>("reference_file", "Path to .pcd or .stl reference"),
      BT::InputPort<double>("dist_thresh", 0.01, "Threshold for completeness/accuracy (m)"),

      // Outputs (metrics)
      BT::OutputPort<double>("rmse_map_to_ref"),
      BT::OutputPort<double>("rmse_ref_to_map"),
      BT::OutputPort<double>("mean_map_to_ref"),
      BT::OutputPort<double>("mean_ref_to_map"),
      BT::OutputPort<double>("chamfer_mean"),
      BT::OutputPort<double>("accuracy"),     // fraction map->ref within thresh
      BT::OutputPort<double>("completeness")  // fraction ref->map within thresh
    };
  }

  BT::NodeStatus tick() override;

private:
  // ========== Types ==========
  using Cloud   = pcl::PointCloud<pcl::PointXYZ>;
  using CloudPtr = Cloud::Ptr;
  using CloudConstPtr = Cloud::ConstPtr;

  // ========== Node state / params ==========
  rclcpp::Node::SharedPtr node_;

  // Sub/topic fallback when BT port not given
  std::string map_topic_;
  std::string map_frame_;

  // Reference model config
  std::string reference_file_;
  std::string reference_type_;      // "auto" | "pcd" | "stl"
  int         sample_points_{100000}; // STL sampling count
  double      leaf_map_{0.0};
  double      leaf_ref_{0.0};
  double      dist_thresh_{0.01};

  // Internal ref cache
  CloudPtr ref_cloud_;
  std::string ref_loaded_from_;
  std::mutex ref_mtx_;

  // ========== Helpers ==========
  // Convert ROS2 cloud to PCL
  static CloudPtr toPCL(const sensor_msgs::msg::PointCloud2& msg);
  // Voxel downsample (no-op if leaf<=0)
  static void voxelDownsample(CloudPtr& cloud, double leaf);

  // Loaders (PCD or STL). Throws std::runtime_error on failure.
  CloudPtr loadReferencePCD(const std::string& path);
  CloudPtr loadReferenceSTL(const std::string& path, int n_samples);

  // Ensure reference cloud loaded (from YAML/ports). Returns nullptr on error.
  CloudPtr getOrLoadReference(const std::string& file, const std::string& type, int n_samples);

  // Subscribe once if port missing
  bool waitOneCloudFromTopic(const std::string& topic,
                             sensor_msgs::msg::PointCloud2::SharedPtr& out,
                             std::chrono::milliseconds timeout);

  // Compute one-way nearest-neighbor stats
  struct NNStats {
    double mean{0.0}, rmse{0.0}, max{0.0};
    double within_thresh_ratio{0.0};
    size_t n{0};
  };
  NNStats oneWayStats(const Cloud& A, const Cloud& B, double thresh_m);

  // Pretty logging
  static void logStats(const rclcpp::Logger& log, const char* tag, const Cloud& c);
  static std::string guessTypeFromExt(const std::string& path); // "pcd"/"stl"/""
};

} // namespace bt_nodes