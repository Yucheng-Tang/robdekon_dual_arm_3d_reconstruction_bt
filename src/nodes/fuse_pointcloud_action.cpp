#include "nodes/fuse_pointcloud_action.hpp"
#include <one_arm_nbv_bt/param_utils.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <iomanip>
#include <sstream>

#include <pcl/kdtree/kdtree_flann.h>           
#include <tf2/LinearMath/Matrix3x3.h>           
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  

using nbv_param::get_or_default;

namespace bt_nodes {

using namespace std::chrono_literals;

FusePointCloudAction::FusePointCloudAction(const std::string& name,
                                           const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // TF
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Parameters (assumes NodeOptions automatically_declare_parameters_from_overrides=true)
  get_or_default(node_, "one_arm_nbv_bt.fuse.input_topic",   input_topic_,   std::string("/nbv/filtered_cloud"));
  get_or_default(node_, "one_arm_nbv_bt.fuse.map_topic",     map_topic_,     std::string("/nbv/global_map"));
  get_or_default(node_, "one_arm_nbv_bt.fuse.aligned_topic", aligned_topic_, std::string("/nbv/aligned_cloud"));
  get_or_default(node_, "one_arm_nbv_bt.fuse.map_frame",     map_frame_,     std::string("base"));

  get_or_default(node_, "one_arm_nbv_bt.fuse.input_leaf", input_leaf_, 0.01);
  get_or_default(node_, "one_arm_nbv_bt.fuse.map_leaf",   map_leaf_,   0.01);

  get_or_default(node_, "one_arm_nbv_bt.fuse.icp.use_gicp",      use_gicp_,       false);
  get_or_default(node_, "one_arm_nbv_bt.fuse.icp.max_iter",      icp_max_iter_,   50);
  get_or_default(node_, "one_arm_nbv_bt.fuse.icp.max_corr_dist", icp_max_corr_dist_, 0.05);
  get_or_default(node_, "one_arm_nbv_bt.fuse.icp.trans_eps",     icp_trans_eps_,  1e-6);
  get_or_default(node_, "one_arm_nbv_bt.fuse.icp.fitness_eps",   icp_fitness_eps_,1e-6);
  get_or_default(node_, "one_arm_nbv_bt.fuse.icp.max_fitness",   icp_max_fitness_,0.01);

  get_or_default(node_, "one_arm_nbv_bt.fuse.reset_on_empty",      reset_on_empty_, false);
  get_or_default(node_, "one_arm_nbv_bt.fuse.save_pcd_on_success", save_pcd_on_success_, false);
  get_or_default(node_, "one_arm_nbv_bt.fuse.save_pcd_path",       save_pcd_path_, std::string("/tmp/nbv_global_map.pcd"));

  map_.reset(new CloudT);

  // NBV service
  get_or_default(node_, "one_arm_nbv_bt.fuse.nbv_service.name", nbv_service_name_, std::string("/nbv_trigger"));
  get_or_default(node_, "one_arm_nbv_bt.fuse.nbv_service.wait_ms",
                 nbv_wait_ms_, 2000);
  get_or_default(node_, "one_arm_nbv_bt.fuse.nbv_service.call_timeout_ms",
                 nbv_call_timeout_ms_, 5000);
  
  nbv_client_ = node_->create_client<utils_msgs::srv::NBVTrigger>(nbv_service_name_);

  RCLCPP_INFO(node_->get_logger(),
  "Fuse params: use_gicp=%d input_leaf=%.4f map_leaf=%.4f icp(max_iter=%d, corr=%.3f, trans_eps=%.1e, fit_eps=%.1e, max_fit=%.4f) map_frame=%s",
  (int)use_gicp_, input_leaf_, map_leaf_, icp_max_iter_, icp_max_corr_dist_,
  icp_trans_eps_, icp_fitness_eps_, icp_max_fitness_, map_frame_.c_str());
}

bool FusePointCloudAction::waitOneCloud(sensor_msgs::msg::PointCloud2::SharedPtr& out,
                                        std::chrono::milliseconds timeout)
{
  std::mutex m; std::condition_variable cv; bool got=false;

  auto sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg){
      {
        std::lock_guard<std::mutex> lk(m);
        out = msg;
        got = true;
      }
      cv.notify_one();
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    exec.spin_some();
    {
      std::lock_guard<std::mutex> lk(m);
      if (got) return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return false;
}

FusePointCloudAction::CloudPtr
FusePointCloudAction::toPCL(const sensor_msgs::msg::PointCloud2& msg)
{
  CloudPtr cloud(new CloudT);
  pcl::fromROSMsg(msg, *cloud);
  return cloud;
}

sensor_msgs::msg::PointCloud2
FusePointCloudAction::toROS(const CloudT& pcl_cloud, const std::string& frame)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(pcl_cloud, msg);
  msg.header.stamp = node_->now();
  msg.header.frame_id = frame;
  return msg;
}

void FusePointCloudAction::voxelDownsample(CloudPtr& cloud, double leaf)
{
  if (!cloud || cloud->empty() || leaf <= 1e-9) return;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(leaf, leaf, leaf);
  vg.setInputCloud(cloud);
  CloudPtr tmp(new CloudT);
  vg.filter(*tmp);
  cloud.swap(tmp);
}

bool FusePointCloudAction::runICP(const CloudConstPtr& source,
                                  const CloudConstPtr& target,
                                  Eigen::Matrix4f& T_out,
                                  double& fitness_out)
{
  if (!source || !target || source->empty() || target->empty())
    return false;

  if (use_gicp_) {
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaxCorrespondenceDistance(icp_max_corr_dist_);
    gicp.setMaximumIterations(icp_max_iter_);
    gicp.setTransformationEpsilon(icp_trans_eps_);
    gicp.setEuclideanFitnessEpsilon(icp_fitness_eps_);
    gicp.setInputSource(source);
    gicp.setInputTarget(target);

    CloudT aligned;
    gicp.align(aligned, Eigen::Matrix4f::Identity());
    T_out = gicp.getFinalTransformation();
    fitness_out = gicp.getFitnessScore();
    return gicp.hasConverged();
  } else {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(icp_max_corr_dist_);
    icp.setMaximumIterations(icp_max_iter_);
    icp.setTransformationEpsilon(icp_trans_eps_);
    icp.setEuclideanFitnessEpsilon(icp_fitness_eps_);
    icp.setInputSource(source);
    icp.setInputTarget(target);

    CloudT aligned;
    icp.align(aligned, Eigen::Matrix4f::Identity());
    T_out = icp.getFinalTransformation();
    fitness_out = icp.getFitnessScore();
    return icp.hasConverged();
  }
}

void FusePointCloudAction::publishMapLocked(const std::string& frame)
{
  if (!pub_map_) {
    pub_map_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic_, 1);
  }
  auto msg = toROS(*map_, frame);
  pub_map_->publish(msg);
}

void FusePointCloudAction::publishAligned(const CloudT& aligned, const std::string& frame)
{
  if (!pub_aligned_) {
    pub_aligned_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(aligned_topic_, 1);
  }
  auto msg = toROS(aligned, frame);
  pub_aligned_->publish(msg);
}

void FusePointCloudAction::saveMapIfRequestedLocked()
{
  if (!save_pcd_on_success_) return;
  try {
    pcl::io::savePCDFileBinary(save_pcd_path_, *map_);
    RCLCPP_INFO(node_->get_logger(), "Fuse: saved map to %s (%zu pts)",
                save_pcd_path_.c_str(), map_->size());
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(), "Fuse: failed to save PCD: %s", e.what());
  }
}

bool FusePointCloudAction::callNbvService(const sensor_msgs::msg::PointCloud2& cloud,
                     const geometry_msgs::msg::Pose& current_camera_pose,
                     utils_msgs::srv::NBVTrigger::Response::SharedPtr& out_res)
{
  if (!nbv_client_) {
    RCLCPP_ERROR(node_->get_logger(), "NBV client not initialized");
    return false;
  }  
  if (!nbv_client_->wait_for_service(std::chrono::milliseconds(nbv_wait_ms_))) {
    RCLCPP_WARN(node_->get_logger(), "CameraSample: service %s not available", nbv_service_name_.c_str());
    return false;
  }

  auto req = std::make_shared<utils_msgs::srv::NBVTrigger::Request>();
  req->current_fused_point_cloud = cloud;
  req->current_camera_pose = current_camera_pose;

  auto fut = nbv_client_->async_send_request(req);

  // Spin with a hard deadline
  auto rc = rclcpp::spin_until_future_complete(
              node_, fut);

  // auto rc = rclcpp::spin_until_future_complete(node_, fut, std::chrono::seconds(30));
  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "CameraSample: NBV service call failed / timed out");
    return false;
  }

  out_res = fut.get();
  if (!out_res) {
    RCLCPP_ERROR(node_->get_logger(), "CameraSample: NBV service returned null response");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
              "NBV result: is_terminated=%d, result=%ld, voxels=%ld, ellipsoids=%ld",
              (int)out_res->is_terminated, (long)out_res->result,
              (long)out_res->voxel_map_size, (long)out_res->ellipsoids_size);
  return true;
}

BT::NodeStatus FusePointCloudAction::tick()
{
  // Optional reset via BT input port
  bool reset_flag = false;
  (void)getInput("reset", reset_flag);
  if (reset_flag) {
    std::lock_guard<std::mutex> lk(map_mtx_);
    map_->clear();
    RCLCPP_INFO(node_->get_logger(), "Fuse: map reset (BT input)");
  }

  // 1) Prefer cloud from BT input port; if absent, fall back to topic
  sensor_msgs::msg::PointCloud2::SharedPtr ros_cloud;
  if (getInput("cloud", ros_cloud) && ros_cloud) {
    RCLCPP_DEBUG(node_->get_logger(), "Fuse: using cloud from BT port");
  } else {
    RCLCPP_DEBUG(node_->get_logger(),
                 "Fuse: no 'cloud' on BT port; falling back to topic '%s'",
                 input_topic_.c_str());
    if (!waitOneCloud(ros_cloud, 1500ms) || !ros_cloud) {
      RCLCPP_WARN(node_->get_logger(),
                  "Fuse: no input cloud available (BT port empty and topic timeout)");
      if (reset_on_empty_) {
        std::lock_guard<std::mutex> lk(map_mtx_);
        map_->clear();
      }
      return BT::NodeStatus::FAILURE;
    }
  }
  if (ros_cloud->header.frame_id != map_frame_) {
    RCLCPP_WARN(node_->get_logger(),
      "Fuse: input frame '%s' != map_frame '%s' (ensure CameraSample outputs in map_frame)",
      ros_cloud->header.frame_id.c_str(), map_frame_.c_str());
  }

  CloudPtr cur = toPCL(*ros_cloud);
  if (!cur || cur->empty()) {
    RCLCPP_WARN(node_->get_logger(), "Fuse: empty input cloud");
    if (reset_on_empty_) {
      std::lock_guard<std::mutex> lk(map_mtx_);
      map_->clear();
    }
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(),
            "Fuse: got cloud frame='%s' size=%u x %u is_dense=%d",
            ros_cloud->header.frame_id.c_str(),
            ros_cloud->width, ros_cloud->height,
            (int)ros_cloud->is_dense);
  logStats( "scan(raw)", *cur);

  // 2) Preprocess current scan
  voxelDownsample(cur, input_leaf_);

  logStats("scan(ds)", *cur);

  // 3) Initialize map if empty
  {
    std::lock_guard<std::mutex> lk(map_mtx_);
    if (map_->empty()) {
      *map_ = *cur;
      voxelDownsample(map_, map_leaf_);
      publishMapLocked(map_frame_);
      RCLCPP_INFO(node_->get_logger(), "Fuse: initialized map (%zu pts)", map_->size());
      logStats("map(init)", *map_);
      auto map_msg_ptr =
        std::make_shared<sensor_msgs::msg::PointCloud2>(toROS(*map_, map_frame_));
      (void)setOutput("map_cloud", map_msg_ptr);
      setOutput("map_ready", false);
      return BT::NodeStatus::SUCCESS;
    }
  }
  setOutput("map_ready", true);

  // 4) Align
  Eigen::Matrix4f T_sm; double fitness = 1e9;
  CloudPtr source_ds = cur; // already downsampled
  CloudPtr target_ds(new CloudT);
  {
    std::lock_guard<std::mutex> lk(map_mtx_);
    *target_ds = *map_;
  }

  logStats("map(dst)", *target_ds);
  RCLCPP_INFO(node_->get_logger(),
    "Fuse: running %s | source_pts=%zu target_pts=%zu",
    use_gicp_ ? "GICP" : "ICP", source_ds->size(), target_ds->size());

  const bool ok = runICP(source_ds, target_ds, T_sm, fitness);

  RCLCPP_INFO(node_->get_logger(), "Fuse: PCL fitness=%.6g", fitness);

  RCLCPP_INFO(node_->get_logger(),
    "Fuse: T_sm=\n%s", matToStr(T_sm).c_str());

  const Eigen::Vector3f t_sm = T_sm.block<3,1>(0,3);
  const Eigen::Matrix3f R_sm = T_sm.block<3,3>(0,0);
  RCLCPP_INFO(node_->get_logger(),
    "Fuse: |t|=%.6f m  rot=%.6f rad (%.2f deg)",
    t_sm.norm(), rotationAngleRad(R_sm), rotationAngleRad(R_sm)*180.0/M_PI);

  // Independent RMSE check
  double rmse = quickRMSE(*cur, *target_ds, T_sm, /*max_samples=*/5000, /*max_range=*/icp_max_corr_dist_);
  RCLCPP_INFO(node_->get_logger(), "Fuse: quickRMSE (to target) = %.6g", rmse);

  if (fitness == 0.0) {
    RCLCPP_WARN(node_->get_logger(),
      "Fuse: PCL fitness reported 0.0 (suspicious unless clouds are identical). "
      "RMSE=%.6g, src=%zu tgt=%zu", rmse, cur->size(), target_ds->size());
  }

  if (!ok || !std::isfinite(fitness)) {
    RCLCPP_WARN(node_->get_logger(), "Fuse: ICP failed (ok=%d, fitness=%g)", (int)ok, fitness);
    return BT::NodeStatus::FAILURE;
  }
  if (fitness > icp_max_fitness_) {
    RCLCPP_WARN(node_->get_logger(),
                "Fuse: ICP fitness too high (%g > %g); rejecting merge",
                fitness, icp_max_fitness_);
    return BT::NodeStatus::FAILURE;
  }

  // 5) Transform current scan and merge
  CloudPtr aligned(new CloudT);
  pcl::transformPointCloud(*cur, *aligned, T_sm);
  logStats("aligned", *aligned);

  publishAligned(*aligned, map_frame_);

  {
    std::lock_guard<std::mutex> lk(map_mtx_);
    *map_ += *aligned;
    voxelDownsample(map_, map_leaf_);
    logStats( "map(updated)", *map_);
    publishMapLocked(map_frame_);
    saveMapIfRequestedLocked();

    auto map_msg_ptr =
        std::make_shared<sensor_msgs::msg::PointCloud2>(toROS(*map_, map_frame_));
    (void)setOutput("map_cloud", map_msg_ptr);
  }

  RCLCPP_INFO(node_->get_logger(),
              "Fuse: merged scan (fitness=%.4f, pts_map=%zu)", fitness, map_->size());
  

  

  // 6) Compute camera pose in the map frame (base <- camera)
  geometry_msgs::msg::TransformStamped T_base_cam;
  try {
    auto time = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    T_base_cam = tf_buffer_->lookupTransform(
        "base",                    // target (expressed in map/base)
        "camera_color_optical_frame",  // source
        time,
        rclcpp::Duration::from_seconds(0.2));
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(), "Fuse: TF lookup '%s' <- camera failed: %s",
                "base", e.what());
    return BT::NodeStatus::FAILURE;
  }

  const auto& t = T_base_cam.transform.translation;
  const auto& q = T_base_cam.transform.rotation;

  tf2::Quaternion qtf;
  tf2::fromMsg(q, qtf);
  double roll, pitch, yaw;
  tf2::Matrix3x3(qtf).getRPY(roll, pitch, yaw);

  RCLCPP_INFO(node_->get_logger(),
    "T_base_cam (%s <- %s): t=[%.4f %.4f %.4f] q=[%.5f %.5f %.5f %.5f] rpy=[%.3f %.3f %.3f] rad",
    T_base_cam.header.frame_id.c_str(),  
    T_base_cam.child_frame_id.c_str(),   
    t.x, t.y, t.z,
    q.x, q.y, q.z, q.w,
    roll, pitch, yaw);

  // Build Pose for NBV
  geometry_msgs::msg::Pose cam_pose_msg;
  cam_pose_msg.position.x = T_base_cam.transform.translation.x;
  cam_pose_msg.position.y = T_base_cam.transform.translation.y;
  cam_pose_msg.position.z = T_base_cam.transform.translation.z;
  cam_pose_msg.orientation = T_base_cam.transform.rotation; 

  // Choose which cloud to send:
  // - aligned scan (current scan in map frame)
  sensor_msgs::msg::PointCloud2 cloud_for_nbv = toROS(*aligned, map_frame_);

  // Call NBV
  utils_msgs::srv::NBVTrigger::Response::SharedPtr nbv_res;
  if (!callNbvService(cloud_for_nbv, cam_pose_msg, nbv_res)) {
    return BT::NodeStatus::FAILURE;
  }

  // Log and expose outputs
  const auto& P = nbv_res->nbv_camera_pose;
  RCLCPP_INFO(node_->get_logger(),
              "NBV response:\n"
              "  is_terminated: %s\n"
              "  result: %ld\n"
              "  voxel_map_size: %ld\n"
              "  ellipsoids_size: %ld\n"
              "  nbv_camera_pose: pos[%.3f %.3f %.3f] quat[%.3f %.3f %.3f %.3f]",
              nbv_res->is_terminated ? "true" : "false",
              (long)nbv_res->result,
              (long)nbv_res->voxel_map_size,
              (long)nbv_res->ellipsoids_size,
              P.position.x, P.position.y, P.position.z,
              P.orientation.x, P.orientation.y, P.orientation.z, P.orientation.w);

  // Only do these setOutput calls because we declared the ports in providedPorts().
  (void)setOutput("nbv_camera_pose", nbv_res->nbv_camera_pose);
  (void)setOutput("is_terminated",   nbv_res->is_terminated);
  (void)setOutput("voxel_map_size",  static_cast<int64_t>(nbv_res->voxel_map_size));
  (void)setOutput("ellipsoids_size", static_cast<int64_t>(nbv_res->ellipsoids_size));
  (void)setOutput("nbv_result",      static_cast<int64_t>(nbv_res->result));
  
  return BT::NodeStatus::SUCCESS;
}

// ====== debug helpers (member impls) ======
FusePointCloudAction::CloudStats
FusePointCloudAction::calcStats(const CloudT& c) const
{
  CloudStats s; s.size = c.size();
  for (const auto& p : c.points) {
    const bool ok = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    if (!ok) { s.nan_pts++; continue; }
    s.finite++;
    s.min.x() = std::min(s.min.x(), p.x);
    s.min.y() = std::min(s.min.y(), p.y);
    s.min.z() = std::min(s.min.z(), p.z);
    s.max.x() = std::max(s.max.x(), p.x);
    s.max.y() = std::max(s.max.y(), p.y);
    s.max.z() = std::max(s.max.z(), p.z);
  }
  return s;
}

void FusePointCloudAction::logStats(const char* name, const CloudT& c) const
{
  const auto s = calcStats(c);
  RCLCPP_INFO(node_->get_logger(),
    "%s: size=%zu finite=%zu nan=%zu bounds=[min(%.3f %.3f %.3f) max(%.3f %.3f %.3f)]",
    name, s.size, s.finite, s.nan_pts,
    s.min.x(), s.min.y(), s.min.z(), s.max.x(), s.max.y(), s.max.z());
}

double FusePointCloudAction::quickRMSE(const CloudT& src, const CloudT& tgt,
                                       const Eigen::Matrix4f& T,
                                       size_t max_samples,
                                       float max_range) const
{
  if (src.empty() || tgt.empty()) return std::numeric_limits<double>::quiet_NaN();

  pcl::KdTreeFLANN<pcl::PointXYZ> kdt;
  kdt.setInputCloud(tgt.makeShared());

  const size_t samples = std::min(max_samples, src.size());
  const size_t step    = std::max<size_t>(1, src.size() / std::max<size_t>(size_t{1}, samples));

  double sum2 = 0.0; size_t used = 0;

  for (size_t i = 0; i < src.size(); i += step) {
    const auto& p = src.points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

    Eigen::Vector4f ph(p.x, p.y, p.z, 1.f);
    Eigen::Vector4f pt = T * ph;
    pcl::PointXYZ q; q.x = pt.x(); q.y = pt.y(); q.z = pt.z();

    std::vector<int> idx(1); std::vector<float> d2(1);
    if (kdt.nearestKSearch(q, 1, idx, d2) == 1) {
      if (d2[0] <= max_range * max_range) { sum2 += d2[0]; used++; }
    }
  }

  if (used == 0) return std::numeric_limits<double>::infinity();
  return std::sqrt(sum2 / static_cast<double>(used));
}

std::string FusePointCloudAction::matToStr(const Eigen::Matrix4f& M) const
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed); oss << std::setprecision(6);
  for (int r = 0; r < 4; ++r) {
    oss << "[" << M(r,0) << " " << M(r,1) << " " << M(r,2) << " " << M(r,3) << "]";
    if (r != 3) oss << "\n";
  }
  return oss.str();
}

double FusePointCloudAction::rotationAngleRad(const Eigen::Matrix3f& R) const
{
  double tr = (R.trace() - 1.0) * 0.5;
  tr = std::max(-1.0, std::min(1.0, tr));
  return std::acos(tr);
}

} // namespace bt_nodes