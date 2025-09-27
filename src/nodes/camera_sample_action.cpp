#include "nodes/camera_sample_action.hpp"
#include <one_arm_nbv_bt/param_utils.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <geometric_shapes/shapes.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>
#include <condition_variable>
#include <chrono>
#include <thread>
#include <sstream>
#include <iomanip>
#include <algorithm>


using nbv_param::get_or_default;

namespace bt_nodes {

using namespace std::chrono_literals;

CameraSampleAction::CameraSampleAction(const std::string& name,
                                       const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // TF
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // --- Parameters (read without declaring; your main enables auto-declare) ---
  // Camera topics/frames
  get_or_default(node_, "one_arm_nbv_bt.sample.camera.topic",        cloud_topic_,   std::string("/camera/depth/color/points"));
  get_or_default(node_, "one_arm_nbv_bt.sample.camera.working_frame",working_frame_, std::string("base"));
  get_or_default(node_, "one_arm_nbv_bt.sample.camera.publish_topic",publish_topic_, std::string("/nbv/filtered_cloud"));
  get_or_default(node_, "one_arm_nbv_bt.sample.camera.voxel_leaf",   voxel_leaf_,    0.0);

  // Desk
  get_or_default(node_, "one_arm_nbv_bt.sample.desk.enable",  desk_enable_,  true);
  get_or_default(node_, "one_arm_nbv_bt.sample.desk.height",  desk_height_,  0.0);
  get_or_default(node_, "one_arm_nbv_bt.sample.desk.margin",  desk_margin_,  0.01);

  // ROI box
  get_or_default(node_, "one_arm_nbv_bt.sample.roi.box.enable", roi_box_enable_, false);
  {
    std::vector<double> vmin{ -1e9, -1e9, -1e9 }, vmax{ 1e9, 1e9, 1e9 };
    get_or_default(node_, "one_arm_nbv_bt.sample.roi.box.min", vmin, vmin);
    get_or_default(node_, "one_arm_nbv_bt.sample.roi.box.max", vmax, vmax);
    if (vmin.size()==3) roi_min_ = { vmin[0], vmin[1], vmin[2] };
    if (vmax.size()==3) roi_max_ = { vmax[0], vmax[1], vmax[2] };
  }

  // ROI sphere
  get_or_default(node_, "one_arm_nbv_bt.roi.sample.sphere.enable", roi_sphere_enable_, false);
  get_or_default(node_, "one_arm_nbv_bt.roi.sample.sphere.frame",  roi_sphere_frame_,  std::string(""));
  get_or_default(node_, "one_arm_nbv_bt.roi.sample.sphere.radius", roi_sphere_radius_, 0.25);

  // Robot mask mode
  get_or_default(node_, "one_arm_nbv_bt.sample.robot_mask.enable", robot_mask_enable_, false);
  get_or_default(node_, "one_arm_nbv_bt.sample.robot_mask.mode",   robot_mask_mode_,   std::string("spheres"));
  std::transform(robot_mask_mode_.begin(), robot_mask_mode_.end(),
                 robot_mask_mode_.begin(), ::tolower);
  mesh_mask_enable_ = robot_mask_enable_ && (robot_mask_mode_ == "mesh");

  // Spherical mask parameters
  get_or_default(node_, "one_arm_nbv_bt.sample.robot_mask.frames", robot_mask_frames_,
                 std::vector<std::string>{});
  get_or_default(node_, "one_arm_nbv_bt.sample.robot_mask.radii",  robot_mask_radii_,
                 std::vector<double>{});

  // Attached object mask (sphere at ee_link)
  get_or_default(node_, "one_arm_nbv_bt.ee_link", attached_mask_frame_, std::string("fr3_hand"));
  get_or_default(node_, "one_arm_nbv_bt.sample.attached_mask.enable", attached_mask_enable_, true);
  get_or_default(node_, "one_arm_nbv_bt.sample.attached_mask.radius", attached_mask_radius_, 0.05);

  // Mesh mask specifics
  get_or_default(node_, "one_arm_nbv_bt.sample.robot_mask.mesh_probe_radius", mesh_probe_radius_, 0.01);
  get_or_default(node_, "one_arm_nbv_bt.sample.joint_states_topic", joint_states_topic_, std::string("/joint_states"));

  if (mesh_mask_enable_) {
    // Build MoveIt model/scene from robot_description on THIS node
    try {
      robot_model_loader::RobotModelLoader rml(node_, "robot_description");
      robot_model_ = rml.getModel();
      if (!robot_model_) {
        RCLCPP_ERROR(node_->get_logger(),
                     "CameraSample(mesh): robot_description missing; fallback to spherical masks");
        mesh_mask_enable_ = false;
      } else {
        planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
        robot_state_    = std::make_shared<moveit::core::RobotState>(robot_model_);
        robot_state_->setToDefaultValues();
        acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(
                 planning_scene_->getAllowedCollisionMatrix());

        // Create a single probe sphere we move around
        probe_shape_ = std::shared_ptr<const shapes::Shape>(new shapes::Sphere(mesh_probe_radius_));
        planning_scene_->getWorldNonConst()->addToObject(probe_id_, probe_shape_, Eigen::Isometry3d::Identity());

        RCLCPP_INFO(node_->get_logger(),
          "CameraSample: Mesh mask enabled (probe radius=%.3f m)", mesh_probe_radius_);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "CameraSample(mesh) init failed: %s", e.what());
      mesh_mask_enable_ = false;
    }
  }


  // get_or_default(node_, "one_arm_nbv_bt.sample.save.enable",  save_enable_,  false);
  // get_or_default(node_, "one_arm_nbv_bt.sample.save.dir",     save_dir_,     std::string("/tmp"));
  // get_or_default(node_, "one_arm_nbv_bt.sample.save.prefix",  save_prefix_,  std::string("nbv_filtered"));
  // get_or_default(node_, "one_arm_nbv_bt.sample.save.ascii",   save_ascii_,   false);
}

bool CameraSampleAction::lookupFrame(const std::string& target,
                                     const std::string& source,
                                     geometry_msgs::msg::TransformStamped& T,
                                     const rclcpp::Duration& timeout)
{
  try {
    auto time    = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    T = tf_buffer_->lookupTransform(target, source, time, timeout);
    return true;
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(), "TF lookup failed %s <- %s : %s",
                target.c_str(), source.c_str(), e.what());
    return false;
  }
}

bool CameraSampleAction::waitOneCloud(sensor_msgs::msg::PointCloud2::SharedPtr& out,
                                      std::chrono::milliseconds timeout)
{
  std::mutex m; std::condition_variable cv; bool got=false;

  auto sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
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

  auto deadline = std::chrono::steady_clock::now() + timeout;
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

void CameraSampleAction::publishFiltered(const sensor_msgs::msg::PointCloud2& cloud_msg)
{
  static rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
  if (!pub) {
    pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(publish_topic_, 1);
  }
  pub->publish(cloud_msg);
}

bool CameraSampleAction::read_joint_state_once(sensor_msgs::msg::JointState& out,
                                               std::chrono::milliseconds timeout)
{
  std::mutex m; std::condition_variable cv; bool got=false;

  auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, rclcpp::SystemDefaultsQoS(),
    [&](const sensor_msgs::msg::JointState::SharedPtr msg){
      {
        std::lock_guard<std::mutex> lk(m);
        out = *msg;
        got = true;
      }
      cv.notify_one();
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);

  auto deadline = std::chrono::steady_clock::now() + timeout;
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

void CameraSampleAction::apply_joint_state(const sensor_msgs::msg::JointState& js)
{
  if (!robot_state_) return;
  const auto n = std::min(js.name.size(), js.position.size());
  for (size_t i=0; i<n; ++i) {
    robot_state_->setVariablePosition(js.name[i], js.position[i]);
  }
  robot_state_->update();
}

std::string CameraSampleAction::makePcdFilename(const std::string& frame)
{
  const auto now = node_->now();
  std::ostringstream oss;
  oss << save_prefix_ << "_" << now.seconds() << "_" << now.nanoseconds() << "_" << frame << ".pcd";
  std::filesystem::create_directories(save_dir_);
  return (std::filesystem::path(save_dir_) / oss.str()).string();
}

bool CameraSampleAction::saveCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                const std::string& frame,
                std::string& out_path)
{
  if (cloud.empty()) return false;
  out_path = makePcdFilename(frame);
  int rc = save_ascii_
           ? pcl::io::savePCDFileASCII(out_path, cloud)
           : pcl::io::savePCDFileBinaryCompressed(out_path, cloud);
  if (rc != 0) {
    RCLCPP_WARN(node_->get_logger(), "CameraSample: failed to save PCD to %s (rc=%d)", out_path.c_str(), rc);
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "CameraSample: saved PCD to %s (%zu pts)", out_path.c_str(), cloud.size());
  return true;
}

// bool CameraSampleAction::callNbvService(sensor_msgs::msg::PointCloud2& out_pc,
//                      const geometry_msgs::msg::Pose& current_camera_pose,
//                      utils_msgs::srv::NBVTrigger::Response::SharedPtr& out_res)
// {
//   if (!nbv_client_) return false;
//   if (!nbv_client_->wait_for_service(std::chrono::seconds(2))) {
//     RCLCPP_WARN(node_->get_logger(), "CameraSample: service %s not available", nbv_service_name_.c_str());
//     return false;
//   }

//   auto req = std::make_shared<utils_msgs::srv::NBVTrigger::Request>();
//   req->current_fused_point_cloud = out_pc;
//   req->current_camera_pose = current_camera_pose;

//   auto fut = nbv_client_->async_send_request(req);
//   auto rc = rclcpp::spin_until_future_complete(node_, fut, std::chrono::seconds(30));
//   if (rc != rclcpp::FutureReturnCode::SUCCESS) {
//     RCLCPP_ERROR(node_->get_logger(), "CameraSample: NBV service call failed / timed out");
//     return false;
//   }

//   out_res = fut.get();
//   if (!out_res) {
//     RCLCPP_ERROR(node_->get_logger(), "CameraSample: NBV service returned null response");
//     return false;
//   }

//   RCLCPP_INFO(node_->get_logger(),
//               "NBV result: is_terminated=%d, result=%ld, voxels=%ld, ellipsoids=%ld",
//               (int)out_res->is_terminated, (long)out_res->result,
//               (long)out_res->voxel_map_size, (long)out_res->ellipsoids_size);
//   return true;
// }

BT::NodeStatus CameraSampleAction::tick()
{
  RCLCPP_INFO(node_->get_logger(), "CameraSample: waiting one cloud on %s", cloud_topic_.c_str());

  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg;
  if (!waitOneCloud(cloud_msg, 5000ms)) {
    RCLCPP_WARN(node_->get_logger(), "CameraSample: timeout waiting for pointcloud");
    return BT::NodeStatus::FAILURE;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud_cam;
  pcl::fromROSMsg(*cloud_msg, cloud_cam);
  if (cloud_cam.empty()) {
    RCLCPP_WARN(node_->get_logger(), "CameraSample: received empty cloud");
    return BT::NodeStatus::FAILURE;
  }

  // Transform to working frame
  const std::string src_frame = cloud_msg->header.frame_id;
  geometry_msgs::msg::TransformStamped T_w_c;
  Eigen::Isometry3d iso_w_c = Eigen::Isometry3d::Identity();
  std::string work_frame = working_frame_;
  if (work_frame.empty()) work_frame = src_frame;

  if (work_frame != src_frame) {
    if (!lookupFrame(work_frame, src_frame, T_w_c, rclcpp::Duration::from_seconds(0.2))) {
      RCLCPP_WARN(node_->get_logger(), "CameraSample: no TF %s<- %s; using sensor frame",
                  work_frame.c_str(), src_frame.c_str());
      work_frame = src_frame;
    } else {
      iso_w_c = tf2::transformToEigen(T_w_c);
    }
  }

  Eigen::Matrix4f T = iso_w_c.matrix().cast<float>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_w(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(cloud_cam, *cloud_w, T);

  // Downsample
  if (voxel_leaf_ > 1e-6) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
    vg.setInputCloud(cloud_w);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*tmp);
    cloud_w.swap(tmp);
  }

  // Prepare masks
  struct Sphere { Eigen::Vector3f c; float r; };
  std::vector<Sphere> masks;

  // Spherical robot link masks
  if (robot_mask_enable_ && !mesh_mask_enable_) {
    size_t n = std::min(robot_mask_frames_.size(), robot_mask_radii_.size());
    for (size_t i=0; i<n; ++i) {
      if (robot_mask_radii_[i] <= 0.0) continue;
      geometry_msgs::msg::TransformStamped T_w_f;
      if (lookupFrame(work_frame, robot_mask_frames_[i], T_w_f, rclcpp::Duration::from_seconds(0.1))) {
        Eigen::Isometry3d iso = tf2::transformToEigen(T_w_f);
        masks.push_back({ iso.translation().cast<float>(), static_cast<float>(robot_mask_radii_[i]) });
      }
    }
  }

  // Attached-object mask (sphere at ee_link)
  if (attached_mask_enable_ && attached_mask_radius_ > 0.0) {
    geometry_msgs::msg::TransformStamped T_w_tcp;
    if (lookupFrame(work_frame, attached_mask_frame_, T_w_tcp, rclcpp::Duration::from_seconds(0.1))) {
      Eigen::Isometry3d iso = tf2::transformToEigen(T_w_tcp);
      masks.push_back({ iso.translation().cast<float>(), static_cast<float>(attached_mask_radius_) });
    }
  }

  // ROI sphere center (if enabled)
  Eigen::Vector3f roi_center = Eigen::Vector3f::Zero();
  float roi_radius = static_cast<float>(roi_sphere_radius_);
  bool have_roi_center = false;
  if (roi_sphere_enable_ && !roi_sphere_frame_.empty()) {
    geometry_msgs::msg::TransformStamped T_w_roi;
    if (lookupFrame(work_frame, roi_sphere_frame_, T_w_roi, rclcpp::Duration::from_seconds(0.1))) {
      Eigen::Isometry3d iso = tf2::transformToEigen(T_w_roi);
      roi_center = iso.translation().cast<float>();
      have_roi_center = true;
    }
  }

  // If mesh mask: update robot_state from /joint_states
  if (mesh_mask_enable_ && planning_scene_ && robot_state_) {
    sensor_msgs::msg::JointState js;
    if (read_joint_state_once(js, 200ms)) {
      apply_joint_state(js);
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "CameraSample(mesh): no joint_states; using last state");
    }
  }

  // Filter loop
  pcl::PointCloud<pcl::PointXYZ> cloud_out;
  cloud_out.header = cloud_w->header;
  cloud_out.reserve(cloud_w->size());

  const float z_cut = static_cast<float>(desk_height_ + desk_margin_);

  for (const auto& p : cloud_w->points)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;

    // Desk removal: keep only points above plane z > height+margin
    if (desk_enable_ && p.z <= z_cut)
      continue;

    // ROI box (keep inside)
    if (roi_box_enable_) {
      if (p.x < roi_min_[0] || p.x > roi_max_[0] ||
          p.y < roi_min_[1] || p.y > roi_max_[1] ||
          p.z < roi_min_[2] || p.z > roi_max_[2]) {
        continue;
      }
    }

    // ROI sphere (keep inside)
    if (roi_sphere_enable_ && have_roi_center) {
      Eigen::Vector3f v(p.x, p.y, p.z);
      if ((v - roi_center).squaredNorm() > roi_radius * roi_radius)
        continue;
    }

    // Robot/object masking
    bool masked = false;

    if (mesh_mask_enable_ && planning_scene_ && robot_state_ && probe_shape_) {
      // Move probe to point and check collision against robot
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() = Eigen::Vector3d(p.x, p.y, p.z);
      planning_scene_->getWorldNonConst()->moveObject(probe_id_, pose);

      collision_detection::CollisionRequest req;
      collision_detection::CollisionResult  res;
      req.contacts = false; req.max_contacts = 0;
      planning_scene_->checkCollision(req, res, *robot_state_);
      masked = res.collision;
    } else {
      // Spherical masks
      for (const auto& s : masks) {
        Eigen::Vector3f v(p.x, p.y, p.z);
        if ((v - s.c).squaredNorm() <= s.r * s.r) { masked = true; break; }
      }
    }

    if (masked) continue;
    cloud_out.push_back(p);
  }

  // Publish result
  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(cloud_out, out_msg);
  out_msg.header.stamp = node_->now();
  out_msg.header.frame_id = work_frame;
  publishFiltered(out_msg);

  auto out_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(out_msg);
  (void)setOutput("cloud", out_ptr);

  RCLCPP_INFO(node_->get_logger(),
    "CameraSample: in=%zu -> out=%zu (frame=%s, desk=%s, roi_box=%s, roi_sphere=%s, mask=%s)",
    cloud_w->size(), cloud_out.size(), work_frame.c_str(),
    desk_enable_?"on":"off",
    roi_box_enable_?"on":"off",
    (roi_sphere_enable_&&have_roi_center)?"on":"off",
    mesh_mask_enable_?"mesh":"spheres");

  rclcpp::sleep_for(5s);

  return (cloud_out.empty() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS);
}

} // namespace bt_nodes