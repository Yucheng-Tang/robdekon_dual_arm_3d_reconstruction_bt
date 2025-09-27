#include "nodes/evaluate_reconstruction_action.hpp"
#include <one_arm_nbv_bt/param_utils.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>  // loadPolygonFileSTL (requires PCL built with VTK)

#include <random>
#include <fstream>
#include <algorithm>
#include <cctype>
#include <chrono>

using nbv_param::get_or_default;

namespace bt_nodes {

using namespace std::chrono_literals;

// ================== Utilities ==================
static inline std::string to_lower(std::string s){
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c){ return std::tolower(c); });
  return s;
}

std::string EvaluateReconstructionAction::guessTypeFromExt(const std::string& path){
  auto p = path;
  auto pos = p.find_last_of('.');
  if (pos == std::string::npos) return "";
  auto ext = to_lower(p.substr(pos+1));
  if (ext == "pcd") return "pcd";
  if (ext == "stl") return "stl";
  return "";
}

EvaluateReconstructionAction::EvaluateReconstructionAction(
    const std::string& name, const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // YAML defaults (params are not declared here; we assume autodeclare in main)
  get_or_default(node_, "one_arm_nbv_bt.fuse.map_frame",     map_frame_,     std::string("base"));
  get_or_default(node_, "one_arm_nbv_bt.fuse.map_topic",     map_topic_,     std::string("/nbv/global_map"));

  get_or_default(node_, "one_arm_nbv_bt.eval.reference_file", reference_file_, std::string(""));
  get_or_default(node_, "one_arm_nbv_bt.eval.reference_type", reference_type_, std::string("auto"));
  get_or_default(node_, "one_arm_nbv_bt.eval.sample_points",  sample_points_,  100000);
  get_or_default(node_, "one_arm_nbv_bt.eval.leaf_map",       leaf_map_,       0.0);
  get_or_default(node_, "one_arm_nbv_bt.eval.leaf_ref",       leaf_ref_,       0.0);
  get_or_default(node_, "one_arm_nbv_bt.eval.dist_thresh",    dist_thresh_,    0.01);

  RCLCPP_INFO(node_->get_logger(),
    "Eval params: ref='%s' type='%s' leaf_map=%.4f leaf_ref=%.4f dist_thresh=%.3f sample_points=%d map_topic=%s",
    reference_file_.c_str(), reference_type_.c_str(), leaf_map_, leaf_ref_, dist_thresh_,
    sample_points_, map_topic_.c_str());
}

// ---- Conversions / Filtering ----
EvaluateReconstructionAction::CloudPtr
EvaluateReconstructionAction::toPCL(const sensor_msgs::msg::PointCloud2& msg)
{
  CloudPtr c(new Cloud);
  pcl::fromROSMsg(msg, *c);
  return c;
}

void EvaluateReconstructionAction::voxelDownsample(CloudPtr& cloud, double leaf)
{
  if (!cloud || cloud->empty() || leaf <= 1e-9) return;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(leaf, leaf, leaf);
  vg.setInputCloud(cloud);
  CloudPtr tmp(new Cloud);
  vg.filter(*tmp);
  cloud.swap(tmp);
}

// ---- Loaders ----
EvaluateReconstructionAction::CloudPtr
EvaluateReconstructionAction::loadReferencePCD(const std::string& path)
{
  CloudPtr c(new Cloud);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *c) != 0) {
    throw std::runtime_error("Failed to load PCD: " + path);
  }
  return c;
}

EvaluateReconstructionAction::CloudPtr
EvaluateReconstructionAction::loadReferenceSTL(const std::string& path, int n_samples)
{
  pcl::PolygonMesh mesh;
  if (pcl::io::loadPolygonFileSTL(path, mesh) == 0) {
    throw std::runtime_error("Failed to load STL: " + path);
  }

  // Extract vertices and triangles
  pcl::PointCloud<pcl::PointXYZ> verts;
  pcl::fromPCLPointCloud2(mesh.cloud, verts);

  // Build triangle list
  struct Tri { int a,b,c; float area; };
  std::vector<Tri> tris;
  tris.reserve(mesh.polygons.size());
  double total_area = 0.0;

  for (const auto& poly : mesh.polygons) {
    if (poly.vertices.size() < 3) continue;
    for (size_t i=1; i+1<poly.vertices.size(); ++i) {
      int ia = poly.vertices[0], ib = poly.vertices[i], ic = poly.vertices[i+1];
      const auto &A=verts[ia], &B=verts[ib], &C=verts[ic];
      Eigen::Vector3f a(A.x,A.y,A.z), b(B.x,B.y,B.z), c(C.x,C.y,C.z);
      float area = 0.5f * ((b-a).cross(c-a)).norm();
      if (area <= 1e-12f) continue;
      tris.push_back({ia,ib,ic,area});
      total_area += area;
    }
  }
  if (tris.empty() || total_area <= 0.0) {
    throw std::runtime_error("STL has no valid triangles: " + path);
  }

  // CDF by area
  std::vector<double> cdf(tris.size());
  double run = 0.0;
  for (size_t i=0;i<tris.size();++i){ run += tris[i].area; cdf[i]=run; }
  for (auto& x : cdf) x /= run;

  // Sample points
  std::mt19937 rng{std::random_device{}()};
  std::uniform_real_distribution<double> U(0.0,1.0);

  CloudPtr out(new Cloud);
  out->reserve(std::max(1, n_samples));

  auto pick_tri = [&](double u){
    auto it = std::lower_bound(cdf.begin(), cdf.end(), u);
    size_t idx = std::min<size_t>(cdf.size()-1, std::distance(cdf.begin(), it));
    return idx;
  };

  for (int i=0; i<n_samples; ++i) {
    size_t k = pick_tri(U(rng));
    const auto t = tris[k];
    const auto &A=verts[t.a], &B=verts[t.b], &C=verts[t.c];
    Eigen::Vector3f a(A.x,A.y,A.z), b(B.x,B.y,B.z), c(C.x,C.y,C.z);
    // Uniform barycentric
    double r1 = std::sqrt(U(rng));
    double r2 = U(rng);
    Eigen::Vector3f p = (1-r1)*a + r1*( (1-r2)*b + r2*c );
    out->push_back(pcl::PointXYZ{p.x(),p.y(),p.z()});
  }
  return out;
}

EvaluateReconstructionAction::CloudPtr
EvaluateReconstructionAction::getOrLoadReference(const std::string& file,
                                                 const std::string& type,
                                                 int n_samples)
{
  std::lock_guard<std::mutex> lk(ref_mtx_);
  if (!ref_cloud_ || file != ref_loaded_from_) {
    const std::string use_type =
      (type=="auto" || type.empty()) ? guessTypeFromExt(file) : to_lower(type);

    if (use_type == "pcd") {
      ref_cloud_ = loadReferencePCD(file);
    } else if (use_type == "stl") {
      ref_cloud_ = loadReferenceSTL(file, n_samples);
    } else {
      throw std::runtime_error("Unknown/unsupported reference type for: " + file);
    }
    ref_loaded_from_ = file;
    voxelDownsample(ref_cloud_, leaf_ref_);
    RCLCPP_INFO(node_->get_logger(), "Eval: loaded reference (%s), points=%zu",
                use_type.c_str(), ref_cloud_->size());
  }
  return ref_cloud_;
}

// ---- One-shot subscription fallback ----
bool EvaluateReconstructionAction::waitOneCloudFromTopic(
    const std::string& topic,
    sensor_msgs::msg::PointCloud2::SharedPtr& out,
    std::chrono::milliseconds timeout)
{
  std::mutex m; std::condition_variable cv; bool got=false;
  auto sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic, rclcpp::SensorDataQoS(),
    [&](sensor_msgs::msg::PointCloud2::SharedPtr msg){
      { std::lock_guard<std::mutex> lk(m); out = std::move(msg); got=true; }
      cv.notify_one();
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline){
    exec.spin_some();
    { std::lock_guard<std::mutex> lk(m); if (got) return true; }
    std::this_thread::sleep_for(5ms);
  }
  return false;
}

// ---- Nearest-neighbor stats ----
EvaluateReconstructionAction::NNStats
EvaluateReconstructionAction::oneWayStats(const Cloud& A, const Cloud& B, double thresh_m)
{
  NNStats s;
  if (A.empty() || B.empty()) return s;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdt;
  kdt.setInputCloud(B.makeShared());

  double sum = 0.0, sum2 = 0.0; double mx = 0.0;
  size_t within = 0, n = 0;

  std::vector<int> idx(1); std::vector<float> d2(1);

  for (const auto& p : A.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (kdt.nearestKSearch(p, 1, idx, d2) != 1) continue;
    double d = std::sqrt(d2[0]);
    sum += d; sum2 += d*d; mx = std::max(mx, d);
    if (d <= thresh_m) within++;
    n++;
  }

  if (n>0){
    s.mean = sum/static_cast<double>(n);
    s.rmse = std::sqrt(sum2/static_cast<double>(n));
    s.max  = mx;
    s.within_thresh_ratio = static_cast<double>(within)/static_cast<double>(n);
    s.n = n;
  }
  return s;
}

// ---- Logging ----
void EvaluateReconstructionAction::logStats(const rclcpp::Logger& log,
                                            const char* tag, const Cloud& c)
{
  if (c.empty()){
    RCLCPP_INFO(log, "%s: empty cloud", tag);
    return;
  }
  Eigen::Vector3f mn( std::numeric_limits<float>::infinity(),
                      std::numeric_limits<float>::infinity(),
                      std::numeric_limits<float>::infinity() );
  Eigen::Vector3f mx(-mn.x(), -mn.y(), -mn.z());
  size_t finite=0, nan=0;
  for (const auto& p : c.points){
    bool ok = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    if (!ok) { nan++; continue; }
    finite++;
    mn.x() = std::min(mn.x(), p.x); mn.y() = std::min(mn.y(), p.y); mn.z() = std::min(mn.z(), p.z);
    mx.x() = std::max(mx.x(), p.x); mx.y() = std::max(mx.y(), p.y); mx.z() = std::max(mx.z(), p.z);
  }
  RCLCPP_INFO(log, "%s: size=%zu finite=%zu nan=%zu bounds[min(%.3f %.3f %.3f) max(%.3f %.3f %.3f)]",
              tag, c.size(), finite, nan, mn.x(), mn.y(), mn.z(), mx.x(), mx.y(), mx.z());
}

// ================== tick() ==================
BT::NodeStatus EvaluateReconstructionAction::tick()
{
  // 1) Check toggle (BT > YAML)
  bool do_eval = false;
  (void)getInput("do_eval", do_eval);
  if (!do_eval) {
    RCLCPP_INFO(node_->get_logger(), "Eval: do_eval=false -> skipping (SUCCESS).");
    return BT::NodeStatus::SUCCESS;
  }

  // 2) Get reference file / params (BT overrides YAML)
  std::string ref_file = reference_file_;
  (void)getInput("reference_file", ref_file);
  double dist_thresh = dist_thresh_;
  (void)getInput("dist_thresh", dist_thresh);

  if (ref_file.empty()){
    RCLCPP_ERROR(node_->get_logger(), "Eval: reference_file is empty.");
    return BT::NodeStatus::FAILURE;
  }

  // 3) Get map cloud (BT port preferred)
  sensor_msgs::msg::PointCloud2::SharedPtr map_ros;
  if (!(getInput("map_cloud", map_ros) && map_ros)) {
    RCLCPP_INFO(node_->get_logger(),
      "Eval: no 'map_cloud' on BT port; waiting on topic '%s'...", map_topic_.c_str());
    if (!waitOneCloudFromTopic(map_topic_, map_ros, 2000ms) || !map_ros){
      RCLCPP_ERROR(node_->get_logger(), "Eval: no map cloud available.");
      return BT::NodeStatus::FAILURE;
    }
  }

  // 4) Convert & filter map
  CloudPtr map = toPCL(*map_ros);
  voxelDownsample(map, leaf_map_);
  logStats(node_->get_logger(), "map", *map);

  if (map->empty()){
    RCLCPP_ERROR(node_->get_logger(), "Eval: map cloud is empty.");
    return BT::NodeStatus::FAILURE;
  }

  // 5) Load / prepare reference
  CloudPtr ref;
  try {
    ref = getOrLoadReference(ref_file, reference_type_, sample_points_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Eval: load reference failed: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
  if (!ref || ref->empty()){
    RCLCPP_ERROR(node_->get_logger(), "Eval: reference cloud is empty.");
    return BT::NodeStatus::FAILURE;
  }
  logStats(node_->get_logger(), "ref", *ref);

  // 6) Metrics (both ways)
  auto st_map_ref = oneWayStats(*map, *ref, dist_thresh);
  auto st_ref_map = oneWayStats(*ref, *map, dist_thresh);

  const double chamfer = st_map_ref.mean + st_ref_map.mean;

  RCLCPP_INFO(node_->get_logger(),
    "Eval metrics (th=%.3f m):\n"
    "  map->ref: n=%zu  mean=%.4f  rmse=%.4f  max=%.4f  within=%.2f%%\n"
    "  ref->map: n=%zu  mean=%.4f  rmse=%.4f  max=%.4f  within=%.2f%%\n"
    "  chamfer_mean = %.6f",
    dist_thresh,
    st_map_ref.n, st_map_ref.mean, st_map_ref.rmse, st_map_ref.max, 100.0*st_map_ref.within_thresh_ratio,
    st_ref_map.n, st_ref_map.mean, st_ref_map.rmse, st_ref_map.max, 100.0*st_ref_map.within_thresh_ratio,
    chamfer);

  // 7) Outputs
  setOutput("rmse_map_to_ref",   st_map_ref.rmse);
  setOutput("rmse_ref_to_map",   st_ref_map.rmse);
  setOutput("mean_map_to_ref",   st_map_ref.mean);
  setOutput("mean_ref_to_map",   st_ref_map.mean);
  setOutput("chamfer_mean",      chamfer);
  setOutput("accuracy",          st_map_ref.within_thresh_ratio);
  setOutput("completeness",      st_ref_map.within_thresh_ratio);

  return BT::NodeStatus::SUCCESS;
}

} // namespace bt_nodes