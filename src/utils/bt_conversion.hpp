#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

// #include <utils_msgs/srv/get_pose.hpp>
// #include <utils_msgs/srv/get_joint_value.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace bt_nodes {

// ------------------ small helpers ------------------
inline void print_error(const std::string& s){ std::cerr << "\033[31m" << s << "\033[0m\n"; }
inline void print_warn (const std::string& s){ std::cout  << "\033[33m" << s << "\033[0m\n"; }
inline void print_success(const std::string& s){ std::cout<< "\033[32m" << s << "\033[0m\n"; }

template<typename T>
inline void get_or_default(const rclcpp::Node::SharedPtr& node,
                           const std::string& name, T& out, const T& def)
{
  if (!node->get_parameter(name, out)) out = def;  // works even if param is undeclared (returns false)
}

inline std::vector<double> parse_numbers(const std::string& s)
{
  std::string t = s;
  std::replace(t.begin(), t.end(), ',', ' ');
  std::replace(t.begin(), t.end(), '[', ' ');
  std::replace(t.begin(), t.end(), ']', ' ');
  std::istringstream iss(t);
  std::vector<double> out; double v;
  while (iss >> std::setprecision(16) >> v) out.push_back(v);
  return out;
}

// ------------------ parsing / formatting ------------------
inline geometry_msgs::msg::Pose analyseTargetPose(const std::string& target_pose)
/* expects "[x y z w x y z]" */ {
  geometry_msgs::msg::Pose pose{};
  auto data = parse_numbers(target_pose);
  if (data.size() != 7){
    print_error("analyseTargetPose(): expected 7 values [x y z w x y z], got "
                + std::to_string(data.size()) + " from \"" + target_pose + "\"");
    return pose;
  }
  pose.position.x = data[0]; pose.position.y = data[1]; pose.position.z = data[2];
  pose.orientation.w = data[3]; pose.orientation.x = data[4];
  pose.orientation.y = data[5]; pose.orientation.z = data[6];
  return pose;
}

inline std::vector<double> analyseJointValue(const std::string& s){ return parse_numbers(s); }
inline std::vector<double> analyseDataArray(const std::string& s){ return parse_numbers(s); }
// back-compat for your typo’d name
inline std::vector<double> analyseDataArry(const std::string& s){ return parse_numbers(s); }

inline std::string jointValueToString(const std::vector<double>& v)
{
  std::ostringstream ss; ss << "[";
  for (size_t i=0;i<v.size();++i){ if(i) ss<<" "; ss<<std::setprecision(16)<<v[i]; }
  ss << "]"; return ss.str();
}
// back-compat
inline std::string jointValueTostring(const std::vector<double>& v){ return jointValueToString(v); }

inline std::string poseToString(const geometry_msgs::msg::Pose& p)
{
  std::ostringstream ss;
  ss << "[" << p.position.x << " " << p.position.y << " " << p.position.z << " "
     << p.orientation.x << " " << p.orientation.y << " " << p.orientation.z << " " << p.orientation.w << "]";
  return ss.str();
}

// ------------------ transforms ------------------
inline Eigen::Matrix4d poseToMatrix(const geometry_msgs::msg::Pose& p)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  T.topLeftCorner<3,3>() = q.toRotationMatrix();
  T(0,3)=p.position.x; T(1,3)=p.position.y; T(2,3)=p.position.z;
  return T;
}

inline geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4d& T)
{
  geometry_msgs::msg::Pose p{};
  Eigen::Quaterniond q(T.topLeftCorner<3,3>());
  p.orientation.w=q.w(); p.orientation.x=q.x(); p.orientation.y=q.y(); p.orientation.z=q.z();
  p.position.x=T(0,3); p.position.y=T(1,3); p.position.z=T(2,3);
  return p;
}

// ------------------ math ------------------
inline double get_angle(const geometry_msgs::msg::Pose& cur,
                        const geometry_msgs::msg::Pose& tgt)
{
  const double a = std::atan2(tgt.position.y, tgt.position.x)
                 - std::atan2(cur.position.y, cur.position.x);
  if (a < -M_PI) return a + 2.0*M_PI;
  if (a >  M_PI) return a - 2.0*M_PI;
  return a;
}
// back-compat alias
inline double get_angel(const geometry_msgs::msg::Pose& cur,
                        const geometry_msgs::msg::Pose& tgt){ return get_angle(cur,tgt); }

// ------------------ parameter → PoseStamped ------------------
inline bool get_pose_param(const rclcpp::Node::SharedPtr& node,
                           geometry_msgs::msg::PoseStamped& out,
                           const std::string& ns = "bt_node",
                           const std::string& frame_id_def = "base",
                           const std::vector<double>& xyz_def = {0.4, 0.0, 0.35},
                           const std::vector<double>& rpy_def = {M_PI, 0.0, 0.0})
{
  std::string frame_id; std::vector<double> xyz, rpy;
  get_or_default(node, ns + ".frame_id", frame_id, frame_id_def);
  get_or_default(node, ns + ".xyz",      xyz,      xyz_def);
  get_or_default(node, ns + ".rpy",      rpy,      rpy_def);

  if (xyz.size()!=3 || rpy.size()!=3){
    print_error("get_pose_param(): bad xyz/rpy sizes under ns='" + ns + "'");
    return false;
  }

  out.header.frame_id = frame_id;
  out.pose.position.x = xyz[0];
  out.pose.position.y = xyz[1];
  out.pose.position.z = xyz[2];

  tf2::Quaternion q; q.setRPY(rpy[0], rpy[1], rpy[2]);
  out.pose.orientation.x = q.x();
  out.pose.orientation.y = q.y();
  out.pose.orientation.z = q.z();
  out.pose.orientation.w = q.w();
  return true;
}

// ------------------ ROS 2 services ------------------
// inline bool callGetPose(const rclcpp::Node::SharedPtr& node,
//                         const std::string& link_name,
//                         geometry_msgs::msg::Pose& pose)
// {
//   using Service = utils_msgs::srv::GetPose;
//   auto client = node->create_client<Service>("GetPose");
//   if (!client->wait_for_service(std::chrono::seconds(2))){
//     RCLCPP_ERROR(node->get_logger(), "Service 'GetPose' not available");
//     return false;
//   }
//   auto req = std::make_shared<Service::Request>(); req->link_name = link_name;
//   auto fut = client->async_send_request(req);
//   auto rc  = rclcpp::spin_until_future_complete(node, fut, std::chrono::seconds(5));
//   if (rc != rclcpp::FutureReturnCode::SUCCESS){
//     RCLCPP_ERROR(node->get_logger(), "GetPose call failed");
//     return false;
//   }
//   pose = fut.get()->pose; return true;
// }

// inline bool callGetJointValue(const rclcpp::Node::SharedPtr& node,
//                               std::vector<double>& joint_value)
// {
//   using Service = utils_msgs::srv::GetJointValue;
//   auto client = node->create_client<Service>("GetJointValue");
//   if (!client->wait_for_service(std::chrono::seconds(2))){
//     RCLCPP_ERROR(node->get_logger(), "Service 'GetJointValue' not available");
//     return false;
//   }
//   auto req = std::make_shared<Service::Request>();
//   auto fut = client->async_send_request(req);
//   auto rc  = rclcpp::spin_until_future_complete(node, fut, std::chrono::seconds(5));
//   if (rc != rclcpp::FutureReturnCode::SUCCESS){
//     RCLCPP_ERROR(node->get_logger(), "GetJointValue call failed");
//     return false;
//   }
//   joint_value = fut.get()->joint_value; return true;
// }

} // namespace bt_node