#pragma once
#include <moveit/kinematic_constraints/utils.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace utils {
inline moveit_msgs::msg::Constraints ee_goal(const std::string& ee_link,
                                             const geometry_msgs::msg::PoseStamped& p,
                                             double pos_tol=0.005, double rot_tol=0.01) {
  return kinematic_constraints::constructGoalConstraints(ee_link, p, pos_tol, rot_tol);
}
}  // namespace utils