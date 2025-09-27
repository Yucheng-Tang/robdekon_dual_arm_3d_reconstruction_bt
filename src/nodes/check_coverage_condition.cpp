#include "check_coverage_condition.hpp"
#include <one_arm_nbv_bt/param_utils.hpp> 
#include <cmath>
#include <limits>

using nbv_param::get_or_default;

namespace bt_nodes {

CheckCoverageCondition::CheckCoverageCondition(const std::string& name,
                                               const BT::NodeConfiguration& cfg)
: BT::ConditionNode(name, cfg)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  get_or_default(node_, "one_arm_nbv_bt.coverage_threshold", threshold_, 0.90);

  RCLCPP_INFO(node_->get_logger(),
              "[CheckCoverage] using threshold param: %.3f", threshold_);
}

BT::NodeStatus CheckCoverageCondition::tick()
{
  double threshold = threshold_;
  (void) getInput("threshold", threshold);

  double covered = std::numeric_limits<double>::quiet_NaN();
  if (auto cov = getInput<double>("coverage"))
  {
    covered = *cov;
  }
  else
  {
    get_or_default(node_, "one_arm_nbv_bt.coverage", covered,
                   std::numeric_limits<double>::quiet_NaN());
  }

  if (!std::isfinite(covered))
  {
    RCLCPP_DEBUG(node_->get_logger(),
                 "[CheckCoverage] no coverage value available yet");
    return BT::NodeStatus::FAILURE; 
  }

  RCLCPP_INFO(node_->get_logger(),
              "[CheckCoverage] Coverage: %.3f / %.3f (%.1f%%)",
              covered, threshold, 100.0 * covered);

  if (covered >= threshold)
  {
    RCLCPP_INFO_ONCE(node_->get_logger(),
                     "[CheckCoverage] Target coverage reached (>= %.1f%%) ✅",
                     100.0 * threshold);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                       "[CheckCoverage] Coverage below threshold: %.1f%% < %.1f%%",
                       100.0 * covered, 100.0 * threshold);
  return BT::NodeStatus::FAILURE;
}

}  // namespace bt_nodes