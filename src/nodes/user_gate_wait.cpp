#include "nodes/user_gate_wait.hpp"
#include <one_arm_nbv_bt/param_utils.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <mutex>
#include <condition_variable>

using nbv_param::get_or_default;

namespace bt_nodes {

UserGateWait::UserGateWait(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  // defaults from YAML (optional)
  get_or_default(node_, "one_arm_nbv_bt.user_gate.topic", topic_, std::string("/nbv/next"));
  get_or_default(node_, "one_arm_nbv_bt.user_gate.expect", expect_, std::string("next"));
  get_or_default(node_, "one_arm_nbv_bt.user_gate.timeout_ms", timeout_ms_, 15000);
}

BT::NodeStatus UserGateWait::tick()
{
  // override from ports if provided
  (void)getInput("topic", topic_);
  (void)getInput("expect", expect_);
  (void)getInput("timeout_ms", timeout_ms_);

  std::mutex m; std::condition_variable cv; bool ok=false;

  auto sub = node_->create_subscription<std_msgs::msg::String>(
    topic_, rclcpp::QoS(1).transient_local(),   // allow latched one-shot
    [&](const std_msgs::msg::String::SharedPtr msg){
      if (msg && msg->data == expect_) {
        {
          std::lock_guard<std::mutex> lk(m);
          ok = true;
        }
        cv.notify_one();
      }
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);

  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);
  RCLCPP_INFO(node_->get_logger(), "UserGateWait: waiting on %s for '%s' (%d ms)",
              topic_.c_str(), expect_.c_str(), timeout_ms_);

  while (std::chrono::steady_clock::now() < deadline)
  {
    exec.spin_some();
    {
      std::lock_guard<std::mutex> lk(m);
      if (ok) {
        RCLCPP_INFO(node_->get_logger(), "UserGateWait: received '%s'", expect_.c_str());
        return BT::NodeStatus::SUCCESS;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_WARN(node_->get_logger(), "UserGateWait: timeout");
  return BT::NodeStatus::FAILURE;
}

} // namespace bt_nodes