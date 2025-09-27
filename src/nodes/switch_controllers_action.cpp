#include "switch_contorllers_action.hpp"
#include <one_arm_nbv_bt/param_utils.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>  

using nbv_param::get_or_default;

namespace bt_nodes {

SwitchControllersAction::SwitchControllersAction(const std::string& name,
                                                 const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Load defaults from YAML (optional)
  get_or_default(node_, "one_arm_nbv_bt.controllers.cm_ns", cm_ns_, std::string(""));
  get_or_default(node_, "one_arm_nbv_bt.controllers.activate", activate_, std::string{});
  get_or_default(node_, "one_arm_nbv_bt.controllers.deactivate", deactivate_, std::string{});
  get_or_default(node_, "one_arm_nbv_bt.controllers.strictness", strictness_, 2);
  get_or_default(node_, "one_arm_nbv_bt.controllers.activate_asap", activate_asap_, true);
  get_or_default(node_, "one_arm_nbv_bt.controllers.timeout_s", timeout_s_, 5.0);
}

bool SwitchControllersAction::switchControllers(const std::string& cm_ns,
                                                const std::string& act,
                                                const std::string& deact,
                                                int strictness,
                                                bool activate_asap,
                                                double timeout_s)
{
  auto service_name = cm_ns;
  if (!service_name.empty() && service_name.back() != '/')
    service_name += '/';
  service_name += "controller_manager/switch_controller";

  auto client = node_->create_client<controller_manager_msgs::srv::SwitchController>(service_name);

  if (!client->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(), "SwitchControllers: service %s unavailable", service_name.c_str());
    return false;
  }

  auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  if (!act.empty())   req->activate_controllers   = {act};
  if (!deact.empty()) req->deactivate_controllers = {deact};
  req->strictness = strictness;          // 1 BEST_EFFORT, 2 STRICT
  req->activate_asap = activate_asap;
  req->timeout = rclcpp::Duration::from_seconds(timeout_s);

  RCLCPP_INFO(node_->get_logger(),
              "[SwitchControllers] start='%s' stop='%s' strict=%d asap=%s timeout=%.2f",
              act.c_str(), deact.c_str(), strictness,
              activate_asap ? "true" : "false", timeout_s);

  auto future = client->async_send_request(req);
  auto rc = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(10));
  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "SwitchControllers: call failed");
    return false;
  }

    const auto resp = future.get();
    if (!resp->ok) {
      RCLCPP_ERROR(node_->get_logger(), "[SwitchControllers] switch rejected");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[SwitchControllers] OK");
    return true;
}

std::string SwitchControllersAction::getControllerState(const std::string& cm_ns,
                                                        const std::string& name)
{
  if (name.empty()) return {};

  std::string service = cm_ns;
  if (!service.empty() && service.back() != '/')
    service += '/';
  service += "controller_manager/list_controllers";

  auto client = node_->create_client<controller_manager_msgs::srv::ListControllers>(service);
  if (!client->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_WARN(node_->get_logger(), "ListControllers: service %s unavailable", service.c_str());
    return {};
  }

  auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto future = client->async_send_request(req);
  auto rc = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(10));
  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(node_->get_logger(), "ListControllers: call failed");
    return {};
  }

  const auto resp = future.get();
  for (const auto& c : resp->controller) {
    if (c.name == name) {
      // c.state is typically: "active", "inactive", "unconfigured", "finalized"
      return c.state;
    }
  }
  return {}; // not found
}

BT::NodeStatus SwitchControllersAction::tick()
{
  // Start with defaults from YAML
  std::string cm_ns = cm_ns_;
  std::string act = activate_;
  std::string deact = deactivate_;
  int strict = strictness_;
  bool asap = activate_asap_;
  double tout = timeout_s_;

  // Override from BT ports if provided
  // (void)getInput("cm_ns", cm_ns);
  (void)getInput("activate", act);
  (void)getInput("deactivate", deact);
  // (void)getInput("strictness", strict);
  // (void)getInput("activate_asap", asap);
  // (void)getInput("timeout_s", tout);

  if (!act.empty()) {
    const auto s = getControllerState(cm_ns, act);
    if (s == "active") {
      RCLCPP_INFO(node_->get_logger(), "[SwitchControllers] '%s' already active -> no-op for activate", act.c_str());
      act.clear();
    }
  }
  if (!deact.empty()) {
    const auto s = getControllerState(cm_ns, deact);
    if (!s.empty() && s != "active") {
      RCLCPP_INFO(node_->get_logger(), "[SwitchControllers] '%s' already not active -> no-op for deactivate", deact.c_str());
      deact.clear();
    }
  }

  if (act.empty() && deact.empty()) {
    RCLCPP_WARN(node_->get_logger(), "SwitchControllers: nothing to do");
    return BT::NodeStatus::SUCCESS;
  }

  bool ok = switchControllers(cm_ns, act, deact, strict, asap, tout);
  if (!ok) return BT::NodeStatus::FAILURE;

   const bool act_ok   = act.empty()   || (getControllerState(cm_ns, act)   == "active");
  const bool deact_ok = deact.empty() || (getControllerState(cm_ns, deact) != "active");

  if (act_ok && deact_ok) {
    RCLCPP_INFO(node_->get_logger(), "SwitchControllers: end state verified -> SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(node_->get_logger(), "SwitchControllers: end state NOT as requested -> FAILURE");
  return BT::NodeStatus::FAILURE;

}

} // namespace bt_nodes