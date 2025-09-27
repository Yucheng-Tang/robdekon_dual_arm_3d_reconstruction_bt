#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <memory>
#include <string>
#include <vector>

namespace bt_nodes {

class SwitchControllersAction : public BT::SyncActionNode
{
public:
  SwitchControllersAction(const std::string& name, const BT::NodeConfiguration& cfg);

  // Ports:
  // - Optional input ports to override params at tick time
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("activate"),   // controllers to activate
      BT::InputPort<std::string>("deactivate"), // controllers to stop
      BT::InputPort<std::string>("cm_ns"),                   // controller manager namespace (default "")
      BT::InputPort<int>("strictness"),                      // 1: BEST_EFFORT, 2: STRICT
      BT::InputPort<bool>("activate_asap"),
      BT::InputPort<double>("timeout_s")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  // defaults from YAML (loaded once)
  std::string cm_ns_;
  std::string activate_;
  std::string deactivate_;
  int strictness_{2};
  bool activate_asap_{true};
  double timeout_s_{5.0};

  bool switchControllers(const std::string& cm_ns,
                         const std::string& act,
                         const std::string& deact,
                         int strictness,
                         bool activate_asap,
                         double timeout_s);

  std::string getControllerState(const std::string& cm_ns, 
                                 const std::string& name);

};

} // namespace bt_nodes