#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

#include "nodes/camera_sample_action.hpp"
#include "nodes/fuse_pointcloud_action.hpp"
#include "nodes/compute_nbv_action.hpp"
#include "nodes/plan_arm_move_action.hpp"
#include "nodes/pick_action.hpp"
#include "nodes/compute_reorient_action.hpp"
#include "nodes/place_action.hpp"
#include "nodes/check_coverage_condition.hpp"
#include "nodes/emit_pose_param_action.hpp"
#include "nodes/move_to_joint_action.hpp"
#include "nodes/switch_contorllers_action.hpp"
#include "nodes/add_obstacle_action.hpp"
// #include "nodes/user_gate_wait.hpp"
#include "nodes/change_object_center_action.hpp"
#include "nodes/is_true_condition.hpp"
#include "nodes/evaluate_reconstruction_action.hpp"



int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  //   auto node = rclcpp::Node::make_shared("one_arm_nbv_bt_node");
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("one_arm_nbv_bt_node", opts);

  std::string pkg = ament_index_cpp::get_package_share_directory("one_arm_nbv_bt");
  std::string bt_xml = pkg + "/bt_trees/pick_place_manual_sampling.xml";  // pick_place_manual_sampling pick_place_only

  BT::BehaviorTreeFactory f;
  f.registerNodeType<bt_nodes::CameraSampleAction>("CameraSample");
  f.registerNodeType<bt_nodes::FusePointCloudAction>("FusePointCloud");
//   f.registerNodeType<bt_nodes::ComputeNBVAction>("ComputeNBV");
  f.registerNodeType<bt_nodes::AddObstacle>("AddObstacle");
  f.registerNodeType<bt_nodes::PlanArmMoveAction>("PlanArmMove");
  f.registerNodeType<bt_nodes::PickAction>("Pick");
//   f.registerNodeType<bt_nodes::ComputeReorientAction>("ComputeReorient");
  f.registerNodeType<bt_nodes::PlaceAction>("Place");
  f.registerNodeType<bt_nodes::CheckCoverageCondition>("CheckCoverage");
  // f.registerNodeType<bt_nodes::EmitPoseParamAction>("EmitPoseParam");
  f.registerNodeType<bt_nodes::MoveToJointsAction>("MoveToJoints");
  f.registerNodeType<bt_nodes::SwitchControllersAction>("SwitchControllers");
  // f.registerNodeType<bt_nodes::UserGateWait>("UserGateWait");
  f.registerNodeType<bt_nodes::ChangeObjectCenterAction>("ChangeObjectCenter");
  f.registerNodeType<bt_nodes::IsTrueCondition>("IsTrue");
  f.registerNodeType<bt_nodes::EvaluateReconstructionAction>("EvaluateReconstruction");



  std::ifstream xml(bt_xml);
  if (!xml){ RCLCPP_FATAL(node->get_logger(), "BT XML missing: %s", bt_xml.c_str()); return 1; }
  std::string xml_text((std::istreambuf_iterator<char>(xml)), std::istreambuf_iterator<char>());

  auto bb = BT::Blackboard::create();
  bb->set("node", node);

  auto tree = f.createTreeFromText(xml_text, bb);
  rclcpp::Rate r(20.0);
  // while (rclcpp::ok()){
  //   auto s = tree.tickRoot();
  //   if (s == BT::NodeStatus::SUCCESS) break;
  //   rclcpp::spin_some(node);
  //   r.sleep();
  // }
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
  {
    status = tree.tickRoot();
    rclcpp::spin_some(node);
    r.sleep();
  }

  // Halt the tree to clean up any running nodes.
  tree.haltTree();

  int ret = 0;
  if (status == BT::NodeStatus::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Behavior Tree finished: SUCCESS");
    ret = 0;
  }
  else if (status == BT::NodeStatus::FAILURE)
  {
    RCLCPP_WARN(node->get_logger(), "Behavior Tree finished: FAILURE");
    ret = 2;  // non-zero to signal failure
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "Behavior Tree aborted (status: %d)", static_cast<int>(status));
    ret = 3;
}


  rclcpp::shutdown();
  return 0;
}