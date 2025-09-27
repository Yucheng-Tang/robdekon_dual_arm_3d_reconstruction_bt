#include "nodes/add_obstacle_action.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace bt_nodes
{

AddObstacle::AddObstacle(const std::string& name,
                         const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  planning_scene_client_ =
      node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");

  // Optional: wait briefly for the service (avoid blocking forever)
  if (!planning_scene_client_->wait_for_service(2s))
  {
    RCLCPP_WARN(node_->get_logger(),
                "[%s] Service 'apply_planning_scene' not available yet; will still try on tick()",
                name.c_str());
  }
}

BT::PortsList AddObstacle::providedPorts()
{
  return {
      BT::InputPort<std::string>("obstacle_name"),
      BT::InputPort<std::string>("parent_frame"),
      BT::InputPort<std::string>("obstacle_shape"),  // "box" | "cylinder" | "sphere"
      BT::InputPort<std::string>("obstacle_pose"),   // your utils parse to geometry_msgs::msg::Pose
      BT::InputPort<std::string>("obstacle_size")    // parsed to vector<double>
  };
}

BT::NodeStatus AddObstacle::tick()
{
  // --- Read ports
  auto name       = getInput<std::string>("obstacle_name");
  auto parent     = getInput<std::string>("parent_frame");
  auto shape_str  = getInput<std::string>("obstacle_shape");
  auto pose_str   = getInput<std::string>("obstacle_pose");
  auto size_str   = getInput<std::string>("obstacle_size");

  if (!name || !parent || !shape_str || !pose_str || !size_str)
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Missing required input port(s).", this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Parse pose & size using your existing helpers (ROS 2 types)
  geometry_msgs::msg::Pose pose;
  std::vector<double> size;

  try
  {
    pose = analyseTargetPose(*pose_str);     // must return geometry_msgs::msg::Pose in ROS 2
    size = analyseJointValue(*size_str);     // vector<double>
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Parsing error: %s", this->name().c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  // Build primitive according to shape
  shape_msgs::msg::SolidPrimitive primitive;
  const auto& s = *shape_str;

  if (s == "box")
  {
    // size = [x, y, z]
    if (size.size() < 3)
    {
      RCLCPP_ERROR(node_->get_logger(), "[%s] BOX needs size=[x y z].", this->name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = { size[0], size[1], size[2] };
  }
  else if (s == "cylinder")
  {
    // size = [height, radius]
    if (size.size() < 2)
    {
      RCLCPP_ERROR(node_->get_logger(), "[%s] CYLINDER needs size=[height radius].", this->name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = size[0];
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = size[1];
  }
  else if (s == "sphere")
  {
    // size = [radius]
    if (size.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "[%s] SPHERE needs size=[radius].", this->name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    primitive.dimensions.resize(1);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = size[0];
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Unknown shape '%s'. Use box|cylinder|sphere.",
                 this->name().c_str(), s.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Build CollisionObject
  moveit_msgs::msg::CollisionObject obj;
  obj.id = *name;
  obj.header.frame_id = *parent;
  obj.primitives.push_back(primitive);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Build PlanningScene diff
  moveit_msgs::msg::PlanningScene scene_msg;
  scene_msg.world.collision_objects.push_back(obj);
  scene_msg.is_diff = true;

  auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  req->scene = scene_msg;

  // Ensure client exists
  if (!planning_scene_client_)
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] planning_scene_client_ is null.", this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!planning_scene_client_->wait_for_service(2s))
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Service 'apply_planning_scene' not available.", this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Call service (sync wait)
  auto future = planning_scene_client_->async_send_request(req);
  auto rc = rclcpp::spin_until_future_complete(node_, future, 5s);

  if (rc != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] ApplyPlanningScene call failed (timeout or error).",
                 this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto resp = future.get();
  if (!resp || !resp->success)
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] ApplyPlanningScene returned success=false.", this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Logs (optional)
  RCLCPP_INFO(node_->get_logger(), "[%s] Added obstacle '%s' in frame '%s' (shape=%s).",
              this->name().c_str(), name->c_str(), parent->c_str(), shape_str->c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace Turntable_NBV

// Optional: if you build this as a BT plugin, register it:
// #include <behaviortree_cpp_v3/bt_factory.h>
// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<Turntable_NBV::AddObstacle>(\"AddObstacle\");
// }