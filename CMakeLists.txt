cmake_minimum_required(VERSION 3.5)
project(dual_arm_nbv_bt)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)  # MoveItCpp, PlanningSceneInterface
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(utils_msgs REQUIRED)


find_package(PCL REQUIRED COMPONENTS io filters)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(controller_manager_msgs REQUIRED)


# Point cloud & conversions
include_directories(include)
install(DIRECTORY include/ DESTINATION include)

# For mesh masking
find_package(geometric_shapes REQUIRED)

include_directories(src)
install(DIRECTORY include/ DESTINATION include)

add_library(bt_nodes
  src/nodes/camera_sample_action.cpp
  src/nodes/fuse_pointcloud_action.cpp
  src/nodes/compute_nbv_action.cpp
  src/nodes/plan_arm_move_action.cpp
  src/nodes/pick_action.cpp
  src/nodes/compute_reorient_action.cpp
  src/nodes/place_action.cpp
  src/nodes/check_coverage_condition.cpp
  src/nodes/emit_pose_param_action.cpp
  src/nodes/move_to_joint_action.cpp
  src/nodes/switch_controllers_action.cpp
  src/nodes/add_obstacle_action.cpp
  src/nodes/user_gate_wait.cpp
  src/nodes/change_object_center_action.cpp
  src/nodes/is_true_condition.cpp
  src/nodes/evaluate_reconstruction_action.cpp
  src/utils/constraints.hpp
  src/utils/bt_conversion.hpp
)

# Add PCL includes & link, and avoid PCL precompiled point types issues
target_include_directories(bt_nodes SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(bt_nodes ${PCL_LIBRARIES})
target_compile_definitions(bt_nodes PRIVATE PCL_NO_PRECOMPILE)

ament_target_dependencies(bt_nodes
  rclcpp behaviortree_cpp_v3
  moveit_ros_planning_interface moveit_msgs
  geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs
  franka_msgs rclcpp_action
  tf2_eigen pcl_conversions geometric_shapes
  controller_manager_msgs utils_msgs tf2
) 

add_executable(nbv_bt_node src/main_bt_node.cpp)
target_link_libraries(nbv_bt_node bt_nodes)
ament_target_dependencies(nbv_bt_node
  rclcpp behaviortree_cpp_v3
  moveit_ros_planning_interface moveit_msgs
  geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2_eigen
  controller_manager_msgs utils_msgs tf2
)

install(TARGETS bt_nodes nbv_bt_node 
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY bt_trees/ DESTINATION share/${PROJECT_NAME}/bt_trees)
install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/   DESTINATION share/${PROJECT_NAME}/config)

ament_package()