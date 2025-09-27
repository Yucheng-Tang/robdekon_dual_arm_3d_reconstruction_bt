from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


import os

def generate_launch_description():
    nodes = []
    fr3_moveit_dir = get_package_share_directory('franka_fr3_moveit_config')
    moveit_launch = os.path.join(fr3_moveit_dir, 'launch', 'moveit.launch.py')
    move_group_launch = os.path.join(fr3_moveit_dir, 'launch', 'move_group.launch.py')

    if os.path.exists(moveit_launch):
        print("[bringup_fr3_nbv_bt] USING moveit.launch.py")
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                # These args are accepted by the FR3 MoveIt launch:
                launch_arguments={
                    'robot_ip': "172.168.8.10",              # ignored if fake hardware
                    'use_fake_hardware': "false",
                    # Different configs use either 'use_rviz' or 'launch_rviz'; pass both.
                    'use_rviz': "true",
                    'launch_rviz': "true",
                }.items()
            )
        )
    elif os.path.exists(move_group_launch):
        print("[bringup_fr3_nbv_bt] USING move_group.launch.py")
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(move_group_launch),
                launch_arguments={
                    'use_rviz': "true",
                }.items()
            )
        )
    else:
        # If neither exists, warn and continue (your BT may still run if robot_description is available)
        print("[bringup_fr3_nbv_bt] WARNING: No MoveIt launch found in franka_fr3_moveit_config")

    pkg_bt = get_package_share_directory('one_arm_nbv_bt')
    nodes.append(
        Node(
            package='one_arm_nbv_bt',
            executable='nbv_bt_node',
            name='one_arm_nbv_bt_node',
            output='screen',
            parameters=[os.path.join(pkg_bt, 'config', 'planning.yaml')],
        )
    )
    return LaunchDescription(nodes)