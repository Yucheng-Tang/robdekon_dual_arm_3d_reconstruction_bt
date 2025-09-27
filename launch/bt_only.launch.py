from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_bt = get_package_share_directory('one_arm_nbv_bt')

    # Xacro paths
    franka_desc = get_package_share_directory('franka_description')
    urdf_xacro  = os.path.join(franka_desc, 'robots', 'fr3', 'fr3.urdf.xacro')
    srdf_xacro  = os.path.join(franka_desc, 'robots', 'fr3', 'fr3.srdf.xacro')

    # Example args (ensure they are strings "true"/"false")
    robot_ip = "0.0.0.0"          # or LaunchConfiguration(...)
    use_fake_hardware   = "true"
    fake_sensor_commands = "false"

    # URDF
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )

    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=true',
         ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands, ' ros2_control:=true'])

    robot_description = {'robot_description': ParameterValue(
        robot_description_config, value_type=str)}

    
    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.srdf.xacro'
    )

    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ',
         franka_semantic_xacro_file, ' hand:=true']
    )

    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}

    # Your BT params YAML
    planning_yaml = os.path.join(pkg_bt, 'config', 'planning.yaml')

    return LaunchDescription([
        Node(
            package='one_arm_nbv_bt',
            executable='nbv_bt_node',
            name='one_arm_nbv_bt_node',
            output='screen',
            # parameters=[os.path.join(pkg_bt, 'config', 'planning.yaml')],
            parameters=[
                robot_description,
                robot_description_semantic,
                planning_yaml,
            ]
        )
    ]
)