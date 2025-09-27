from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    pkg = get_package_share_directory('one_arm_nbv_bt')
    return LaunchDescription([
        Node(package='one_arm_nbv_bt', executable='nbv_bt_node', name='nbv_bt_node',
             parameters=[os.path.join(pkg,'config','planning.yaml')])
    ])