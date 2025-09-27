from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart


import os, yaml

def generate_launch_description():
    # --- Launch arguments ---
    arm_id     = DeclareLaunchArgument('arm_id', default_value='')
    robot_ip   = DeclareLaunchArgument('robot_ip', default_value='172.168.8.20')
    use_fake = DeclareLaunchArgument('use_fake_hardware', default_value='false')

    # RViz toggles (avoid launching two RViz instances)
    bringup_rviz = DeclareLaunchArgument('bringup_use_rviz', default_value='false')
    moveit_rviz  = DeclareLaunchArgument('moveit_use_rviz',  default_value='true')

   # Gripper + cuMotion toggles
    use_gripper  = DeclareLaunchArgument('use_gripper', default_value='true')
    gripper_namespace = DeclareLaunchArgument('gripper_namespace', default_value='fr3_gripper')
    use_cumotion = DeclareLaunchArgument('use_cumotion', default_value='false')
    cumo_xrdf    = DeclareLaunchArgument('cumotion_xrdf', default_value='')
    cumo_urdf    = DeclareLaunchArgument('cumotion_urdf', default_value='')

    # RealSense toggles
    use_realsense   = DeclareLaunchArgument('use_realsense', default_value='true')
    camera_name     = DeclareLaunchArgument('camera_name', default_value='camera')
    camera_serial   = DeclareLaunchArgument('camera_serial', default_value='')    # set to lock a specific device
    align_depth     = DeclareLaunchArgument('align_depth', default_value='true')  # align depth->color
    enable_pointcloud = DeclareLaunchArgument('enable_pointcloud', default_value='true')

    # BT config
    bt_xml = DeclareLaunchArgument(
        'bt_xml',
        default_value=os.path.join(
            get_package_share_directory('one_arm_nbv_bt'), 'bt_trees', 'nbv_pick_place.xml'
        )
    )
    cfg_yaml = DeclareLaunchArgument(
        'config_yaml',
        default_value=os.path.join(
            get_package_share_directory('one_arm_nbv_bt'), 'config', 'planning.yaml'
        )
    )

    # Spawn gravity controller for testing
    spawn_gravity = DeclareLaunchArgument('spawn_gravity', default_value='true')
    gravity_name  = DeclareLaunchArgument('gravity_controller_name', default_value='gravity_compensation_example_controller')
    gravity_type  = DeclareLaunchArgument('gravity_controller_type', default_value='franka_example_controllers/GravityCompensationExampleController')

    calib_yaml = DeclareLaunchArgument(
        'calib_file',
        default_value=os.path.join(
            get_package_share_directory('one_arm_nbv_bt'),
            'config', 'franka_left_camera_link.calib'
        )
    )

    return LaunchDescription([
        arm_id, robot_ip, use_fake,
        bringup_rviz, moveit_rviz,
        use_gripper, use_cumotion, cumo_xrdf, cumo_urdf,
        use_realsense, camera_name, camera_serial, align_depth, enable_pointcloud,
        bt_xml, cfg_yaml,
        spawn_gravity, gravity_name, gravity_type,
        calib_yaml, 
        OpaqueFunction(function=launch_all)
    ])

def launch_all(context, *args, **kwargs):
    lc = lambda name: LaunchConfiguration(name).perform(context)
    nodes = []

    # # virtual transformation should start at first
    # # --- 1.1) Realsense Camera launch ---
    # realsense_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    #     ),
    #     launch_arguments={
    #         'camera_name': LaunchConfiguration('camera_name'),
    #         'serial_no':   LaunchConfiguration('camera_serial'),
    #         'align_depth.enable': LaunchConfiguration('align_depth'),
    #         'pointcloud.enable':  LaunchConfiguration('enable_pointcloud'),
    #         # common stream profiles (tweak if you like)
    #         'rgb_camera.profile':   '1280x720x30',
    #         'depth_module.profile': '640x480x30',
    #         # helpful to timestamp-sync IMU/frames
    #         'enable_sync': 'true',
    #         # publish TFs for camera frames
    #         'publish_tf': 'false',
    #     }.items(),
    #     condition=IfCondition(LaunchConfiguration('use_realsense'))
    # )
    # nodes.append(realsense_launch)

    # # --- 1.2) Realsense Camera tf pub ---
    # calib_path = lc('calib_file')
    # try:
    #     with open(calib_path, 'r') as f:
    #         calib = yaml.safe_load(f)
    # except Exception as e:
    #     print(f"[bringup_fr3_nbv_bt] ERROR reading calib file '{calib_path}': {e}")
    #     calib = None

    # if calib is not None:
    #     try:
    #         T = calib['transform']
    #         tx = T['translation']['x']
    #         ty = T['translation']['y']
    #         tz = T['translation']['z']
    #         qx = T['rotation']['x']
    #         qy = T['rotation']['y']
    #         qz = T['rotation']['z']
    #         qw = T['rotation']['w']

    #         ee_frame  = calib['parameters']['robot_effector_frame']     
    #         cam_frame = calib['parameters']['tracking_base_frame']      

    #         nodes.append(
    #             Node(
    #                 package='tf2_ros',
    #                 executable='static_transform_publisher',
    #                 name='handeye_static_tf',
    #                 output='screen',
    #                 arguments=[
    #                     '--x',  str(tx), '--y',  str(ty), '--z',  str(tz),
    #                     '--qx', str(qx), '--qy', str(qy), '--qz', str(qz), '--qw', str(qw),
    #                     '--frame-id',       ee_frame,
    #                     '--child-frame-id', cam_frame,
    #                 ],
    #             )
    #         )
    #     except KeyError as e:
    #         print(f"[bringup_fr3_nbv_bt] ERROR: missing key in calib file: {e}")

    # # --- 2) FR3 hardware bringup (franka_bringup) ---
    # franka_bringup_dir = get_package_share_directory('franka_bringup')
    # nodes.append(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(franka_bringup_dir, 'launch', 'franka.launch.py')
    #         ),
    #         launch_arguments={
    #             'arm_id': lc('arm_id'),
    #             'robot_ip': lc('robot_ip'),
    #             'use_rviz': lc('bringup_use_rviz'),
    #             'use_fake_hardware': lc('use_fake_hardware'),
    #             'load_gripper': lc('use_gripper'),
    #         }.items()
    #     )
    # )


    # --- 3) MoveIt for FR3 (prefer moveit.launch.py; fallback to move_group.launch.py) ---
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
                    'robot_ip': lc('robot_ip'),              # ignored if fake hardware
                    'use_fake_hardware': lc('use_fake_hardware'),
                    # Different configs use either 'use_rviz' or 'launch_rviz'; pass both.
                    'use_rviz': lc('moveit_use_rviz'),
                    'launch_rviz': lc('moveit_use_rviz'),
                }.items()
            )
        )
    elif os.path.exists(move_group_launch):
        print("[bringup_fr3_nbv_bt] USING move_group.launch.py")
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(move_group_launch),
                launch_arguments={
                    'use_rviz': lc('moveit_use_rviz'),
                }.items()
            )
        )
    else:
        # If neither exists, warn and continue (your BT may still run if robot_description is available)
        print("[bringup_fr3_nbv_bt] WARNING: No MoveIt launch found in franka_fr3_moveit_config")

    


    # # --- 4) Optional: cuMotion planner node ---
    # if lc('use_cumotion').lower() in ('true', '1', 'yes'):
    #     nodes.append(
    #         Node(
    #             package='isaac_ros_cumotion',
    #             executable='cumotion_planner_node',
    #             name='cumotion_planner_node',
    #             output='screen',
    #             parameters=[
    #                 {'robot': lc('cumotion_xrdf')},     # path to .xrdf
    #                 {'urdf_path': lc('cumotion_urdf')}, # path to .urdf
    #             ]
    #         )
    #     )

    # spawner_all = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "joint_state_broadcaster",
    #         "franka_robot_state_broadcaster",
    #         "fr3_arm_controller",
    #         "--controller-manager", "/controller_manager",
    #     ],
    #     output="screen",
    # )

    # spawn_after_control = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=ros2_control_node,          # your existing control node
    #         on_start=[TimerAction(period=1.5, actions=[spawner_all])]
    #     )
    # )
    # nodes.append(spawn_after_control)
    

    # # --- 5) Spawn Gravity Controller for testing ---
    # if lc('spawn_gravity').lower() in ('true', '1', 'yes'):
    #     nodes.append(
    #         Node(
    #             package='controller_manager',
    #             executable='spawner',
    #             name='spawner_gravity_compensation_example_controller',
    #             output='screen',
    #             arguments=[
    #                 lc('gravity_controller_name'),
    #                 '--controller-type', lc('gravity_controller_type'),
    #                 '--controller-manager', '/controller_manager',
    #                 '--inactive',
    #             ],
    #         )
    #     )

    # # --- 6) Your Behavior Tree node ---
    # nodes.append(
    #     Node(
    #         package='one_arm_nbv_bt',
    #         executable='nbv_bt_node',
    #         name='one_arm_nbv_bt_node',
    #         output='screen',
    #         parameters=[
    #             {'bt_xml': lc('bt_xml')},
    #             lc('config_yaml'),  # loads planning.yaml
    #         ]
    #     )
    # )

    return nodes