import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    map_file = '/home/rpd/Nav2/my_robot/my_robot/my_map.yaml'

    return LaunchDescription([

        # 0. THE SPINE: Odometry + Full TF Tree Publisher
        # Publishes odom -> base_footprint -> base_link -> camera_link -> camera_depth_frame
        # AMCL needs this TF chain to exist before it can publish the map frame
        Node(
            package='my_robot',
            executable='odom',
            name='motor_odom',
            output='screen'
        ),

        # 1. THE EYES: Depth to Laser Scan
        # Takes 3D camera data and flattens it into a 2D laser scan for AMCL
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/scan')
            ],
            parameters=[{
                'scan_time': 0.033,
                'range_min': 0.45,
                'range_max': 5.0,
                'output_frame': 'camera_depth_frame'
            }]
        ),

        # 2. THE BRAIN: Nav2 Bringup
        # Launches AMCL, the Planner, and the Controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'false',
                'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
            }.items()
        ),

        # 3. THE AUTO-START: Initial Pose Injector
        # Blasts the (0,0) coordinate 20 times to ensure AMCL catches it on boot
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-r', '1', '--times', '20',
                '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
                '"{header: {frame_id: \'map\'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"'
            ],
            name='auto_pose_injector',
            output='screen'
        )
    ])
