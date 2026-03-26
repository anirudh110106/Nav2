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
        # 1. Base to Camera Transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
        ),

        # 2. Depth to Laser Scan
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
            ],
            parameters=[{'scan_time': 0.033, 'range_min': 0.2, 'range_max': 5.0, 'output_frame': 'camera_depth_frame'}]
        ),

        # 3. Nav2 Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'false',
                'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
            }.items()
        ),

        # 4. THE HARDCODED STARTING POSITION
        # This publishes the (0,0) coordinate 15 times (once per second). 
        # It guarantees Nav2 catches it during bootup!
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-r', '1', '--times', '15',
                '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
                '"{header: {frame_id: \'map\'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"'
            ],
            name='auto_pose_injector',
            output='screen'
        )
    ])
