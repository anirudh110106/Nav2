import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # ================================
        # 1. TF: base_link → camera_link
        # ================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0.1', '0.0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
        ),

        # ==========================================
        # 2. TF: base_footprint → base_link (IMPORTANT)
        # ==========================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='footprint_to_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),

        # ==========================================
        # 3. Depth → LaserScan
        # ==========================================
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan',
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/scan')
            ],
            parameters=[{
                'scan_time': 0.033,
                'range_min': 0.45,
                'range_max': 5.0,
                'scan_height': 5,
                'output_frame': 'camera_link',
                'use_sim_time': False
            }]
        ),

        # ==========================================
        # 4. RVIZ (optional but useful)
        # ==========================================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
