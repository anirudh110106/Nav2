from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '0.0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
        ),

        # RGBD SYNC NODE (🔥 THIS IS THE KEY FIX)
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            parameters=[{
                'approx_sync': True,
                'queue_size': 20
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('depth/image', '/camera/camera/depth/image_rect_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('rgbd_image', '/rgbd_image')
            ]
        ),

        # RTABMAP (now uses synced input)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_rgbd': True,   # 🔥 IMPORTANT
                'approx_sync': True,
                'queue_size': 10
            }],
            remappings=[
                ('rgbd_image', '/rgbd_image'),
                ('odom', '/odom')
            ]
        ),

        # RVIZ
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
