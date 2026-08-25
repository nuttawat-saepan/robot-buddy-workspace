import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_share = get_package_share_directory('go2_control')
    pc2scan_param = os.path.join(go2_share, 'pc2scan_hesai.yaml')

    lidar_topic = LaunchConfiguration('lidar_topic', default='/lidar_points')
    lidar_frame = LaunchConfiguration('lidar_frame', default='hesai_lidar')
    base_frame = LaunchConfiguration('base_frame', default='base_link')
    publish_rate = LaunchConfiguration('publish_rate', default='10.0')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/lidar_points'),
        DeclareLaunchArgument('lidar_frame', default_value='hesai_lidar'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),

        Node(
            package='go2_control',
            executable='fake_hesai_xt16',
            name='fake_hesai_xt16',
            output='screen',
            parameters=[{
                'topic': lidar_topic,
                'frame_id': lidar_frame,
                'publish_rate': publish_rate,
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='hesai_lidar_tf',
            arguments=[
                '0.25', '0.0', '0.12',
                '0.0', '0.0', '0.0',
                base_frame, lidar_frame,
            ],
            output='screen',
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc2scan_hesai',
            output='screen',
            remappings=[
                ('cloud_in', lidar_topic),
                ('scan', '/scan'),
            ],
            parameters=[pc2scan_param],
        ),
    ])
