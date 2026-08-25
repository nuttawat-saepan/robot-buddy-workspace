import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_share = get_package_share_directory('go2_control')
    pc2scan_param = os.path.join(go2_share, 'pc2scan_livox.yaml')

    lidar_topic = LaunchConfiguration('lidar_topic', default='/livox/lidar')
    imu_topic = LaunchConfiguration('imu_topic', default='/livox/imu')
    lidar_frame = LaunchConfiguration('lidar_frame', default='livox_frame')
    base_frame = LaunchConfiguration('base_frame', default='base_link')
    publish_rate = LaunchConfiguration('publish_rate', default='10.0')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('lidar_frame', default_value='livox_frame'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),

        Node(
            package='go2_control',
            executable='fake_livox_mid360',
            name='fake_livox_mid360',
            output='screen',
            parameters=[{
                'topic': lidar_topic,
                'imu_topic': imu_topic,
                'frame_id': lidar_frame,
                'publish_rate': publish_rate,
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='livox_lidar_tf',
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
            name='pc2scan_livox',
            output='screen',
            remappings=[
                ('cloud_in', lidar_topic),
                ('scan', '/scan'),
            ],
            parameters=[pc2scan_param],
        ),
    ])
