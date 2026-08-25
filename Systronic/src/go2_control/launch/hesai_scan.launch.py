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
    x = LaunchConfiguration('x', default='0.25')
    y = LaunchConfiguration('y', default='0.0')
    z = LaunchConfiguration('z', default='0.12')
    yaw = LaunchConfiguration('yaw', default='0.0')
    pitch = LaunchConfiguration('pitch', default='0.0')
    roll = LaunchConfiguration('roll', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/lidar_points'),
        DeclareLaunchArgument('lidar_frame', default_value='hesai_lidar'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('x', default_value='0.25'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.12'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('roll', default_value='0.0'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='hesai_lidar_tf',
            arguments=[
                x, y, z,
                yaw, pitch, roll,
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
