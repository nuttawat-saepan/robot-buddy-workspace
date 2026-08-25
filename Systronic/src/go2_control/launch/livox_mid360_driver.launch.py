"""Livox Mid-360 driver bringup, sensor readout only.

Differs from the upstream msg_MID360_launch.py in one important way:
xfer_format is 0 (PointCloud2) instead of 1 (CustomMsg), because
pointcloud_to_laserscan and the Nav2 stack cannot consume CustomMsg.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_share = get_package_share_directory('go2_control')
    default_config = os.path.join(
        go2_share, 'config', 'livox_mid360_field.example.json')

    user_config_path = LaunchConfiguration('user_config_path')
    frame_id = LaunchConfiguration('frame_id')
    publish_freq = LaunchConfiguration('publish_freq')
    xfer_format = LaunchConfiguration('xfer_format')

    return LaunchDescription([
        DeclareLaunchArgument(
            'user_config_path', default_value=default_config,
            description='Copy the example to livox_mid360_field.json and pass it here.'),
        DeclareLaunchArgument('frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('publish_freq', default_value='10.0'),
        DeclareLaunchArgument(
            'xfer_format', default_value='0',
            description='0 = PointCloud2, 1 = Livox CustomMsg.'),

        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[
                {'xfer_format': xfer_format},
                {'multi_topic': 0},
                {'data_src': 0},
                {'publish_freq': publish_freq},
                {'output_data_type': 0},
                {'frame_id': frame_id},
                {'user_config_path': user_config_path},
                {'cmdline_input_bd_code': 'livox0000000001'},
            ],
        ),
    ])
