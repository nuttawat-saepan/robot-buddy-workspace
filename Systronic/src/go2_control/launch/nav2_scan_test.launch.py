import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_share = get_package_share_directory('go2_control')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml',
        ),
    )
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(go2_share, 'nav2_params_go2w.yaml'),
    )

    urdf_file = os.path.join(go2_share, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_file),
        DeclareLaunchArgument('params_file', default_value=params_file),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fake_map_to_odom_tf',
            arguments=[
                '0.0', '0.0', '0.0',
                '0.0', '0.0', '0.0',
                'map', 'odom',
            ],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fake_odom_to_base_tf',
            arguments=[
                '0.0', '0.0', '0.0',
                '0.0', '0.0', '0.0',
                'odom', 'base_link',
            ],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true',
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav2_scan_test',
            output='screen',
        ),
    ])
