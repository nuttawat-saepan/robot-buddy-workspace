import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    go2_share = get_package_share_directory('go2_control')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
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
        default=os.path.join(go2_share, 'nav2_costmap_only_params.yaml'),
    )

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file,
        },
        convert_types=True,
    )

    urdf_file = os.path.join(go2_share, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_file),
        DeclareLaunchArgument('params_file', default_value=params_file),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),

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

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
        ),

        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            namespace='costmap',
            output='screen',
            parameters=[configured_params],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_scan_test',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'costmap/costmap',
                ],
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_costmap_scan_test',
            condition=IfCondition(enable_rviz),
            output='screen',
        ),
    ])
