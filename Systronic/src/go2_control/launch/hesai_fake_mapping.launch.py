import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_share = get_package_share_directory('go2_control')
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle')

    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_topic = LaunchConfiguration('lidar_topic')
    slam_params = LaunchConfiguration('slam_params')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_house.launch.py',
            )
        ),
    )

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_' + turtlebot3_model + '.urdf',
    )
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    pc2scan_param = os.path.join(go2_share, 'pc2scan_fake_mapping.yaml')
    slam_params_default = os.path.join(go2_share, 'config', 'slam_fake_hesai.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('lidar_topic', default_value='/lidar_points'),
        DeclareLaunchArgument('slam_params', default_value=slam_params_default),

        SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model),

        gazebo_launch,

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
            package='go2_control',
            executable='scan_to_pointcloud',
            name='gazebo_scan_to_fake_hesai_cloud',
            output='screen',
            parameters=[{
                'input_topic': '/scan',
                'output_topic': lidar_topic,
                'z': 0.05,
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fake_hesai_lidar_tf',
            arguments=[
                '0.0', '0.0', '0.0',
                '0.0', '0.0', '0.0',
                'base_scan', 'hesai_lidar',
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
                ('scan', '/hesai_scan'),
            ],
            parameters=[pc2scan_param, {
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_fake_hesai_mapping',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
