import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_share = get_package_share_directory('go2_control')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world = os.path.join(go2_share, 'worlds', 'sim_safe_empty.world')
    model_path = os.path.join(go2_share, 'models')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_drive = LaunchConfiguration('enable_drive', default='false')
    linear_x = LaunchConfiguration('linear_x', default='0.08')
    angular_z = LaunchConfiguration('angular_z', default='0.0')
    duration = LaunchConfiguration('duration', default='3.0')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_drive', default_value='false'),
        DeclareLaunchArgument('linear_x', default_value='0.08'),
        DeclareLaunchArgument('angular_z', default_value='0.0'),
        DeclareLaunchArgument('duration', default_value='3.0'),

        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            [model_path, ':', os.environ.get('GAZEBO_MODEL_PATH', '')],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
            ),
        ),

        Node(
            package='go2_control',
            executable='sim_safe_drive',
            name='sim_safe_drive',
            condition=IfCondition(enable_drive),
            parameters=[{
                'topic': '/sim_cmd_vel',
                'linear_x': linear_x,
                'angular_z': angular_z,
                'duration': duration,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
    ])
