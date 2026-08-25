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
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_description_share = get_package_share_directory('turtlebot3_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    slam_params = LaunchConfiguration('slam_params')

    world_default = os.path.join(go2_share, 'worlds', 'turtlebot3_house_empty.world')
    model_default = os.path.join(
        go2_share,
        'models',
        'turtlebot3_waffle_3d_lidar',
        'model.sdf',
    )
    slam_params_default = os.path.join(go2_share, 'config', 'slam_fake_hesai.yaml')
    pc2scan_param = os.path.join(go2_share, 'pc2scan_hesai.yaml')

    urdf_path = os.path.join(
        turtlebot3_description_share,
        'urdf',
        'turtlebot3_waffle.urdf',
    )
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    model_path = os.pathsep.join([
        os.path.join(go2_share, 'models'),
        os.path.join(turtlebot3_gazebo_share, 'models'),
        os.environ.get('GAZEBO_MODEL_PATH', ''),
    ])

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=world_default),
        DeclareLaunchArgument('model', default_value=model_default),
        DeclareLaunchArgument('slam_params', default_value=slam_params_default),

        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),

        gzserver,
        gzclient,

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
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_turtlebot3_waffle_3d_lidar',
            output='screen',
            arguments=[
                '-entity', 'turtlebot3_waffle_3d_lidar',
                '-file', model,
                '-x', '-2.0',
                '-y', '-0.5',
                '-z', '0.01',
                '-Y', '0.0',
            ],
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc2scan_hesai',
            output='screen',
            remappings=[
                ('cloud_in', '/lidar_points'),
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
            name='rviz2_hesai_3d_mapping',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
