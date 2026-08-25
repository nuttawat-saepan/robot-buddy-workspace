import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_share = get_package_share_directory('go2_control')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz',
        'tb3_navigation2.rviz',
    )

    map_default = os.path.join(
        os.path.expanduser('~'),
        'UnitreeRos',
        'unitree_ros2',
        'map',
        'house_map.yaml',
    )
    params_default = os.path.join(go2_share, 'nav2_params_go2w.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_dir = LaunchConfiguration('map')
    param_dir = LaunchConfiguration('params_file')
    enable_cmd_vel = LaunchConfiguration('enable_cmd_vel')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_april = LaunchConfiguration('enable_april')
    enable_mqtt = LaunchConfiguration('enable_mqtt')
    enable_joint_state_gui = LaunchConfiguration('enable_joint_state_gui')
    enable_unitree_read = LaunchConfiguration('enable_unitree_read')
    enable_rviz = LaunchConfiguration('enable_rviz')
    mqtt_broker = LaunchConfiguration('mqtt_broker')
    mqtt_port = LaunchConfiguration('mqtt_port')
    unitree_interface = LaunchConfiguration('unitree_interface')
    robot_ack = LaunchConfiguration('robot_ack')
    pc2scan_param = os.path.join(go2_share, 'pc2scan.yaml')
    april_localizer_param = os.path.join(go2_share, 'config', 'april_localizer.yaml')

    urdf_file = os.path.join(go2_share, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_default),
        DeclareLaunchArgument('params_file', default_value=params_default),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_name', default_value='go2'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('enable_cmd_vel', default_value='false'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('enable_april', default_value='false'),
        DeclareLaunchArgument('enable_mqtt', default_value='false'),
        DeclareLaunchArgument('enable_joint_state_gui', default_value='false'),
        DeclareLaunchArgument('enable_unitree_read', default_value='false'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('mqtt_broker', default_value='192.168.68.62'),
        DeclareLaunchArgument('mqtt_port', default_value='1883'),
        DeclareLaunchArgument('unitree_interface', default_value='eth0'),
        DeclareLaunchArgument('robot_ack', default_value=''),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'autostart': 'true',
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(enable_rviz),
            output='screen',
        ),

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
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(enable_joint_state_gui),
        ),

        Node(
            package='go2_control',
            executable='go2w_read',
            name='go2w_read',
            condition=IfCondition(enable_unitree_read),
            output='screen',
        ),

        Node(
            package='go2_control',
            executable='cmd_vel_node',
            name='cmd_vel_node',
            condition=IfCondition(enable_cmd_vel),
            parameters=[{
                'network_interface': unitree_interface,
                'robot_ack': robot_ack,
            }],
            output='screen',
        ),

        Node(
            package='go2_control',
            executable='camera',
            name='camera',
            arguments=['wlp4s0'],
            condition=IfCondition(enable_camera),
            output='screen',
        ),

        Node(
            package='go2_control',
            executable='main',
            name='main_node',
            output='screen',
            parameters=[{
                'sim_mode': False,
                'enable_mqtt': enable_mqtt,
                'cmd_vel_enabled': enable_cmd_vel,
                'robot_ack': robot_ack,
                'mqtt_broker': mqtt_broker,
                'mqtt_port': mqtt_port,
            }],
        ),

        Node(
            package='go2_control',
            executable='april_localizer',
            name='april_localizer',
            output='screen',
            condition=IfCondition(enable_april),
            parameters=[april_localizer_param, {
                'cmd_vel_enabled': enable_cmd_vel,
                'robot_ack': robot_ack,
            }],
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc2scan',
            output='screen',
            remappings=[
                ('cloud_in', 'pointcloud'),
                ('scan', '/scan'),
            ],
            parameters=[pc2scan_param],
        ),
    ])
