from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    go2_share = get_package_share_directory('go2_control')
    sim_map_default = os.path.join(
        go2_share,
        'map',
        'house_map.yaml'
    )
    sim_params_default = os.path.join(go2_share, 'nav2_params.yaml')

    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_cmd_vel = LaunchConfiguration('enable_cmd_vel')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_april = LaunchConfiguration('enable_april')
    enable_mqtt = LaunchConfiguration('enable_mqtt')
    enable_unitree_read = LaunchConfiguration('enable_unitree_read')
    enable_joint_state_gui = LaunchConfiguration('enable_joint_state_gui')
    enable_unitree_bridge = LaunchConfiguration('enable_unitree_bridge')
    enable_stream = LaunchConfiguration('enable_stream')
    enable_init_pose = LaunchConfiguration('enable_init_pose')
    enable_pc2scan = LaunchConfiguration('enable_pc2scan')
    enable_rviz = LaunchConfiguration('enable_rviz')
    mqtt_broker = LaunchConfiguration('mqtt_broker')
    mqtt_port = LaunchConfiguration('mqtt_port')
    unitree_interface = LaunchConfiguration('unitree_interface')
    robot_ack = LaunchConfiguration('robot_ack')
    frame_prefix = LaunchConfiguration('frame_prefix')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('go2_control'),
                'launch',
                'sim.launch.py'
            )
        ),
        condition=LaunchConfigurationEquals('mode', 'sim'),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'frame_prefix': frame_prefix,
            'enable_unitree_bridge': enable_unitree_bridge,
            'enable_april': enable_april,
            'enable_stream': enable_stream,
            'enable_init_pose': enable_init_pose,
            'enable_pc2scan': enable_pc2scan,
            'enable_rviz': enable_rviz,
        }.items()
    )

    real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('go2_control'),
                'launch',
                'real.launch.py'
            )
        ),
        condition=LaunchConfigurationEquals('mode', 'real'),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'enable_cmd_vel': enable_cmd_vel,
            'enable_camera': enable_camera,
            'enable_april': enable_april,
            'enable_mqtt': enable_mqtt,
            'enable_unitree_read': enable_unitree_read,
            'enable_joint_state_gui': enable_joint_state_gui,
            'enable_rviz': enable_rviz,
            'mqtt_broker': mqtt_broker,
            'mqtt_port': mqtt_port,
            'unitree_interface': unitree_interface,
            'robot_ack': robot_ack,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='sim',
            description='sim or real'
        ),
        DeclareLaunchArgument('map', default_value=sim_map_default),
        DeclareLaunchArgument('params_file', default_value=sim_params_default),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument('enable_cmd_vel', default_value='false'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('enable_april', default_value='false'),
        DeclareLaunchArgument('enable_mqtt', default_value='false'),
        DeclareLaunchArgument('enable_unitree_read', default_value='false'),
        DeclareLaunchArgument('enable_joint_state_gui', default_value='false'),
        DeclareLaunchArgument('enable_unitree_bridge', default_value='false'),
        DeclareLaunchArgument('enable_stream', default_value='false'),
        DeclareLaunchArgument('enable_init_pose', default_value='false'),
        DeclareLaunchArgument('enable_pc2scan', default_value='false'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('mqtt_broker', default_value='192.168.68.62'),
        DeclareLaunchArgument('mqtt_port', default_value='1883'),
        DeclareLaunchArgument('unitree_interface', default_value='eth0'),
        DeclareLaunchArgument('robot_ack', default_value=''),

        sim_launch,
        real_launch
    ])
