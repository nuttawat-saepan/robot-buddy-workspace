"""Everything that runs on the ground station.

The other half of livox_robot.launch.py: map, localisation, planning and the
operator's view. Nothing here touches the robot directly - Nav2's velocity goes
to the robot side over the network, where the watchdog and cmd_vel_node decide
whether it reaches the legs.

    ON THE ROBOT                          ON THE GROUND (this file)
    livox driver                          map_server, amcl
    fastlio_mapping                       Nav2 planner/controller/bt/recoveries
    pointcloud_to_laserscan               lifecycle managers
    TF bridges, lio_odom_relay            rviz
    sensor_watchdog, cmd_vel_node         amcl_drift_check

The ground station is a role, not a specific machine: any Ubuntu 20.04 machine
with Foxy that can join the robot's network will do, and a laptop is easier in
the field because it brings its own power.

## Running it

    # in the field
    ros2 launch go2_control livox_ground.launch.py replay:=false \
        map:=/abs/path/site_map.yaml

    # at the desk against a bag, alongside livox_robot.launch.py
    ros2 launch go2_control livox_ground.launch.py enable_rviz:=true

`replay` must match on both machines: it decides whether they follow the bag
clock published by the robot side or the wall clock.

## /cmd_vel

Nav2's controller and its recoveries publish to `cmd_vel_topic`, which defaults
to `/cmd_vel_nav_preview` - the topic livox_robot.launch.py's watchdog reads.
That watchdog then delivers to `/cmd_vel_safe`, which nothing listens to, so
the default configuration plans without commanding anything. Both ends have to
be changed to arm the system.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


NAV2_NODES = [
    'controller_server',
    'planner_server',
    'recoveries_server',
    'bt_navigator',
    'waypoint_follower',
]


def _launch_setup(context, *args, **kwargs):
    go2_share = get_package_share_directory('go2_control')

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def farg(name):
        return float(arg(name))

    use_sim_time = arg('replay').lower() in ('true', '1')
    common = {'use_sim_time': use_sim_time}
    base_frame = arg('base_frame')
    cmd_vel_topic = arg('cmd_vel_topic')

    map_yaml = arg('map')
    if not os.path.isabs(map_yaml):
        map_yaml = os.path.join(go2_share, 'map', map_yaml)

    nav2_params = arg('nav2_params_file')
    if not os.path.isabs(nav2_params):
        nav2_params = os.path.join(go2_share, 'config', nav2_params)

    amcl_param = os.path.join(go2_share, 'config', 'amcl_livox.yaml')

    # Foxy's bt_navigator opens default_bt_xml_filename as given rather than
    # resolving it against the nav2_bt_navigator share directory, so a bare
    # filename aborts the whole lifecycle bringup during configure.
    bt_xml = arg('bt_xml')
    if not os.path.isabs(bt_xml):
        bt_xml = os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', bt_xml)

    amcl_overrides = {
        'base_frame_id': base_frame,
        'odom_frame_id': arg('odom_frame'),
        'global_frame_id': arg('map_frame'),
        'initial_pose.x': farg('initial_x'),
        'initial_pose.y': farg('initial_y'),
        'initial_pose.yaw': farg('initial_yaw'),
    }
    amcl_overrides.update(common)

    banner = (
        '*** cmd_vel_topic is /cmd_vel: THIS RUN CAN MOVE THE REAL ROBOT ***'
        if cmd_vel_topic in ('/cmd_vel', 'cmd_vel') else
        f'Nav2 velocity goes to {cmd_vel_topic}, which the robot-side watchdog '
        'gates before anything reaches the legs')

    return [
        Node(
            package='nav2_map_server', executable='map_server',
            name='map_server', output='screen',
            parameters=[amcl_param, common,
                        {'yaml_filename': map_yaml,
                         'frame_id': arg('map_frame')}],
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            output='screen',
            parameters=[amcl_param, amcl_overrides],
            remappings=[('scan', arg('scan_topic'))],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                         'node_names': ['map_server', 'amcl']}],
        ),

        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', output='screen',
            parameters=[nav2_params, common],
            remappings=[('cmd_vel', cmd_vel_topic),
                        ('odom', arg('odom_topic'))],
        ),
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', output='screen',
            parameters=[nav2_params, common],
        ),
        Node(
            package='nav2_recoveries', executable='recoveries_server',
            name='recoveries_server', output='screen',
            parameters=[nav2_params, common],
            remappings=[('cmd_vel', cmd_vel_topic)],
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', output='screen',
            parameters=[nav2_params, common,
                        {'default_bt_xml_filename': bt_xml}],
        ),
        Node(
            package='nav2_waypoint_follower', executable='waypoint_follower',
            name='waypoint_follower', output='screen',
            parameters=[nav2_params, common],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                         'node_names': NAV2_NODES}],
        ),

        Node(
            package='go2_control', executable='amcl_drift_check',
            name='amcl_drift_check', output='screen',
            condition=IfCondition(LaunchConfiguration('enable_drift_check')),
            parameters=[common, {'map_frame': arg('map_frame'),
                                 'odom_frame': arg('odom_frame'),
                                 'base_frame': base_frame,
                                 'csv_path': arg('drift_csv')}],
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            condition=IfCondition(LaunchConfiguration('enable_rviz')),
            arguments=['-d', os.path.join(go2_share, 'rviz', arg('rviz_config'))],
            parameters=[common],
        ),

        LogInfo(msg=banner),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='livox_slam_02loop.yaml',
            description='Map .yaml. A bare name is looked up in the package '
                        'map/ directory. The default is the bench artefact '
                        'built from 02_loop, not a site map.'),
        DeclareLaunchArgument('nav2_params_file',
                              default_value='nav2_livox_go2.yaml'),
        DeclareLaunchArgument(
            'bt_xml',
            default_value='navigate_w_replanning_and_recovery.xml'),
        DeclareLaunchArgument(
            'replay', default_value='true',
            description='Follow the bag clock. Must match '
                        'livox_robot.launch.py.'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel_nav_preview',
            description='Where Nav2 publishes velocity. Must match '
                        'nav_cmd_topic in livox_robot.launch.py.'),
        DeclareLaunchArgument('enable_drift_check', default_value='true'),
        DeclareLaunchArgument('drift_csv', default_value=''),
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config',
                              default_value='livox_mapping.rviz'),
        OpaqueFunction(function=_launch_setup),
    ])
