"""Nav2 on the Livox Mid-360 stack: planner, controller, BT navigator.

## /cmd_vel is disconnected here, on purpose and by default

Nav2's controller_server and its spin/backup recoveries publish velocity on the
relative topic `cmd_vel`. Left alone, that resolves to /cmd_vel and the moment
a goal is accepted this launch file would be commanding the robot.

Both are therefore remapped to `cmd_vel_topic`, which defaults to
`/cmd_vel_nav_preview` - a topic nothing subscribes to. Nav2 plans, the path
appears in RViz, the costmaps update, and the velocity goes nowhere.

Pointing `cmd_vel_topic` at /cmd_vel is what turns this into a system that
moves a real robot. Do not do it as a convenience. It belongs to the final
movement stage only: safe area, operator on the remote, emergency stop ready,
lowest speed, and the robot_ack gate in main.py.

This launch file starts no cmd_vel_node and cannot start one.

## What it needs running alongside

    ros2 launch go2_control livox_mid360_lio.launch.py \
        enable_driver:=false enable_tf_bridge:=false      # or the real driver
    ros2 launch go2_control livox_amcl.launch.py          # map, AMCL, /scan, TF
    ros2 launch go2_control livox_nav2.launch.py          # this
    ros2 bag play ~/livox_bags_field/02_loop              # replay only

livox_amcl.launch.py owns the map, AMCL, the projection to /scan and the two
static TF bridges. This file owns /odom and Nav2 and nothing else, so the two
can be brought up and torn down independently.

## Sending a goal without moving anything

RViz "2D Goal Pose", or:

    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, \
          orientation: {w: 1.0}}}}"

On a replayed bag the robot never actually moves, so the goal will not be
reached and the progress checker will eventually abort. That is expected: what
is being verified is that a plan is produced, the costmaps populate from
/scan, and the TF chain holds - not that navigation completes.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Every Nav2 server that has to be walked through the lifecycle, in the order
# the manager should bring them up.
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

    params_file = arg('params_file')
    if not os.path.isabs(params_file):
        params_file = os.path.join(go2_share, 'config', params_file)

    # Matches livox_amcl.launch.py: replaying a bag on Foxy needs the /clock
    # that bag_clock publishes, and every node that reads now() has to be on
    # it. livox_amcl.launch.py starts bag_clock; this file only follows it.
    use_sim_time = arg('replay').lower() in ('true', '1')

    cmd_vel_topic = arg('cmd_vel_topic')
    common = {'use_sim_time': use_sim_time}

    # Foxy's bt_navigator does not resolve default_bt_xml_filename against the
    # nav2_bt_navigator share directory - it opens the string as given, and a
    # bare filename fails with "Couldn't open input XML file" during configure,
    # taking the whole lifecycle bringup down with it. Resolve it here so the
    # parameter file can keep naming the tree rather than a machine path.
    bt_xml = arg('bt_xml')
    if not os.path.isabs(bt_xml):
        bt_xml = os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', bt_xml)

    # A loud, unmissable line in the log. Someone reading a terminal months
    # from now should not have to infer whether this run can move the robot.
    if cmd_vel_topic in ('/cmd_vel', 'cmd_vel'):
        banner = (
            '*** cmd_vel_topic is /cmd_vel: THIS RUN CAN MOVE THE REAL ROBOT. '
            'Operator on the remote, emergency stop ready, lowest speed. ***')
    else:
        banner = (f'velocity output goes to {cmd_vel_topic}, which nothing '
                  'subscribes to - Nav2 plans but commands nothing')

    return [
        Node(
            package='go2_control',
            executable='lio_odom_relay',
            name='lio_odom_relay',
            output='screen',
            parameters=[common, {
                'source_topic': arg('source_odom_topic'),
                'odom_topic': arg('odom_topic'),
                'odom_frame': arg('odom_frame'),
                'base_frame': arg('base_frame'),
            }],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, common],
            remappings=[('cmd_vel', cmd_vel_topic),
                        ('odom', arg('odom_topic'))],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, common],
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[params_file, common],
            remappings=[('cmd_vel', cmd_vel_topic)],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, common,
                        {'default_bt_xml_filename': bt_xml}],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file, common],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': NAV2_NODES,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_rviz')),
            arguments=['-d', os.path.join(go2_share, 'rviz',
                                          'livox_mapping.rviz')],
            parameters=[common],
        ),

        # Logged last so it is the final thing on screen at startup.
        LogInfo(msg=banner),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value='nav2_livox_go2.yaml',
            description='Nav2 parameters. A bare name is looked up in the '
                        'package config/ directory. Do not point this at '
                        'nav2go2_params.yaml: it is written for a newer Nav2 '
                        'and Foxy cannot load its plugins.'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel_nav_preview',
            description='Where Nav2 velocity commands go. The default is a '
                        'topic nothing subscribes to, so Nav2 plans without '
                        'commanding anything. Setting this to /cmd_vel makes '
                        'the run able to move the real robot and belongs to '
                        'the final movement stage only.'),
        DeclareLaunchArgument(
            'replay', default_value='true',
            description='Follow the /clock that livox_amcl.launch.py publishes '
                        'from the bag. Set false on the robot.'),
        DeclareLaunchArgument('source_odom_topic', default_value='/Odometry'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'bt_xml', default_value='navigate_w_replanning_and_recovery.xml',
            description='Behaviour tree. A bare name is resolved against the '
                        'nav2_bt_navigator behavior_trees directory, which '
                        'Foxy does not do for itself.'),
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
