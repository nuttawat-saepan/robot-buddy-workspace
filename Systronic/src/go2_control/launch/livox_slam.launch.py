"""Build a 2D map from the Mid-360 with slam_toolbox, which ray traces.

Mapping only. Nothing here publishes /cmd_vel, starts cmd_vel_node, or brings
up a Nav2 controller.

## Why not pcd_to_map

pcd_to_map and livox_grid_map both rasterise only the cells a beam stopped in.
Neither marks the cells a beam passed through, so neither measures free space
at all - pcd_to_map fills it in afterwards with a flood fill seeded at the
robot's starting cell. That is a guess about connectivity, and it has already
failed in both directions: without sealing, the fill escaped through a doorway
and reported 90.7% of the map free; with sealing it stops at whatever the
obstacle thickening happens to close.

slam_toolbox traces each beam from the sensor origin, so a cell becomes free
because a beam demonstrably passed through it. The floor comes out of the
measurement rather than out of an assumption.

## What it cannot fix

The Mid-360 sees 7 degrees below horizontal, and the Go2 mount adds 13 degrees
of nose-down tilt, so the lowest ray leaves the sensor about 20 degrees below
level. From 0.43 m up that ray first meets the floor about 1.2 m away, and the
floor inside that radius is never observed from where the robot is standing.
Ray tracing cannot recover it; only driving over the area later can.

## Running it against a replayed bag

    ros2 launch go2_control livox_mid360_lio.launch.py \
        enable_driver:=false enable_tf_bridge:=false
    ros2 launch go2_control livox_slam.launch.py
    ros2 bag play ~/livox_bags_field/02_loop

This file publishes the same odom->camera_init and body->base_link bridges as
livox_amcl.launch.py, from the same shared arithmetic, and the same /clock for
replay. Do not run the two together: both would drive map->odom, and slam
mapping and AMCL localisation are mutually exclusive anyway.

Save the result when the bag ends:

    ros2 run nav2_map_server map_saver_cli -f src/go2_control/map/livox_slam_02loop

On the robot, with the real driver and the wall clock:

    ros2 launch go2_control livox_slam.launch.py \
        replay:=false lio_pitch_deg:=13.0 lio_roll_deg:=0.0 sensor_height:=0.35
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go2_control.frames import lio_bridges


def _launch_setup(context, *args, **kwargs):
    go2_share = get_package_share_directory('go2_control')

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def farg(name):
        return float(arg(name))

    base_frame = arg('base_frame')
    odom_frame = arg('odom_frame')
    replay = arg('replay').lower() in ('true', '1')
    use_sim_time = replay

    odom_tf, body_tf = lio_bridges(
        farg('lio_pitch_deg'), farg('lio_roll_deg'), farg('sensor_height'))

    slam_param = os.path.join(go2_share, 'config', 'slam_livox.yaml')
    pc2scan_param = os.path.join(go2_share, 'pc2scan_livox_lio.yaml')

    nodes = []

    if replay:
        nodes.append(Node(
            package='go2_control',
            executable='bag_clock',
            name='bag_clock',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'topic': arg('clock_topic'),
                'type': 'sensor_msgs/msg/Imu',
            }],
        ))

    return nodes + [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lio_odom_bridge',
            output='screen',
            arguments=[f'{v:.6f}' for v in odom_tf] + [
                odom_frame, 'camera_init'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lio_body_bridge',
            output='screen',
            arguments=[f'{v:.6f}' for v in body_tf] + ['body', base_frame],
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc2scan_livox_lio',
            output='screen',
            remappings=[
                ('cloud_in', arg('cloud_topic')),
                ('scan', arg('scan_topic')),
            ],
            parameters=[pc2scan_param,
                        {'use_sim_time': use_sim_time,
                         'target_frame': base_frame}],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_param, {
                'use_sim_time': use_sim_time,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'map_frame': arg('map_frame'),
                'scan_topic': arg('scan_topic'),
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_slam',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_rviz')),
            arguments=['-d', os.path.join(go2_share, 'rviz',
                                          'livox_mapping.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cloud_topic', default_value='/cloud_registered_body',
            description='Cloud to project. The FAST-LIO deskewed cloud, not '
                        'the raw /livox/lidar.'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument(
            'lio_pitch_deg', default_value='10.26',
            description='Sensor tilt to undo, positive nose-down. 10.26 is '
                        'what was fitted from the 02_loop cloud; the Unitree '
                        'mount figure is 13.0.'),
        DeclareLaunchArgument('lio_roll_deg', default_value='1.72'),
        DeclareLaunchArgument(
            'sensor_height', default_value='0.43',
            description='Sensor height above the floor at the start, metres. '
                        'Sets where base_link sits and so which band of the '
                        'cloud becomes /scan.'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'replay', default_value='true',
            description='Publish /clock from the bag. Set false on the robot.'),
        DeclareLaunchArgument('clock_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
