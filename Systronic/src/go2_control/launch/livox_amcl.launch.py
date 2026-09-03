"""AMCL on a Livox Mid-360 map: map_server + AMCL + the 2D projection.

Localisation only. Nothing here publishes /cmd_vel, starts cmd_vel_node, or
brings up a Nav2 controller. There is no path from this launch file to robot
motion.

It does not start FAST-LIO or the sensor either. Run one of those first and
this second, because the scan it projects, /cloud_registered_body, only exists
while FAST-LIO is running.

Replay against a recorded bag, which is what this was written for:

    # terminal 1 - FAST-LIO, with its own tf bridge off: this file owns the
    # two static transforms, and they need the tilt of the bag being replayed
    # rather than the robot mount pose hard-coded there. FAST-LIO stamps its
    # output from the message headers, so it needs no sim time of its own.
    ros2 launch go2_control livox_mid360_lio.launch.py \
        enable_driver:=false enable_tf_bridge:=false

    # terminal 2 - defaults to replay:=true, which starts bag_clock and puts
    # every node here on sim time
    ros2 launch go2_control livox_amcl.launch.py

    # terminal 3
    ros2 bag play ~/livox_bags_field/02_loop

On the robot the sensor is bolted on at the published 13 degree mount pose, the
clock is the wall clock, and the map is a real one:

    ros2 launch go2_control livox_amcl.launch.py \
        replay:=false map:=/path/to/site_map.yaml \
        lio_pitch_deg:=13.0 lio_roll_deg:=0.0 sensor_height:=0.35

## Replaying a bag needs a clock, and Foxy will not give you one

`ros2 bag play` gained `--clock` in Galactic. On Foxy the bag replays its
recorded stamps while every node reads the wall clock, and the two are days
apart on these bags. That is not a cosmetic difference:
`pointcloud_to_laserscan` stamps /scan with `now()` instead of copying the
stamp of the cloud it projected, so on the wall clock the scan is dated days
after the transform it needs, and AMCL drops every single one with

    Message Filter dropping message: frame 'base_link' ... for reason 'Unknown'

publishing no pose and never saying that time was the problem. `replay:=true`
fixes it by running go2_control's bag_clock, which republishes the bag's own
header stamps as /clock, and by putting the nodes below on sim time.

## The frame tree, and why the two static transforms are here

    map --(AMCL)--> odom --(static)--> camera_init --(FAST-LIO)--> body
                                                    --(static)--> base_link

FAST-LIO's world frame camera_init inherits the sensor's *orientation* at
startup, so it is tilted by however the sensor happened to be held or mounted.
Two consequences, and both are why this cannot be left as identity:

  * odom has to be gravity-aligned. Every costmap, footprint and height filter
    downstream assumes it. odom->camera_init therefore carries the levelling
    rotation.
  * base_link has to be gravity-aligned too, because pointcloud_to_laserscan
    slices a horizontal band in it. Slice a tilted frame and the cut runs
    diagonally through the floor, painting floor as wall across the room -
    the solid black blob in LIVOX_FIELD_DAY_2026-08-19.md section 6.
    body->base_link therefore carries the inverse of that rotation, plus the
    drop from the sensor down to the floor.

The tilt has to match whatever the map was built with, because AMCL correlates
a slice of the live cloud against a slice of that map. pcd_to_map levels with
`points @ (Ry(pitch) @ Rx(roll)).T`, so the rotation reproduced below is
exactly Ry(pitch) @ Rx(roll) and the map frame is the levelled camera_init
frame, offset only by the origin in the .yaml. That is also why the AMCL seed
is the origin: at t=0 the sensor sits at camera_init's origin, so map and odom
coincide there.

The defaults are the values measured off 02_loop - pitch +10.26, roll +1.72,
sensor 0.43 m up - not the datasheet 13 degrees, which describes a mount the
handheld bags never used.
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

    height = farg('sensor_height')
    base_frame = arg('base_frame')

    # Shared with livox_slam.launch.py so the two cannot drift apart.
    odom_tf, body_tf = lio_bridges(farg('lio_pitch_deg'),
                                   farg('lio_roll_deg'), height)

    map_yaml = arg('map')
    if not os.path.isabs(map_yaml):
        map_yaml = os.path.join(go2_share, 'map', map_yaml)

    amcl_param = arg('amcl_params_file')
    if not os.path.isabs(amcl_param):
        amcl_param = os.path.join(go2_share, 'config', amcl_param)
    pc2scan_param = os.path.join(go2_share, 'pc2scan_livox_lio.yaml')
    replay = arg('replay').lower() in ('true', '1')
    # Sim time is not a separate choice from replaying: without a bag there is
    # no /clock, and with a bag nothing works without one.
    use_sim_time = replay

    overrides = {
        'use_sim_time': use_sim_time,
        'base_frame_id': base_frame,
        'odom_frame_id': arg('odom_frame'),
        'global_frame_id': arg('map_frame'),
        'initial_pose.x': farg('initial_x'),
        'initial_pose.y': farg('initial_y'),
        'initial_pose.yaw': farg('initial_yaw'),
    }

    nodes = []

    if replay:
        # Started first and deliberately on the system clock: everything below
        # blocks at t=0 until this publishes.
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
                arg('odom_frame'), 'camera_init'],
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
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[amcl_param,
                        {'use_sim_time': use_sim_time,
                         'yaml_filename': map_yaml,
                         'frame_id': arg('map_frame')}],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_param, overrides],
            remappings=[('scan', arg('scan_topic'))],
        ),

        # Without this both nodes sit unconfigured and neither the map nor
        # map->odom ever appears, with nothing logged to say why.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),

        Node(
            package='go2_control',
            executable='amcl_drift_check',
            name='amcl_drift_check',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_drift_check')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_frame': arg('map_frame'),
                'odom_frame': arg('odom_frame'),
                'base_frame': base_frame,
                'csv_path': arg('drift_csv'),
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
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
            'map', default_value='livox_02loop.yaml',
            description='Map .yaml for map_server. A bare name is looked up in '
                        'the package map/ directory; an absolute path is used '
                        'as given. The default is the bench artefact built '
                        'from 02_loop, not a site map.'),
        DeclareLaunchArgument(
            'cloud_topic', default_value='/cloud_registered_body',
            description='Cloud to project into /scan. The FAST-LIO deskewed '
                        'cloud, not the raw /livox/lidar: measured beam fill '
                        'is 54.5% mean with std 2.9, against 36-49% and '
                        'unstable for the raw cloud.'),
        DeclareLaunchArgument(
            'amcl_params_file', default_value='amcl_livox.yaml',
            description='AMCL parameter file. A bare name is looked up in the '
                        'package config/ directory. Pass amcl_livox_lowcpu.yaml '
                        'when measure_board_load.sh reports OVER.'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument(
            'lio_pitch_deg', default_value='10.26',
            description='Sensor tilt to undo, positive nose-down. Must match '
                        'the --pitch-deg the map was built with. 10.26 is the '
                        'measured tilt of the handheld 02_loop bag; the Go2 '
                        'mount is 13.0.'),
        DeclareLaunchArgument(
            'lio_roll_deg', default_value='1.72',
            description='Sensor roll to undo. Must match the --roll-deg the '
                        'map was built with.'),
        DeclareLaunchArgument(
            'sensor_height', default_value='0.43',
            description='Sensor height above the floor at the start of the '
                        'run, metres. Sets where base_link sits, and so which '
                        'band of the cloud pointcloud_to_laserscan cuts.'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'initial_x', default_value='0.0',
            description='AMCL seed pose in the map frame. The map is the '
                        'levelled camera_init frame, so the origin is where '
                        'the run started and 0,0,0 is correct for a replay of '
                        'the bag the map was built from.'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'enable_drift_check', default_value='true',
            description='Run the node that measures AMCL against FAST-LIO.'),
        DeclareLaunchArgument(
            'drift_csv', default_value='',
            description='Optional path for a per-sample CSV from the drift '
                        'check. Empty writes no file.'),
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        DeclareLaunchArgument(
            'replay', default_value='true',
            description='Replaying a bag rather than running live. Starts '
                        'bag_clock to publish /clock from the bag stamps and '
                        'puts every node here on sim time, which Foxy rosbag2 '
                        'cannot do for itself. Set false on the robot.'),
        DeclareLaunchArgument(
            'clock_topic', default_value='/livox/imu',
            description='Topic whose header stamps drive /clock in replay '
                        'mode. The IMU runs at 200 Hz on these bags; the '
                        'lidar would give a 100 ms clock granularity.'),
        OpaqueFunction(function=_launch_setup),
    ])
