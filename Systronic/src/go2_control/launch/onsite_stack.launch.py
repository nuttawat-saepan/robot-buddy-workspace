"""Everything the robot needs, in one launch, in the right order.

On 2026-09-07 this stack was brought up by hand in six terminals on the board,
each needing four `source` lines first, and the day lost more time to that than
to any bug. Two failures came from it repeatedly:

  * Nav2's controller_server refuses to activate when the sensors are not yet
    publishing TF, and the lifecycle manager then aborts the whole bringup with
    "Failed to bring up all requested nodes". Six terminals rely on the
    operator waiting long enough between two of them; this file waits.

  * The board's ~/.bashrc sources an older go2_control from ~/UnitreeRos, so
    every new shell finds the wrong package unless the workspace is sourced
    over the top. One launch means one shell to get right instead of six.

## What is not here, and why

`unitree_udp_bridge` is deliberately excluded. It is the only process that can
put velocity on the robot's legs, and it should keep costing a deliberate
command with the acknowledgement phrase spelled out in it:

    ros2 run go2_control unitree_udp_bridge --mode raw --interface eth0 \\
        --max-linear 0.25 --robot-ack I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT

Starting the stack and arming the robot are different decisions and should stay
different commands. Everything below plans, localises, talks to the web and
photographs; none of it can move anything.

RViz also stays out. It belongs on the ground station, which is a different
machine and does not need the rest of this.

## Running it

    cd ~/go2_ws
    source ~/ws_livox/install/setup.bash
    source ~/ws_fastlio_livox/install/setup.bash
    source scripts/setup_robot_env.sh
    ros2 launch go2_control onsite_stack.launch.py map:=$SITE_MAP

Add `enable_web:=false` to leave the MQTT bridge out, `enable_capture:=false`
to leave the camera out, and `enable_relay:=false` to leave even the UDP relay
out - the relay cannot move the robot on its own, but a run that will never be
armed has no reason to carry it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('go2_control')

    def arg(name):
        return LaunchConfiguration(name)

    launches = os.path.join(share, 'launch')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launches, 'livox_robot.launch.py')),
        launch_arguments={
            'replay': arg('replay'),
            'enable_cmd_vel': 'false',
            'lio_pitch_deg': arg('lio_pitch_deg'),
            'lio_roll_deg': arg('lio_roll_deg'),
            'sensor_height': arg('sensor_height'),
        }.items(),
    )

    ground = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launches, 'livox_ground.launch.py')),
        launch_arguments={
            'replay': arg('replay'),
            'map': arg('map'),
            'enable_rviz': 'false',
        }.items(),
    )

    map_points = Node(
        package='go2_control',
        executable='occupancy_grid_points',
        name='occupancy_grid_points',
        output='screen',
    )

    relay = Node(
        package='go2_control',
        executable='cmd_vel_udp_relay',
        name='cmd_vel_udp_relay',
        output='screen',
        condition=IfCondition(arg('enable_relay')),
    )

    web = Node(
        package='go2_control',
        executable='mqtt_mission_bridge',
        name='mqtt_mission_bridge',
        output='screen',
        parameters=[{'broker': arg('mqtt_broker')}],
        condition=IfCondition(arg('enable_web')),
    )

    capture = Node(
        package='go2_control',
        executable='mission_capture',
        name='mission_capture',
        output='screen',
        parameters=[{'source': arg('capture_source')}],
        condition=IfCondition(arg('enable_capture')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', description='Absolute path to the site map yaml.'),
        DeclareLaunchArgument(
            'replay', default_value='false',
            description='true only when following a bag clock. The default is '
                        'false because a live run that inherits true waits '
                        'forever for a /clock that never comes.'),
        DeclareLaunchArgument('lio_pitch_deg', default_value='13.0'),
        DeclareLaunchArgument('lio_roll_deg', default_value='0.0'),
        DeclareLaunchArgument('sensor_height', default_value='0.35'),
        DeclareLaunchArgument(
            'mqtt_broker',
            default_value=os.environ.get('MQTT_BROKER', '192.168.80.233')),
        DeclareLaunchArgument('enable_web', default_value='true'),
        DeclareLaunchArgument('enable_capture', default_value='true'),
        DeclareLaunchArgument(
            'enable_relay', default_value='true',
            description='The relay forwards /cmd_vel_safe to a UDP port. On '
                        'its own it moves nothing - the Unitree bridge on the '
                        'far end of that port is what does, and it is not '
                        'started here.'),
        DeclareLaunchArgument(
            'capture_source', default_value='rtsp',
            description='rtsp is the only source that works on this robot: '
                        'the camera topic is on ROS_DOMAIN_ID 0, where every '
                        'ROS node of ours segfaults.'),

        # The sensors first, alone. FAST-LIO also needs the robot to stand
        # still while it initialises its IMU - start this with nobody touching
        # the robot, or the pose runs away to tens of thousands of metres with
        # every process still alive and no error anywhere.
        robot,

        # Then Nav2, once TF exists. Fifteen seconds is not a guess: on the
        # board FAST-LIO reported "IMU Initial Done" about nine seconds after
        # launch, and the watchdog said "sensors fresh again" a fraction later.
        # Starting Nav2 before that is what aborts the bringup.
        TimerAction(period=15.0, actions=[ground]),

        # And the rest once there is a map and a pose to work with.
        TimerAction(period=25.0, actions=[map_points, relay, web, capture]),
    ])
