"""Everything that runs on the Unitree board.

One command for the robot side of the two-machine deployment. The ground
station runs livox_ground.launch.py.

    ON THE ROBOT                          ON THE GROUND
    livox driver                          map_server, amcl
    fastlio_mapping                       Nav2 planner/controller/bt
    pointcloud_to_laserscan               rviz
    TF bridges, lio_odom_relay
    sensor_watchdog, cmd_vel_node

The split follows one rule: the point cloud never crosses the network. It is
41.6 Mbps fragmented into hundreds of UDP packets per frame, and that is what
failed in the field on 2026-08-19. Everything that consumes the cloud -
FAST-LIO and the projection to /scan - therefore lives here, and only the 3 KB
scan, the odometry and the TF go over Wi-Fi.

## Movement is off by default and takes two separate keys to turn on

    enable_cmd_vel     starts cmd_vel_node at all
    robot_ack          the exact gate string, checked by cmd_vel_node itself
    cmd_vel_topic      where the watchdog delivers, /cmd_vel_safe by default

With the defaults, no node publishes to /cmd_vel and `ros2 topic info /cmd_vel`
reports the topic does not exist. That check is the standing field verification
and it must keep working through every non-movement test.

## Running it

    # on the robot, live
    ros2 launch go2_control livox_robot.launch.py replay:=false

    # at the desk against a bag, on one machine with livox_ground.launch.py
    ros2 launch go2_control livox_robot.launch.py enable_driver:=false
    ros2 bag play ~/livox_bags_field/02_loop

Both machines must agree on `replay`, because it decides whether they follow
the bag clock or the wall clock.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, \
    LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from go2_control.frames import lio_bridges


def _launch_setup(context, *args, **kwargs):
    go2_share = get_package_share_directory('go2_control')

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def farg(name):
        return float(arg(name))

    def barg(name):
        return arg(name).lower() in ('true', '1')

    replay = barg('replay')
    use_sim_time = replay
    base_frame = arg('base_frame')
    odom_frame = arg('odom_frame')
    common = {'use_sim_time': use_sim_time}

    odom_tf, body_tf = lio_bridges(
        farg('lio_pitch_deg'), farg('lio_roll_deg'), farg('sensor_height'))

    lio_param = os.path.join(go2_share, 'config', 'fast_lio2_mid360.yaml')
    pc2scan_param = os.path.join(go2_share, 'pc2scan_livox_lio.yaml')

    cmd_vel_topic = arg('cmd_vel_topic')
    armed = cmd_vel_topic in ('/cmd_vel', 'cmd_vel')

    nodes = []

    # Replaying a bag on Foxy needs a clock: ros2 bag play gained --clock only
    # in Galactic. bag_clock is started here rather than on the ground station
    # because the robot side is where the bag is played back from.
    if replay:
        nodes.append(Node(
            package='go2_control', executable='bag_clock', name='bag_clock',
            output='screen',
            parameters=[{'use_sim_time': False,
                         'topic': arg('clock_topic'),
                         'type': 'sensor_msgs/msg/Imu'}],
        ))

    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_share, 'launch', 'livox_mid360_driver.launch.py')),
        condition=IfCondition(str(barg('enable_driver') and not replay).lower()),
        launch_arguments={'frame_id': arg('lidar_frame')}.items(),
    ))

    nodes += [
        Node(
            package='fast_lio_livox', executable='fastlio_mapping',
            name='fastlio_mapping', output='screen',
            parameters=[lio_param, common,
                        {'pcd_save.pcd_save_en': ParameterValue(
                            LaunchConfiguration('save_pcd'), value_type=bool)}],
            # The whole map is written during shutdown. Launch escalates to
            # SIGTERM 5 s after SIGINT by default, which kills a long write
            # part way through and leaves no file.
            sigterm_timeout='60',
        ),

        # FAST-LIO's camera_init inherits the sensor's orientation at startup,
        # so both of these carry the mount tilt. The arithmetic is shared with
        # livox_ground.launch.py through go2_control.frames so the two cannot
        # drift apart; if they did, the live scan and the map would be slices
        # of differently-tilted worlds.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='lio_odom_bridge', output='screen',
            arguments=[f'{v:.6f}' for v in odom_tf] + [odom_frame, 'camera_init'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='lio_body_bridge', output='screen',
            arguments=[f'{v:.6f}' for v in body_tf] + ['body', base_frame],
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc2scan_livox_lio', output='screen',
            remappings=[('cloud_in', arg('cloud_topic')),
                        ('scan', arg('scan_topic'))],
            parameters=[pc2scan_param, common, {'target_frame': base_frame}],
        ),

        Node(
            package='go2_control', executable='lio_odom_relay',
            name='lio_odom_relay', output='screen',
            parameters=[common, {'odom_frame': odom_frame,
                                 'base_frame': base_frame,
                                 'odom_topic': arg('odom_topic')}],
        ),

        # Layer 2 of the safety chain: blocks commands when the sensors stop,
        # which cmd_vel_node cannot detect because /cmd_vel keeps arriving.
        Node(
            package='go2_control', executable='sensor_watchdog',
            name='sensor_watchdog', output='screen',
            # Deliberately not `common`: the watchdog runs on wall time even
            # during a replay. Under sim time rclpy drives its timer from
            # /clock, which stops when the bag stops - so the node would freeze
            # at exactly the moment it is needed. See sensor_watchdog.py.
            parameters=[{'use_sim_time': False}, {
                'input_topic': arg('nav_cmd_topic'),
                'output_topic': cmd_vel_topic,
                'scan_topic': arg('scan_topic'),
                'odom_topic': arg('odom_topic'),
                'sensor_timeout': farg('sensor_timeout'),
            }],
        ),

        # Layer 1, and the only thing that can move the robot. It refuses to
        # start without the exact robot_ack string, checked inside the node.
        Node(
            package='go2_control', executable='cmd_vel_node',
            name='cmd_vel_node', output='screen',
            condition=IfCondition(LaunchConfiguration('enable_cmd_vel')),
            parameters=[common, {
                'robot_ack': arg('robot_ack'),
                'network_interface': arg('unitree_interface'),
                'cmd_timeout': farg('cmd_timeout'),
            }],
            remappings=[('/cmd_vel', cmd_vel_topic)],
        ),

        # Unitree read-only telemetry. publish_odom and publish_tf are false
        # because FAST-LIO owns odometry on this stack: two publishers of
        # odom -> base_link break the TF tree silently, with tf2 keeping
        # whichever message arrived last.
        Node(
            package='go2_control', executable='go2w_read',
            name='go2w_read', output='screen',
            condition=IfCondition(LaunchConfiguration('enable_unitree_read')),
            parameters=[common, {'publish_odom': False, 'publish_tf': False}],
        ),
    ]

    banner = (
        '*** cmd_vel_topic is /cmd_vel: THIS RUN CAN MOVE THE REAL ROBOT ***'
        if armed else
        f'velocity output goes to {cmd_vel_topic}; /cmd_vel has no publisher')
    nodes.append(LogInfo(msg=banner))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'replay', default_value='true',
            description='Follow a bag clock instead of the wall clock. Must '
                        'match livox_ground.launch.py. Set false on the robot.'),
        DeclareLaunchArgument('clock_topic', default_value='/livox/imu'),
        DeclareLaunchArgument(
            'enable_driver', default_value='true',
            description='Start the real Livox driver. Ignored when replaying.'),
        DeclareLaunchArgument('lidar_frame', default_value='livox_frame'),
        DeclareLaunchArgument('cloud_topic',
                              default_value='/cloud_registered_body'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'lio_pitch_deg', default_value='10.26',
            description='Mount tilt to undo. Must match the map. 10.26 is what '
                        'was fitted from 02_loop; the Unitree figure is 13.0.'),
        DeclareLaunchArgument('lio_roll_deg', default_value='1.72'),
        DeclareLaunchArgument('sensor_height', default_value='0.43'),
        DeclareLaunchArgument('save_pcd', default_value='false'),
        DeclareLaunchArgument(
            'nav_cmd_topic', default_value='/cmd_vel_nav_preview',
            description='Where Nav2 publishes. Must match cmd_vel_topic in '
                        'livox_ground.launch.py.'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel_safe',
            description='Where the watchdog delivers. Deliberately not '
                        '/cmd_vel: with the default, no node publishes to '
                        '/cmd_vel and the standing field check '
                        '"ros2 topic info /cmd_vel" still reports the topic '
                        'does not exist. Setting this to /cmd_vel is part of '
                        'arming the system for the movement stage.'),
        DeclareLaunchArgument('sensor_timeout', default_value='0.5'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),
        DeclareLaunchArgument(
            'enable_cmd_vel', default_value='false',
            description='Start cmd_vel_node, the only node that can move the '
                        'robot. It still refuses to run without robot_ack.'),
        DeclareLaunchArgument('robot_ack', default_value=''),
        DeclareLaunchArgument('unitree_interface', default_value='eth0'),
        DeclareLaunchArgument('enable_unitree_read', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
