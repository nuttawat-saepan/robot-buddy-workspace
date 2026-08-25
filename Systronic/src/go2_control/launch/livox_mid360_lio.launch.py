"""FAST-LIO2 odometry from a Livox Mid-360. Sensor and odometry only.

Nothing here publishes /cmd_vel or starts cmd_vel_node.

Requires the fast_lio_livox package from ~/ws_fastlio_livox, which is upstream
hku-mars/FAST_LIO (ROS2 branch) renamed to avoid colliding with the fast_lio
package this workspace builds from src/FAST_LIO_Hesai. That Hesai fork has its
MID360 handler removed and cannot read this sensor.

    source /opt/ros/foxy/setup.bash
    source ~/ws_livox/install/setup.bash
    source ~/ws_fastlio_livox/install/setup.bash
    source install/setup.bash

Rehearsal with no hardware:

    ros2 launch go2_control livox_mid360_lio.launch.py use_fake:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    go2_share = get_package_share_directory('go2_control')
    lio_param = os.path.join(go2_share, 'config', 'fast_lio2_mid360.yaml')

    use_fake = LaunchConfiguration('use_fake')
    enable_driver = LaunchConfiguration('enable_driver')
    # The node declares pcd_save_en as a bool, so the launch argument has to be
    # coerced; passing the raw substitution would hand it the string 'false'.
    save_pcd = ParameterValue(
        LaunchConfiguration('save_pcd'), value_type=bool)
    enable_tf_bridge = LaunchConfiguration('enable_tf_bridge')
    lidar_frame = LaunchConfiguration('lidar_frame')
    base_frame = LaunchConfiguration('base_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake', default_value='false',
            description='Publish a synthetic Mid-360 instead of starting the driver.'),
        DeclareLaunchArgument(
            'enable_driver', default_value='true',
            description='Start the real Livox driver. Ignored when use_fake is true.'),
        DeclareLaunchArgument(
            'enable_tf_bridge', default_value='false',
            description=(
                'Publish odom->camera_init and body->base_frame, tying FAST-LIO '
                'into the Go2 frame tree using the mount pose below. Still off '
                'by default: with the sensor off the robot, or handheld, the '
                'bridge would place base_link somewhere it is not.')),
        DeclareLaunchArgument(
            'save_pcd', default_value='false',
            description=(
                'Accumulate the whole run in RAM and write it out on shutdown. '
                'Upstream writes to <FAST_LIO source dir>/PCD/scans.pcd, which '
                'is fixed at compile time and cannot be moved from here. Off by '
                'default: it costs memory and slows the update loop, so it is '
                'only wanted on a deliberate mapping run.')),
        DeclareLaunchArgument('lidar_frame', default_value='livox_frame'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),

        # Real driver. xfer_format 0 so one instance feeds both FAST-LIO
        # (lidar_type 4) and pointcloud_to_laserscan.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_share, 'launch', 'livox_mid360_driver.launch.py')),
            condition=IfCondition(PythonExpression([
                "'", enable_driver, "' == 'true' and '", use_fake, "' == 'false'",
            ])),
            launch_arguments={'frame_id': lidar_frame}.items(),
        ),

        Node(
            package='go2_control',
            executable='fake_livox_mid360',
            name='fake_livox_mid360',
            output='screen',
            condition=IfCondition(use_fake),
            parameters=[{
                'topic': '/livox/lidar',
                'imu_topic': '/livox/imu',
                'frame_id': lidar_frame,
            }],
        ),

        Node(
            package='fast_lio_livox',
            executable='fastlio_mapping',
            name='fastlio_mapping',
            output='screen',
            parameters=[lio_param, {'pcd_save.pcd_save_en': save_pcd}],
            # The whole map is written during shutdown, after Ctrl-C. Launch
            # escalates to SIGTERM 5 s after SIGINT by default, which is enough
            # to kill the write part-way through a long run and leave no file.
            sigterm_timeout='60',
        ),

        # FAST-LIO names its world frame camera_init and its body frame body.
        # Neither matches the Go2 tree, so they have to be tied in explicitly.
        #
        # camera_init sits where the sensor started, which is what odom means,
        # so the translation is zero. The rotation is not: camera_init also
        # inherits the sensor's *orientation* at startup, and the Mid-360 is
        # mounted 13 deg nose-down, so an identity bridge would hand Nav2 an
        # odom frame tilted 13 deg out of level. Everything downstream assumes
        # odom is gravity-aligned - the costmap, the footprint, the height
        # filters - so the tilt is undone here, once, for all consumers.
        #
        # This assumes the robot is standing roughly level when the run starts.
        # Starting on a ramp would bake that ramp into odom.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lio_odom_bridge',
            condition=IfCondition(enable_tf_bridge),
            arguments=['0', '0', '0', '0', '0.22689', '0', 'odom', 'camera_init'],
            output='screen',
        ),
        # Mid-360 mount pose on the Go2, as published by Unitree: the sensor
        # sits at (0.1870, 0, 0.0803) in the body IMU frame, pitched 13 deg
        # about Y. TF wants the transform the other way round, so this is that
        # pose inverted:
        #
        #     R' = Ry(-13 deg)              -> pitch -0.22689 rad
        #     t' = -R' * (0.1870, 0, 0.0803) -> (-0.16414, 0, -0.12031)
        #
        # static_transform_publisher takes x y z yaw pitch roll.
        #
        # Approximation worth knowing about: FAST-LIO's `body` is the Livox
        # internal IMU, while the figure above is to the sensor itself. The two
        # differ by the extrinsic_T in fast_lio2_mid360.yaml, about 4 cm in z.
        # Below the tolerance of a first mapping run, not below the tolerance of
        # a calibration.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lio_body_bridge',
            condition=IfCondition(enable_tf_bridge),
            arguments=['-0.16414', '0.0', '-0.12031',
                       '0.0', '-0.22689', '0.0',
                       'body', base_frame],
            output='screen',
        ),
    ])
