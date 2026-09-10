"""Make a map of a new site, in one launch, without the pieces fighting.

The counterpart to onsite_stack.launch.py: that one localises against a map
that exists, this one draws it. They cannot run together - slam_toolbox and
AMCL both publish `map -> odom`, and two publishers on one transform is not a
configuration, it is a coin toss.

## Why the pair inside is what it is

On site 2026-09-07 mapping was started as livox_robot.launch.py plus
livox_slam.launch.py, and those two overlap: both bring their own
pointcloud_to_laserscan and both bring the two static TF bridges. The result
was two publishers on /scan, `odom -> base_link` disappearing, and slam_toolbox
producing nothing while every process looked healthy.

The combination that does not overlap is:

    livox_mid360_lio   driver and FAST-LIO only, tf bridge off
    livox_slam         the tf bridges, pointcloud_to_laserscan, slam_toolbox

so each node runs exactly once. The tf bridges come from the SLAM half because
they carry the mount pose, and livox_slam and livox_amcl share one definition
of it - if the mount is wrong it is wrong in both, rather than the two
disagreeing.

## Running it

    cd ~/go2_ws
    source ~/ws_livox/install/setup.bash
    source ~/ws_fastlio_livox/install/setup.bash
    source scripts/setup_robot_env.sh
    ros2 launch go2_control mapping_stack.launch.py

Stand the robot still for the first fifteen seconds. FAST-LIO initialises its
IMU from that stillness, and starting it while the robot is being moved is the
leading explanation for the two divergences seen on site - the pose runs away
to tens of thousands of metres with every process alive and no error anywhere.

Then drive the whole site with the remote and **close the loop**: come back to
where you started rather than stopping at the far end. Walk through the places
the robot cannot see from where it stands - the floor within about 1.2 m of it
is never in view, so those patches have to be seen from somewhere else.

## Saving

    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \\
        "{name: {data: '$HOME/go2_ws/src/go2_control/map/site_YYYYMMDD_HHMM'}}"

Not `map_saver_cli`. It needs /map on the wire and waits for a message with a
timeout it measures on the ROS clock, so under `replay:=true` - where the clock
comes from the bag - it hangs for ever once the bag ends. The service writes
from slam_toolbox's own state and does not care.

Afterwards, check the `image:` line in the yaml is a bare filename. map_saver
writes whatever path it was given, and map_server resolves `image` relative to
the yaml rather than the working directory, so a path in there loads nothing
and says nothing.

**Do not retouch the map.** Measured 2026-09-03: hand-editing made AMCL worse
in proportion to how much was edited, and unstable with it. See
LIVOX_AMCL_TUNING_2026-09-03.md.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launches = os.path.join(
        get_package_share_directory('go2_control'), 'launch')

    def arg(name):
        return LaunchConfiguration(name)

    # Driver and FAST-LIO only. The tf bridge is off here because livox_slam
    # brings its own, and two static publishers on the same transform is the
    # overlap that broke mapping on site.
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launches, 'livox_mid360_lio.launch.py')),
        launch_arguments={
            'enable_driver': arg('enable_driver'),
            'enable_tf_bridge': 'false',
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launches, 'livox_slam.launch.py')),
        launch_arguments={
            'lio_pitch_deg': arg('lio_pitch_deg'),
            'lio_roll_deg': arg('lio_roll_deg'),
            'sensor_height': arg('sensor_height'),
            'enable_rviz': arg('enable_rviz'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('lio_pitch_deg', default_value='13.0'),
        DeclareLaunchArgument('lio_roll_deg', default_value='0.0'),
        DeclareLaunchArgument('sensor_height', default_value='0.35'),
        DeclareLaunchArgument(
            'enable_driver', default_value='true',
            description='false to build a map from a replayed bag instead of '
                        'the live sensor.'),
        DeclareLaunchArgument(
            'enable_rviz', default_value='false',
            description='RViz on the board has no screen to draw on. Watch '
                        'from the ground station instead, with '
                        'occupancy_grid_points - a Map display does not '
                        'usually receive /map across two machines on Foxy.'),

        sensors,

        # FAST-LIO reported "IMU Initial Done" about nine seconds after launch
        # on the board. slam_toolbox started before there is a pose has nothing
        # to attach its first scan to.
        TimerAction(period=12.0, actions=[slam]),
    ])
