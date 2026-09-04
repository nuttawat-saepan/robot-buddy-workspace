#!/usr/bin/env bash
# Source this for anything that runs entirely on one machine: replaying a bag,
# the MiniPC-only arrangement, or a bench test.
#
#     source scripts/setup_local_env.sh
#     ros2 launch go2_control livox_mid360_lio.launch.py enable_driver:=false
#
# This is the third of four environments and they are not interchangeable:
#
#     setup_local_env.sh    one machine, Fast DDS on loopback     <- this file
#     setup_robot_env.sh    the board, CycloneDDS, two interfaces
#     setup_ground_env.sh   the ground station, CycloneDDS
#     setup_sdk_env.sh      the Unitree bridge, CycloneDDS, domain 0
#
# Getting the wrong one is not an error. `ros2 topic list` comes back empty, or
# a topic has a publisher and delivers nothing, and neither says why. That is
# what the summary at the end of this file is for: read it every time.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "$HERE")"

source /opt/ros/foxy/setup.bash
[ -f "$HOME/ws_livox/install/setup.bash" ] && source "$HOME/ws_livox/install/setup.bash"
[ -f "$HOME/ws_fastlio_livox/install/setup.bash" ] && source "$HOME/ws_fastlio_livox/install/setup.bash"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

# Fast DDS, not CycloneDDS: the two cannot talk to each other at all, and the
# rest of this project's single-machine work is measured with Fast DDS.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Foxy's Fast DDS shared memory transport can keep a port locked after an
# unclean exit, and the next run then has publishers that deliver nothing with
# no error anywhere. UDP on loopback is slightly slower and does not do that.
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/src/go2_control/config/fastdds_udp_only.xml"

# Nothing leaves this machine. Without it a bench replay is discoverable by the
# board and by anything else on the same network, which has caused a replay and
# a live run to talk to each other.
export ROS_LOCALHOST_ONLY=1

# Deliberately not the site domain. A local replay must not join the field
# graph even if both happen to be running.
export ROS_DOMAIN_ID=0

# Every node in this project launches with output='screen', which on Foxy means
# the terminal and nowhere else: launch.log gets three lines about processes
# starting and not one line of what they said. FAST-LIO's "lidar loop back,
# clear buffer" - the warning that it is about to diverge - has never been on
# disk once. This overrides output= for every action without touching the
# launch files, and it is what makes collect_logs.sh worth running.
export OVERRIDE_LAUNCH_PROCESS_OUTPUT=both

echo "local environment ready - one machine, nothing leaves it"
echo "  RMW              $RMW_IMPLEMENTATION"
echo "  profiles         $(basename "$FASTRTPS_DEFAULT_PROFILES_FILE")"
echo "  ROS_DOMAIN_ID    $ROS_DOMAIN_ID"
echo "  LOCALHOST_ONLY   $ROS_LOCALHOST_ONLY"
echo
echo "  FAST-LIO diverges when starved of CPU - it does not slow down, the pose"
echo "  runs away and every process stays alive. Check the load first:"
echo "    uptime      currently $(cut -d' ' -f1-3 /proc/loadavg)"
echo
echo "  replay:  ros2 launch go2_control livox_mid360_lio.launch.py \\"
echo "               enable_driver:=false enable_tf_bridge:=false"
echo "           ros2 launch go2_control livox_amcl.launch.py"
echo "           ros2 bag play ~/livox_bags_field/02_loop      # no --clock on Foxy"
