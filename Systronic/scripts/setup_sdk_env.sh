#!/usr/bin/env bash
# Source this ONLY in the terminal that runs unitree_udp_bridge.
#
#     source scripts/setup_sdk_env.sh
#     ros2 run go2_control unitree_udp_bridge --mode probe
#
# This is the Unitree side of the split. The velocity path crosses from the
# navigation graph to the robot over UDP on 127.0.0.1:32123 precisely because
# these two environments cannot exist in one process:
#
#     cmd_vel_udp_relay     Fast DDS, the navigation graph
#     unitree_udp_bridge    CycloneDDS, this file, talks to the robot
#
# Two things here are different from every other environment in this project
# and both are easy to get wrong:
#
# ROS_DOMAIN_ID is 0, not the site domain. Unitree's own traffic is on domain
# 0, so the bridge has to be there to see the robot at all. Setting the site
# domain here is a natural mistake - every other terminal uses it - and the
# result is a bridge that arms cleanly and never reaches the robot.
#
# CYCLONEDDS_URI points at cyclonedds_unitree_wlan.xml, which uses the legacy
# <NetworkInterfaceAddress> element. ROS 2 Foxy ships CycloneDDS 0.7; given the
# newer <Interfaces>/<NetworkInterface> syntax it creates no participant at all
# and reports nothing. That cost a field session.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "$HERE")"

source /opt/ros/foxy/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

if [ -f "$HERE/onsite.env" ]; then
    source "$HERE/onsite.env"
else
    echo "warning: $HERE/onsite.env not found, using defaults"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$WS/src/go2_control/config/cyclonedds_unitree_wlan.xml"

# Unitree's own DDS traffic is on domain 0. Overrides whatever onsite.env set
# for the rest of the stack, on purpose.
export ROS_DOMAIN_ID=0

# Must be unset here: it would cut the bridge off from the robot, which is not
# on this machine.
unset ROS_LOCALHOST_ONLY

IFACE="${UNITREE_IF:-eth0}"

echo "Unitree SDK environment ready - THIS TERMINAL TALKS TO THE ROBOT"
echo "  RMW              $RMW_IMPLEMENTATION"
echo "  CYCLONEDDS_URI   $(basename "${CYCLONEDDS_URI#file://}")"
echo "  ROS_DOMAIN_ID    $ROS_DOMAIN_ID   (Unitree's own domain, not the site's)"
echo "  interface        $IFACE"
echo
echo "  Wrong interface is the worst failure in this stack: the bridge arms,"
echo "  says it is ready, and no command reaches the legs. On the board it is"
echo "  eth0; on the MiniPC it is the card that reaches the robot."
echo
echo "  check first, sends no motion command, needs no robot_ack:"
echo "    ros2 run go2_control unitree_udp_bridge --mode probe --interface $IFACE"
echo
echo "  then, and only with the operator ready and the area clear:"
echo "    ros2 run go2_control unitree_udp_bridge --mode api --interface $IFACE \\"
echo "        --robot-ack I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT"
