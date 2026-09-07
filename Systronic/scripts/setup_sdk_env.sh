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

# onsite.env holds the site's values, but a value already set in this shell
# has to win over it - that is how you point one terminal at the cable while
# the file says wireless, without editing the file and forgetting to put it
# back. Without this the override is accepted silently and ignored, which is
# the same failure the file itself was causing.
_pre_UNITREE_IF="${UNITREE_IF:-}"
_pre_ROBOT_IP="${ROBOT_IP:-}"

if [ -f "$HERE/onsite.env" ]; then
    source "$HERE/onsite.env"
else
    echo "warning: $HERE/onsite.env not found, using defaults"
fi

[ -n "$_pre_UNITREE_IF" ] && export UNITREE_IF="$_pre_UNITREE_IF"
[ -n "$_pre_ROBOT_IP" ] && export ROBOT_IP="$_pre_ROBOT_IP"
unset _pre_UNITREE_IF _pre_ROBOT_IP

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Built here from UNITREE_IF rather than read from
# config/cyclonedds_unitree_wlan.xml, which names wlp4s0 and peers
# 192.168.68.70 in the file itself. Passing --interface enp3s0 to the bridge
# while that file still bound the wireless card produced the worst failure
# this stack has: the bridge arms, reports itself ready, and not one command
# reaches the legs. The interface belongs in onsite.env with every other
# machine-specific value, not baked into a config file.
#
# The legacy <NetworkInterfaceAddress> element is not a style choice. Foxy
# ships CycloneDDS 0.7, which given the newer <Interfaces>/<NetworkInterface>
# syntax creates no participant at all and reports nothing.
#
# The unicast peer matters on a network that drops or isolates multicast -
# most site APs do. ROBOT_IP has to be the robot's address on the same subnet
# as UNITREE_IF, or discovery has nowhere to go.
export CYCLONEDDS_URI="<CycloneDDS><Domain id=\"any\"><General>
    <NetworkInterfaceAddress>${UNITREE_IF:-eth0}</NetworkInterfaceAddress>
</General><Discovery><Peers>
    <Peer address=\"${ROBOT_IP:-192.168.123.161}\" />
</Peers></Discovery></Domain></CycloneDDS>"

# Unitree's own DDS traffic is on domain 0. Overrides whatever onsite.env set
# for the rest of the stack, on purpose.
export ROS_DOMAIN_ID=0

# Must be unset here: it would cut the bridge off from the robot, which is not
# on this machine.
unset ROS_LOCALHOST_ONLY

IFACE="${UNITREE_IF:-eth0}"

if ! ip -o link show "$IFACE" > /dev/null 2>&1; then
    echo
    echo "WARNING: interface $IFACE does not exist on this machine."
    echo "         CycloneDDS will create no participant and say nothing useful."
    echo "         Cards here: $(ip -brief link show | awk '$1!="lo"{printf "%s ", $1}')"
    echo "         Set UNITREE_IF in scripts/onsite.env to one of them."
fi

echo "Unitree SDK environment ready - THIS TERMINAL TALKS TO THE ROBOT"
echo "  RMW              $RMW_IMPLEMENTATION"
echo "  bound to         ${UNITREE_IF:-eth0}   (UNITREE_IF, override it per terminal)"
echo "  discovery peer   ${ROBOT_IP:-192.168.123.161}"
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
