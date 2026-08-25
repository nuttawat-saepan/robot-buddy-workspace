#!/usr/bin/env bash
# Source this on the Unitree board before launching anything.
#
#     source scripts/setup_robot_env.sh
#     ros2 launch go2_control livox_robot.launch.py replay:=false
#
# Sourcing order matters: ROS first, then each overlay, ours last.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "$HERE")"

source /opt/ros/foxy/setup.bash
[ -f "$HOME/ws_livox/install/setup.bash" ] && source "$HOME/ws_livox/install/setup.bash"
[ -f "$HOME/ws_fastlio_livox/install/setup.bash" ] && source "$HOME/ws_fastlio_livox/install/setup.bash"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

if [ -f "$HERE/onsite.env" ]; then
    source "$HERE/onsite.env"
else
    echo "warning: $HERE/onsite.env not found, using defaults"
    echo "         cp $HERE/onsite.env.example $HERE/onsite.env"
fi

# CycloneDDS binds to named interfaces and ignores every other one. Unitree's
# setup.sh names a single wired interface, which is why a wireless link between
# the two machines discovers nothing: ping succeeds, `ros2 topic list` is
# empty, and nothing explains why. Both interfaces are listed here so the
# robot's internal network and the link to the ground station both work.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name=\"${ROBOT_NET_IF:-wlan0}\" priority=\"default\" multicast=\"default\" />
    <NetworkInterface name=\"${UNITREE_IF:-eth0}\" priority=\"default\" multicast=\"default\" />
</Interfaces></General></Domain></CycloneDDS>"

echo "robot environment ready"
echo "  RMW              $RMW_IMPLEMENTATION"
echo "  ROS_DOMAIN_ID    ${ROS_DOMAIN_ID:-0}"
echo "  interfaces       ${ROBOT_NET_IF:-wlan0}, ${UNITREE_IF:-eth0}"
echo "  mount            pitch ${LIO_PITCH_DEG:-13.0} roll ${LIO_ROLL_DEG:-0.0} height ${SENSOR_HEIGHT:-0.35}"
echo
echo "  launch:  ros2 launch go2_control livox_robot.launch.py replay:=false \\"
echo "               lio_pitch_deg:=${LIO_PITCH_DEG:-13.0} lio_roll_deg:=${LIO_ROLL_DEG:-0.0} \\"
echo "               sensor_height:=${SENSOR_HEIGHT:-0.35} unitree_interface:=${UNITREE_IF:-eth0}"
