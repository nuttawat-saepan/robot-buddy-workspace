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

# CycloneDDS binds to one named interface and ignores every other one.
# Unitree's setup.sh names a single wired interface, which is why a wireless
# link between the two machines discovers nothing: ping succeeds, `ros2 topic
# list` is empty, and nothing explains why. ROBOT_NET_IF is the link to the
# ground station, so that is the one named here; the Unitree bridge runs in its
# own terminal under setup_sdk_env.sh with its own interface and its own RMW.
#
# This MUST be the legacy <NetworkInterfaceAddress> element. Foxy ships
# CycloneDDS 0.7, and given the newer <Interfaces>/<NetworkInterface> syntax it
# refuses to create a participant at all:
#
#   config: //CycloneDDS/Domain/General: Interfaces: unknown element
#   rmw_create_node: failed to create domain, error Error
#
# 0.7 takes a single interface here, not a list. Naming two is the same error.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General>
    <NetworkInterfaceAddress>${ROBOT_NET_IF:-wlan0}</NetworkInterfaceAddress>
</General></Domain></CycloneDDS>"

# CycloneDDS fails to create a participant at all if the named interface does
# not exist on this machine, and the traceback that follows blames rclpy rather
# than the name. wlan0 exists on the board and nowhere else, so sourcing this
# on the MiniPC out of habit fails in a way that reads like a broken node.
if ! ip -o link show "${ROBOT_NET_IF:-wlan0}" >/dev/null 2>&1; then
    echo "WARNING: interface ${ROBOT_NET_IF:-wlan0} does not exist on this machine."
    echo "         Every node will fail with 'rmw handle is invalid'."
    echo "         On the MiniPC use go2local; on the ground station go2ground."
    echo
fi

# Every node in this project launches with output='screen', which on Foxy means
# the terminal and nowhere else: launch.log gets three lines about processes
# starting and not one line of what they said. FAST-LIO's "lidar loop back,
# clear buffer" - the warning that it is about to diverge - has never been on
# disk once. This overrides output= for every action without touching the
# launch files, and it is what makes collect_logs.sh worth running.
export OVERRIDE_LAUNCH_PROCESS_OUTPUT=both

echo "robot environment ready"
echo "  RMW              $RMW_IMPLEMENTATION"
echo "  ROS_DOMAIN_ID    ${ROS_DOMAIN_ID:-0}"
echo "  interface        ${ROBOT_NET_IF:-wlan0}   (the bridge names its own, separately)"
echo "  mount            pitch ${LIO_PITCH_DEG:-13.0} roll ${LIO_ROLL_DEG:-0.0} height ${SENSOR_HEIGHT:-0.35}"
echo
echo "  launch:  ros2 launch go2_control livox_robot.launch.py replay:=false \\"
echo "               lio_pitch_deg:=${LIO_PITCH_DEG:-13.0} lio_roll_deg:=${LIO_ROLL_DEG:-0.0} \\"
echo "               sensor_height:=${SENSOR_HEIGHT:-0.35} unitree_interface:=${UNITREE_IF:-eth0}"
