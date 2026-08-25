#!/usr/bin/env bash
# Source this on the ground station before launching anything.
#
#     source scripts/setup_ground_env.sh
#     ros2 launch go2_control livox_ground.launch.py replay:=false enable_rviz:=true

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "$HERE")"

source /opt/ros/foxy/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

if [ -f "$HERE/onsite.env" ]; then
    source "$HERE/onsite.env"
else
    echo "warning: $HERE/onsite.env not found, using defaults"
    echo "         cp $HERE/onsite.env.example $HERE/onsite.env"
fi

# Must match the board. FastRTPS and CycloneDDS cannot talk to each other at
# all, and the symptom is an empty topic list rather than an error.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name=\"${GROUND_NET_IF:-wlp4s0}\" priority=\"default\" multicast=\"default\" />
</Interfaces></General></Domain></CycloneDDS>"

echo "ground station environment ready"
echo "  RMW              $RMW_IMPLEMENTATION"
echo "  ROS_DOMAIN_ID    ${ROS_DOMAIN_ID:-0}"
echo "  interface        ${GROUND_NET_IF:-wlp4s0}"
echo "  map              ${SITE_MAP:-<unset, launch default will be used>}"
echo
echo "  launch:  ros2 launch go2_control livox_ground.launch.py replay:=false \\"
echo "               map:=${SITE_MAP:-livox_slam_02loop.yaml} enable_rviz:=true"
