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

# Every node in this project launches with output='screen', which on Foxy means
# the terminal and nowhere else: launch.log gets three lines about processes
# starting and not one line of what they said. FAST-LIO's "lidar loop back,
# clear buffer" - the warning that it is about to diverge - has never been on
# disk once. This overrides output= for every action without touching the
# launch files, and it is what makes collect_logs.sh worth running.
export OVERRIDE_LAUNCH_PROCESS_OUTPUT=both

echo "ground station environment ready"
echo "  RMW              $RMW_IMPLEMENTATION"
echo "  ROS_DOMAIN_ID    ${ROS_DOMAIN_ID:-0}"
echo "  interface        ${GROUND_NET_IF:-wlp4s0}"
echo "  map              ${SITE_MAP:-<unset, launch default will be used>}"
echo "  nav2 params      ${NAV2_PARAMS:-nav2_livox_go2.yaml}"
echo "  amcl params      ${AMCL_PARAMS:-amcl_livox.yaml}"
echo
echo "  launch:  ros2 launch go2_control livox_ground.launch.py replay:=false \\"
echo "               map:=${SITE_MAP:-livox_slam_02loop.yaml} \\"
echo "               nav2_params_file:=${NAV2_PARAMS:-nav2_livox_go2.yaml} \\"
echo "               amcl_params_file:=${AMCL_PARAMS:-amcl_livox.yaml} enable_rviz:=true"
