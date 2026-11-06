#!/usr/bin/env bash
# Drive the simulated robot a little, the way the remote does on site.
#
#     scripts/sim_nudge.sh             0.2 m/s and a slight turn for 3 s
#
# AMCL only updates when the robot moves, so after setting the pose on the web
# the spread stays above the limit and a mission is refused until it has moved.
# Goes in ahead of the collision guard, on the simulation's own ROS domain.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/src/go2_control/config/fastdds_udp_only.xml"
export ROS_DOMAIN_ID=77

timeout -s INT 3 ros2 topic pub -r 10 /cmd_vel_safe geometry_msgs/msg/Twist \
    '{linear: {x: 0.2}, angular: {z: 0.15}}' >/dev/null
ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist '{}' >/dev/null
echo "nudged"
