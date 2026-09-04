#!/usr/bin/env bash
# Source this for the Gazebo simulation, and nothing else.
#
#     source scripts/setup_sim_env.sh        (or: go2sim)
#     ros2 launch go2_control sim.launch.py
#
# The simulation is a different robot on a different map with different frame
# names, so the values that have to change between it and the field are
# collected here rather than typed on each command line. What the simulation is
# for, and what it cannot tell you, is in SIM_VS_FIELD.md.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "$HERE")"

source /opt/ros/foxy/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

# The simulated robot is a TurtleBot3 Waffle, not a Go2W.
export TURTLEBOT3_MODEL=waffle

# Same DDS arrangement as any other single-machine work.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/src/go2_control/config/fastdds_udp_only.xml"
export ROS_DOMAIN_ID=0

# Not set: Gazebo's own transport does not go through ROS, and turning this on
# has been reported to interfere with gzserver's discovery on some setups. The
# simulation is local anyway.
unset ROS_LOCALHOST_ONLY

# A stock Nav2 bringup leaves the controller on /cmd_vel. The Livox launch
# files remap it to /cmd_vel_nav_preview so that a planning run has no path to
# the robot at all - which means every readiness check has to be told which of
# the two to look at.
export GO2_CONTROLLER_TOPIC=/cmd_vel

# TurtleBot3 uses base_footprint where the Go2 stack uses base_link.
export GO2_BASE_FRAME=base_footprint

# Every node in this project launches with output='screen', which on Foxy means
# the terminal and nowhere else: launch.log gets three lines about processes
# starting and not one line of what they said. FAST-LIO's "lidar loop back,
# clear buffer" - the warning that it is about to diverge - has never been on
# disk once. This overrides output= for every action without touching the
# launch files, and it is what makes collect_logs.sh worth running.
export OVERRIDE_LAUNCH_PROCESS_OUTPUT=both

echo "simulation environment ready - TurtleBot3 in Gazebo, not the Go2W"
echo "  TURTLEBOT3_MODEL   $TURTLEBOT3_MODEL"
echo "  RMW                $RMW_IMPLEMENTATION"
echo "  ROS_DOMAIN_ID      $ROS_DOMAIN_ID"
echo "  controller topic   $GO2_CONTROLLER_TOPIC   (field: /cmd_vel_nav_preview)"
echo "  base frame         $GO2_BASE_FRAME   (field: base_link)"
echo
echo "  ros2 launch go2_control sim.launch.py"
echo
echo "  AMCL needs an initial pose and then some motion - it does not update"
echo "  below update_min_d, so a stationary robot never converges:"
echo "    ros2 topic pub --once /initialpose \\"
echo "      geometry_msgs/msg/PoseWithCovarianceStamped \\"
echo "      \"{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5},\\"
echo "        orientation: {w: 1.0}}}}\""
echo
echo "  then check, and send a goal:"
echo "    ros2 run go2_control nav_ready_check --ros-args \\"
echo "        -p controller_cmd_topic:=$GO2_CONTROLLER_TOPIC -p base_frame:=$GO2_BASE_FRAME"
echo "    ros2 run go2_control send_mission --goal 1.0 0.0 0.0 \\"
echo "        --controller-topic $GO2_CONTROLLER_TOPIC"
