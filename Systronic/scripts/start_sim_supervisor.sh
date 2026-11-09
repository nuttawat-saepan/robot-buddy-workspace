#!/usr/bin/env bash
# Show the simulation's settings on the web's Settings page.
#
#     systemd-run --user --unit go2-sim-supervisor scripts/start_sim_supervisor.sh
#
# Read-only: live values can be applied, but it never restarts a stack or arms
# the bridge, because the stack it would start is the robot's and not the sim's.
# The backend finds it through ROBOT_SUPERVISOR_URL=http://127.0.0.1:8091.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/foxy/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/src/go2_control/config/fastdds_udp_only.xml"
export ROS_DOMAIN_ID=77
export MQTT_BROKER=127.0.0.1
export STACK_SUPERVISOR_TOKEN="$(sed -n 's/^ROBOT_SUPERVISOR_TOKEN=//p' "$HOME/robot-app/config/backend.env")"
exec ros2 run go2_control stack_supervisor --read-only \
    --state-dir "$HOME/stack_settings_sim" --bag-dir ""
