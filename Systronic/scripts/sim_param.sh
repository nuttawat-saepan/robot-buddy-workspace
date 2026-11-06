#!/usr/bin/env bash
# Read or change a parameter of the running simulation, without the web page.
#
#     scripts/sim_param.sh get /amcl alpha1
#     scripts/sim_param.sh set /amcl alpha1 0.05
#     scripts/sim_param.sh set /controller_server goal_checker.xy_goal_tolerance 0.15
#
# set goes through go2_control reconfigure: the node is taken through its
# lifecycle so it really uses the new value, since AMCL and the goal checker
# read theirs only at configure. Robot stopped, no mission running. A value set
# this way lasts until the simulation is restarted.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/foxy/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/src/go2_control/config/fastdds_udp_only.xml"
export ROS_DOMAIN_ID=77

case "$1" in
    get) ros2 param get "$2" "$3" ;;
    set) restore=""
         # Only AMCL forgets the pose; putting it back for another node would
         # reset AMCL for nothing.
         [ "$2" != /amcl ] && restore="--no-restore-pose"
         ros2 run go2_control reconfigure --node "$2" --set "$3=$4" $restore \
             && ros2 param get "$2" "$3" ;;
    *)   sed -n 2,11p "$0"; exit 1 ;;
esac
