#!/usr/bin/env bash
# The web page driving a simulated Go2W, with the Gazebo window and RViz open.
#
#     scripts/start_sim_web.sh                 the obstacle test room
#     scripts/start_sim_web.sh meeting_room    the meeting room map of 2026-11-04
#     scripts/start_sim_web.sh meeting_room slam   make a map instead: open /slam
#
# Ctrl-C stops all of it. Set the pose on the web page, as on site: the robot
# starts at x 2, y 0 in the test room and x -0.3, y -1.0 in the meeting room, facing
# +x. Drop boxes in its way from the Gazebo window.
#
# Own ROS domain and own Gazebo port. Nothing here publishes /cmd_vel, and the
# relay and the Unitree bridge are not started. The website still talks to the
# broker on this machine, so do not run this while a real robot is connected to
# that broker: both would take the same mission.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/foxy/setup.bash
source "$WS/install/setup.bash"
source /usr/share/gazebo/setup.sh 2>/dev/null
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/src/go2_control/config/fastdds_udp_only.xml"
export ROS_DOMAIN_ID=77
export GAZEBO_MASTER_URI=http://127.0.0.1:11377
export LIO_SENSOR_X=0.30

echo "simulation, not the robot: ROS domain $ROS_DOMAIN_ID, no /cmd_vel, no bridge"
echo "drive by hand: http://127.0.0.1:8077"
ROOM="${1:-obstacle_room}"
SLAM=false
[ "$2" = slam ] && SLAM=true && echo "map page: http://127.0.0.1:5173/slam"
START_X=2.0 START_Y=0.0
# The only spot near the first waypoint with 0.4 m clear all round.
[ "$ROOM" = meeting_room ] && START_X=-0.3 START_Y=-1.0
exec ros2 launch go2_control sim_obstacle.launch.py web:=true gui:=true \
    room:="$ROOM" start_x:="$START_X" start_y:="$START_Y" slam:="$SLAM"
