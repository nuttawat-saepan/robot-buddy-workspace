# Short names for the commands used most often on site, where one hand is on
# the remote and nobody wants to type a path.
#
#     echo 'source ~/projects/systonic-2307/Systronic/scripts/aliases.sh' >> ~/.bashrc
#
# Deliberately no `export RMW_IMPLEMENTATION` here. This project needs Fast DDS
# for the navigation graph and CycloneDDS for the Unitree bridge, so a global
# default would silently be wrong in half the terminals - and the symptom is an
# empty topic list, or a publisher that delivers nothing, with no error to
# explain either. Every alias below that matters says which side it put you on.

_GO2_WS="$HOME/projects/systonic-2307/Systronic"

# --- the five environments, each prints what it set -------------------------
alias go2local='source $_GO2_WS/scripts/setup_local_env.sh'    # one machine, replay
alias go2robot='source $_GO2_WS/scripts/setup_robot_env.sh'    # the board
alias go2ground='source $_GO2_WS/scripts/setup_ground_env.sh'  # the ground station
alias go2sdk='source $_GO2_WS/scripts/setup_sdk_env.sh'        # the Unitree bridge
alias go2sim='source $_GO2_WS/scripts/setup_sim_env.sh'        # Gazebo, a different robot

# --- where am I ------------------------------------------------------------
# The question that costs the most time on site is not "did I source it" but
# "which side did I source", so make it one word.
alias go2which='echo "RMW=${RMW_IMPLEMENTATION:-<default fastrtps>}  DOMAIN=${ROS_DOMAIN_ID:-0}  LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}  UNITREE_IF=${UNITREE_IF:-<unset>}"'

# --- is it ready -----------------------------------------------------------
alias go2ready='ros2 run go2_control nav_ready_check --ros-args -p controller_cmd_topic:=${GO2_CONTROLLER_TOPIC:-/cmd_vel_nav_preview} -p base_frame:=${GO2_BASE_FRAME:-base_link}'
alias go2tf='ros2 run tf2_ros tf2_echo map odom'
alias go2tf2='ros2 run tf2_ros tf2_echo odom ${GO2_BASE_FRAME:-base_link}'
alias go2life='for n in /map_server /amcl /planner_server /controller_server /bt_navigator; do printf "%-22s " $n; ros2 lifecycle get $n 2>&1 | head -1; done'
alias go2hz='for t in /livox/lidar /scan /Odometry /map; do printf "%-16s " $t; timeout 4 ros2 topic hz $t 2>&1 | grep -m1 "average rate" || echo "silent"; done'

# --- the safety check, run it before every goal ----------------------------
# /cmd_vel having no publisher is the standing proof that a run cannot move the
# robot. Check it, do not assume it.
alias go2safe='echo -n "/cmd_vel: "; ros2 topic info /cmd_vel 2>&1 | head -1; echo -n "cmd_vel nodes: "; ros2 node list 2>/dev/null | grep -c cmd_vel'

# --- sending a goal --------------------------------------------------------
alias go2goal='ros2 run go2_control send_mission --controller-topic ${GO2_CONTROLLER_TOPIC:-/cmd_vel_nav_preview} --goal'
alias go2mission='ros2 run go2_control send_mission --controller-topic ${GO2_CONTROLLER_TOPIC:-/cmd_vel_nav_preview} --file'
alias go2record='ros2 run go2_control record_waypoint --base-frame ${GO2_BASE_FRAME:-base_link} --file'

# --- when it goes wrong ----------------------------------------------------
alias go2logs='$_GO2_WS/scripts/collect_logs.sh'
alias go2cpu='$_GO2_WS/scripts/measure_board_load.sh'
alias go2build='(cd $_GO2_WS && source /opt/ros/foxy/setup.bash && colcon build --packages-select go2_control)'

# Only safe with every ROS process stopped: removing these while a participant
# is alive breaks its transport, and the symptom is a topic that has a
# publisher and delivers nothing.
alias go2shm='pgrep -f "ros2|_node|fastlio|amcl" >/dev/null && echo "ROS is still running - stop everything first" || (rm -f /dev/shm/fastrtps_* && echo "cleared stale Fast DDS segments")'
