#!/usr/bin/env bash
# Collect everything needed to work out what went wrong, into one archive.
#
#     ./scripts/collect_logs.sh [note]
#
# Read-only. It copies files and runs read-only ros2 commands. It starts no
# node, publishes nothing, and cannot move the robot.
#
# Run it while the stack is still up - most of what matters here disappears the
# moment the terminals are closed. Two field sessions ended without an
# explanation because the evidence was in scrollback that is now gone.
#
# The optional note goes into the archive as a line of text. Write what you had
# just done: "goal sent, blue line, robot did not move" is worth more later
# than any log file.

set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NOTE="${1:-}"
STAMP="$(date +%Y%m%d_%H%M%S)"
DEST="${DEST:-$HOME/go2_logs/$STAMP}"
mkdir -p "$DEST"

say() { echo "  $*"; }
grab() { timeout "${2:-15}" bash -c "$1" > "$DEST/$3" 2>&1 || echo "(timed out or failed)" >> "$DEST/$3"; }

echo "collecting into $DEST"

# ---------------------------------------------------------------- the note
{
    echo "collected   $(date -Is)"
    echo "host        $(hostname) ($(uname -m))"
    echo "note        ${NOTE:-<none given>}"
} > "$DEST/00_note.txt"
say "note and host"

# --------------------------------------------------------- the environment
# Half of the failures in this project come down to two terminals disagreeing
# about ROS_DOMAIN_ID or RMW_IMPLEMENTATION, which nothing reports as an error.
{
    echo "== this shell =="
    for v in ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY \
             CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE ROS_DISTRO; do
        echo "$v=${!v:-<unset>}"
    done
    echo
    echo "== per running ROS process =="
    for pid in $(pgrep -f 'ros2|_node|fastlio|amcl|map_server|planner_server|controller_server|rviz2' 2>/dev/null); do
        comm=$(ps -o comm= -p "$pid" 2>/dev/null) || continue
        dom=$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep '^ROS_DOMAIN_ID=' || echo 'ROS_DOMAIN_ID=<unset,0>')
        rmw=$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep '^RMW_IMPLEMENTATION=' || echo 'RMW=<default>')
        printf '%-8s %-24s %-24s %s\n' "$pid" "$comm" "$dom" "$rmw"
    done
} > "$DEST/01_env.txt" 2>&1
say "environment, including per-process domain and RMW"

# ------------------------------------------------------------- the machine
{
    uptime
    echo
    free -m
    echo
    df -h "$HOME" /tmp
    echo
    echo "== top by cpu =="
    ps -eo pid,etimes,pcpu,pmem,comm --sort=-pcpu | head -25
    echo
    echo "== network interfaces =="
    echo "UNITREE_IF and ROBOT_NET_IF have to name one of these. Naming a card"
    echo "that does not exist here is silent in some places and fatal in others."
    ip -brief addr 2>/dev/null || ip addr 2>/dev/null
    echo
    echo "== shared memory segments =="
    echo "a large count here with no ROS running means stale Fast DDS segments;"
    echo "they can only be cleared once every ROS process is stopped."
    ls /dev/shm 2>/dev/null | wc -l
} > "$DEST/02_machine.txt" 2>&1
say "machine load, memory, disk"

# --------------------------------------------------------------- ros graph
if command -v ros2 > /dev/null 2>&1; then
    grab "ros2 node list" 20 "10_nodes.txt"
    grab "ros2 topic list -t" 20 "11_topics.txt"
    say "node and topic lists"

    # A topic with a publisher but no data is the signature of broken shared
    # memory, and it is invisible in the topic list alone.
    : > "$DEST/12_rates.txt"
    for t in /livox/lidar /livox/imu /Odometry /scan /clock /map \
             /amcl_pose /particlecloud /cmd_vel_safe /cmd_vel; do
        {
            echo "=== $t"
            timeout 6 ros2 topic info "$t" 2>&1 | head -4
            timeout 6 ros2 topic hz "$t" 2>&1 | grep -m1 'average rate' || echo 'no data received'
            echo
        } >> "$DEST/12_rates.txt"
    done
    say "publisher counts and actual rates for the topics that matter"

    grab "ros2 run tf2_ros tf2_echo map odom" 8 "13_tf_map_odom.txt"
    grab "ros2 run tf2_ros tf2_echo odom base_link" 8 "14_tf_odom_base.txt"
    say "the two transforms Nav2 needs"

    for n in /amcl /map_server /planner_server /controller_server /bt_navigator; do
        {
            echo "=== $n"
            timeout 6 ros2 lifecycle get "$n" 2>&1
        } >> "$DEST/15_lifecycle.txt"
    done
    say "lifecycle state of the managed nodes"

    grab "timeout 12 ros2 run go2_control nav_ready_check" 20 "16_nav_ready.txt"
    say "nav_ready_check verdict"
else
    echo "ros2 not on PATH - source the workspace before running this" \
        > "$DEST/10_nodes.txt"
    say "ros2 not found, skipping graph capture"
fi

# ----------------------------------------------------- the Unitree side
# The rest of this script sees whatever graph the current shell is pointed at.
# The robot's own traffic is not in it: Unitree publishes on ROS_DOMAIN_ID 0
# over CycloneDDS, while the navigation stack is on the site domain over Fast
# DDS. Collecting only one of the two is how a trip ends with no record of the
# single question it went out to answer - what the robot actually publishes.
if command -v ros2 > /dev/null 2>&1; then
    UNITREE_XML="$WS/src/go2_control/config/cyclonedds_unitree_wlan.xml"
    {
        echo "queried with ROS_DOMAIN_ID=0, rmw_cyclonedds_cpp"
        echo "config: $UNITREE_XML"
        echo
        echo "== everything on domain 0 =="
        env -u ROS_LOCALHOST_ONLY -u FASTRTPS_DEFAULT_PROFILES_FILE \
            ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
            CYCLONEDDS_URI="file://$UNITREE_XML" \
            timeout 20 ros2 topic list 2>&1
        echo
        echo "== the request topic, if it is there under any name =="
        env -u ROS_LOCALHOST_ONLY -u FASTRTPS_DEFAULT_PROFILES_FILE \
            ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
            CYCLONEDDS_URI="file://$UNITREE_XML" \
            timeout 20 ros2 topic list 2>/dev/null \
            | grep -E 'api|sport|lf/|wirelesscontroller' || echo '(nothing matched)'
    } > "$DEST/17_unitree_domain0.txt" 2>&1
    say "the Unitree graph on domain 0 - the answer to 'what is the topic called'"
fi

# --------------------------------------------------- the bridge transcripts
# unitree_udp_bridge prints to the terminal and writes its own transcript;
# nothing of it reaches ~/.ros/log, because it does not use the ROS logger.
if compgen -G "$HOME/go2_logs/bridge_*.log" > /dev/null 2>&1; then
    mkdir -p "$DEST/bridge"
    find "$HOME/go2_logs" -maxdepth 1 -name 'bridge_*.log' -mmin -480 \
        -exec cp {} "$DEST/bridge/" \; 2>/dev/null
    say "unitree_udp_bridge transcripts from the last eight hours"
fi

# ----------------------------------------------------------- the log files
# ~/.ros/log grows without limit, so take only what is recent enough to be
# about this session.
if [ -d "$HOME/.ros/log" ]; then
    mkdir -p "$DEST/ros_log"
    find "$HOME/.ros/log" -maxdepth 1 -mmin -240 -type d -print0 2>/dev/null \
        | xargs -0 -I{} cp -r {} "$DEST/ros_log/" 2>/dev/null
    say "~/.ros/log from the last four hours"
fi

# ---------------------------------------------------------- the config used
mkdir -p "$DEST/config"
cp "$WS"/src/go2_control/config/*.yaml "$DEST/config/" 2>/dev/null
cp "$WS"/src/go2_control/config/*.xml "$DEST/config/" 2>/dev/null
cp "$WS"/src/go2_control/config/*.json "$DEST/config/" 2>/dev/null
# onsite.env is gitignored, so without it there is no record of which
# interface, map and profile this run actually used.
cp "$WS/scripts/onsite.env" "$DEST/config/" 2>/dev/null
say "the config files as they were for this run"

{
    cd "$WS" || exit 0
    echo "== commit =="
    git log --oneline -5 2>/dev/null
    echo
    echo "== uncommitted =="
    git status --porcelain 2>/dev/null
    echo
    echo "== diff =="
    git diff 2>/dev/null
} > "$DEST/03_git.txt" 2>&1
say "which commit this was, and what was edited on top of it"

# --------------------------------------------------------------- archive it
TAR="$DEST.tar.gz"
tar -czf "$TAR" -C "$(dirname "$DEST")" "$(basename "$DEST")" 2>/dev/null

echo
echo "done"
echo "  folder   $DEST"
echo "  archive  $TAR  ($(du -h "$TAR" 2>/dev/null | cut -f1))"
echo
echo "copy it off the board before the day ends:"
echo "  scp unitree@\${ROBOT_IP:-192.168.123.161}:$TAR ."
