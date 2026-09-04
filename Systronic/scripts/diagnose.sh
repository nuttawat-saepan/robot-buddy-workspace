#!/usr/bin/env bash
# Work out what is broken when you do not yet know what is broken.
#
#     ./scripts/diagnose.sh            (or: go2doctor)
#
# Read-only. Runs read-only ros2 commands, starts no node, publishes nothing,
# and cannot move the robot.
#
# The chain in this project has ten links and a break anywhere in it surfaces
# the same way further down: a topic that is silent, or a topic that has a
# publisher and delivers nothing. Both look identical whatever the cause. This
# walks the chain from the bottom and stops at the FIRST break, because
# everything after a break is expected to be broken and reading those failures
# as separate problems is how an afternoon disappears.
#
# It answers "what is wrong". collect_logs.sh answers "keep the evidence".
# Run this first; run that one before the terminals are closed.

set -uo pipefail

FIRST_BREAK=""
note() { printf '  %s\n' "$*"; }
ok()   { printf '  \033[32mok\033[0m    %s\n' "$1"; }
bad()  { printf '  \033[31mBROKEN\033[0m %s\n' "$1"
         [ -z "$FIRST_BREAK" ] && FIRST_BREAK="$1"; }
warn() { printf '  \033[33mwarn\033[0m  %s\n' "$1"; }
hdr()  { printf '\n== %s\n' "$1"; }

# A topic can be silent for two quite different reasons and only one of them is
# visible in the topic list, so both are always checked together.
pub_count() { timeout 6 ros2 topic info "$1" 2>/dev/null | awk '/Publisher count/{print $NF}'; }
has_data()  { timeout 6 ros2 topic hz "$1" 2>/dev/null | grep -qm1 'average rate'; }

check_topic() {
    local topic="$1" why="$2" pubs
    pubs="$(pub_count "$topic")"
    if [ -z "$pubs" ]; then
        bad "$topic does not exist - $why"
    elif [ "$pubs" = "0" ]; then
        bad "$topic exists with no publisher - $why"
    elif has_data "$topic"; then
        ok "$topic  ($pubs publisher/s, delivering)"
    else
        bad "$topic has $pubs publisher/s but delivers nothing"
        note "this is the DDS signature, not a dead node. Usually a stale"
        note "shared-memory segment, or two sides on different domains or RMWs."
    fi
}

echo "diagnose - read-only, sends nothing"

# --------------------------------------------------------------- the shell
hdr "which side is this terminal on"
if ! command -v ros2 > /dev/null 2>&1; then
    bad "ros2 is not on PATH - nothing below can work"
    note "source one of: go2local, go2robot, go2ground, go2sdk, go2sim"
    echo; echo "first break: $FIRST_BREAK"; exit 1
fi
note "RMW            ${RMW_IMPLEMENTATION:-<default fastrtps>}"
note "ROS_DOMAIN_ID  ${ROS_DOMAIN_ID:-0}"
note "LOCALHOST_ONLY ${ROS_LOCALHOST_ONLY:-0}"
if [ -z "${OVERRIDE_LAUNCH_PROCESS_OUTPUT:-}" ]; then
    warn "OVERRIDE_LAUNCH_PROCESS_OUTPUT is unset - launch logs will hold"
    note "only 'process started' and nothing any node said. Source a setup"
    note "script rather than install/setup.bash alone."
fi
for var_if in "${UNITREE_IF:-}" "${ROBOT_NET_IF:-}"; do
    [ -n "$var_if" ] || continue
    if ip -o link show "$var_if" > /dev/null 2>&1; then
        ok "interface $var_if exists here"
    else
        bad "interface $var_if does not exist on this machine"
        note "CycloneDDS creates no participant at all and says nothing useful."
    fi
done

# -------------------------------------------------------------- the machine
hdr "can this machine keep up"
LOAD1="$(awk '{print $1}' /proc/loadavg)"
CORES="$(nproc)"
if awk -v l="$LOAD1" -v c="$CORES" 'BEGIN{exit !(l > c*0.9)}'; then
    bad "load average $LOAD1 on $CORES cores"
    note "FAST-LIO does not slow down when starved, it diverges: the pose runs"
    note "away to tens of thousands of metres while every process stays alive"
    note "and every topic keeps publishing. Check /Odometry before trusting it."
else
    ok "load average $LOAD1 on $CORES cores"
fi

# --------------------------------------------------------- duplicate nodes
hdr "is anything running twice"
# The ros2 CLI's own short-lived nodes duplicate each other constantly and
# mean nothing - including the tf2_echo calls this script makes further down.
DUPES="$(ros2 node list 2>/dev/null | grep -vE '^/(tf2_echo|_ros2cli|ros2cli)' \
    | sort | uniq -d)"
if [ -n "$DUPES" ]; then
    bad "duplicate nodes: $(echo "$DUPES" | tr '\n' ' ')"
    note "a terminal from an earlier attempt is still alive. Two publishers on"
    note "one TF link fight each other and the transform jitters."
else
    ok "no duplicate node names"
fi
STATIC_TF="$(pgrep -f static_transform_publisher 2>/dev/null | wc -l)"
[ "$STATIC_TF" -gt 4 ] && warn "$STATIC_TF static_transform_publisher processes - leftovers from earlier runs"

# ------------------------------------------------------------- the chain
hdr "the chain, in order - the first break is the one to fix"
check_topic /livox/lidar "the Livox driver is not running, or replay:=false was omitted"
check_topic /Odometry    "FAST-LIO is not running"
check_topic /scan        "/scan is projected from FAST-LIO's cloud, not from raw lidar"

hdr "the two transforms Nav2 needs"
if timeout 6 ros2 run tf2_ros tf2_echo odom "${GO2_BASE_FRAME:-base_link}" 2>&1 | grep -q Translation; then
    ok "odom -> ${GO2_BASE_FRAME:-base_link}  (lio_odom_relay)"
else
    bad "odom -> ${GO2_BASE_FRAME:-base_link} missing - lio_odom_relay is not publishing"
fi
if timeout 6 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q Translation; then
    ok "map -> odom  (amcl)"
else
    bad "map -> odom missing - amcl is not running, or died at startup"
    note "amcl segfaults with no message if robot_model_type uses the Galactic"
    note "spelling. Foxy wants \"differential\"."
fi

# ---------------------------------------------------------- managed nodes
hdr "lifecycle nodes - inactive is as silent as absent"
for n in /map_server /amcl /planner_server /controller_server /bt_navigator; do
    state="$(timeout 6 ros2 lifecycle get "$n" 2>/dev/null | head -1)"
    case "$state" in
        *active*)  ok  "$(printf '%-20s %s' "$n" "$state")" ;;
        '')        bad "$n is not there at all" ;;
        *)         bad "$(printf '%-20s %s' "$n" "$state")"
                   note "the lifecycle manager did not bring it up. Its reason is in the launch log." ;;
    esac
done

# ------------------------------------------------------------- the verdict
hdr "verdict"
if [ -z "$FIRST_BREAK" ]; then
    ok "nothing broken found in the chain"
    note "if a goal still does nothing, the break is past ROS: the relay, the"
    note "UDP hop, or the Unitree bridge. Run the bridge in --mode probe."
else
    printf '  first break: \033[31m%s\033[0m\n' "$FIRST_BREAK"
    note ""
    note "fix this one before reading anything below it - every later failure"
    note "is expected while this is broken."
fi
note ""
note "keep the evidence before closing the terminals:"
note "  ./scripts/collect_logs.sh \"what you had just done\""
