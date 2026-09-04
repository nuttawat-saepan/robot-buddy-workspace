#!/usr/bin/env bash
# Put this workspace on the Unitree board and build what has to be built there.
#
#     ./scripts/deploy_to_board.sh unitree@192.168.123.161              # sync + build
#     ./scripts/deploy_to_board.sh unitree@192.168.123.161 --sync-only  # sync only
#     ./scripts/deploy_to_board.sh unitree@192.168.123.161 --dry-run    # show, do nothing
#
# Deploying does not move the robot: it copies files and runs colcon. Nothing
# here starts a node, and `enable_cmd_vel` stays false wherever it appears.
#
# ## What goes, and what does not
#
# Only go2_control and go2_interfaces are synced. Everything else the board
# needs is either built from upstream sources on the board itself, following
# the patch READMEs, or installed from apt - none of it belongs in an rsync
# from a developer machine.
#
# ## The three things that must already exist on the board
#
# This script does not install them, because each one wants a human reading the
# output the first time:
#
#     ~/ws_livox               Livox-SDK2 and livox_ros_driver2, ./build.sh ROS2
#     ~/ws_fastlio_livox       FAST_LIO with src/go2_control/patches/fast_lio/
#     ros-foxy-navigation2     apt, arm64 binaries exist for Focal
#     ros-foxy-pointcloud-to-laserscan
#
# See patches/fast_lio/README.md for the FAST-LIO sequence, including the
# ikd-Tree submodule that a --depth 1 clone silently omits.

set -euo pipefail

TARGET="${1:-}"
MODE="${2:-}"
REMOTE_WS="${REMOTE_WS:-~/go2_ws}"

if [ -z "$TARGET" ]; then
    cat <<'USAGE'
usage: deploy_to_board.sh <user@host> [--sync-only|--dry-run]

  REMOTE_WS=~/other_ws ./scripts/deploy_to_board.sh unitree@192.168.123.161
USAGE
    exit 2
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "$HERE")"
cd "$WS"

RSYNC_FLAGS=(-az --delete
    --exclude 'build/' --exclude 'install/' --exclude 'log/'
    --exclude '__pycache__/' --exclude '*.pyc'
    --exclude '.git/')
[ "$MODE" = "--dry-run" ] && RSYNC_FLAGS+=(--dry-run --itemize-changes)

echo "== syncing to $TARGET:$REMOTE_WS/src"
ssh "$TARGET" "mkdir -p $REMOTE_WS/src"
for pkg in go2_control go2_interfaces; do
    echo "  $pkg"
    rsync "${RSYNC_FLAGS[@]}" "src/$pkg/" "$TARGET:$REMOTE_WS/src/$pkg/"
done

# The scripts travel too. Everything the runbook tells you to do on the board -
# source setup_robot_env.sh, check the link, measure the CPU, collect the logs
# afterwards - is a file in here, and for a long time none of them were on the
# board at all. onsite.env is the one exclusion: UNITREE_IF names a different
# card on each machine, so the board keeps its own copy. rsync's --delete does
# not remove excluded files on the receiver, so the board's survives this.
echo "  scripts"
rsync "${RSYNC_FLAGS[@]}" --exclude 'onsite.env' \
    scripts/ "$TARGET:$REMOTE_WS/scripts/"

# The maps are the largest thing that legitimately travels, and only the site
# map is wanted on the robot - but the robot side does not load a map at all,
# map_server runs on the ground station. Kept out entirely; if that changes,
# sync one file rather than the directory.

if [ "$MODE" = "--dry-run" ]; then
    echo
    echo "dry run: nothing was written and nothing was built"
    exit 0
fi

if [ "$MODE" = "--sync-only" ]; then
    echo
    echo "synced. Build on the board with:"
    echo "  ssh $TARGET 'cd $REMOTE_WS && colcon build --symlink-install \\"
    echo "      --packages-select go2_interfaces go2_control'"
    exit 0
fi

echo
echo "== building on the board"
# --symlink-install matters more than it looks. go2_control is pure Python, so
# with symlinks an edit to a node, a launch file or a YAML takes effect on the
# next node restart with no rebuild at all. Without it every one-line change
# costs a colcon run over ssh.
# --parallel-workers 2 is not tuning, it is a memory ceiling. Each compiler
# process peaked at 2825 MB on the MiniPC and the board has four cores and
# about 13 GB free, so letting colcon use all four risks the OOM killer taking
# whichever process it likes - which on a board also running the leg
# controller is not a compile failure you want to debug.
if ssh "$TARGET" "bash -lc '
    set -e
    source /opt/ros/foxy/setup.bash
    cd $REMOTE_WS
    colcon build --symlink-install --parallel-workers 2 \
        --packages-select go2_interfaces go2_control
'"; then
    echo "  build ok"
else
    # colcon prints a summary and buries the actual compiler error in a file on
    # the board. Fetch it rather than making someone ssh in and go looking:
    # a failed build over ssh otherwise shows a package name and nothing else.
    echo
    echo "== build FAILED - the errors, from the board's own build log"
    ssh "$TARGET" "bash -lc '
        cd $REMOTE_WS/log/latest_build 2>/dev/null || exit 0
        for pkg in */; do
            for f in \"\$pkg\"stderr.log \"\$pkg\"stdout_stderr.log; do
                [ -s \"\$f\" ] || continue
                echo
                echo \"--- \$f\"
                tail -40 \"\$f\"
            done
        done
    '" || true
    echo
    echo "the whole build log stays on the board at $REMOTE_WS/log/latest_build/"
    echo "bring it home with:"
    echo "  rsync -az $TARGET:$REMOTE_WS/log/latest_build/ ./board_build_log/"
    exit 1
fi

echo
echo "== checking what the board is missing"
ssh "$TARGET" "bash -lc '
    source /opt/ros/foxy/setup.bash
    for p in livox_ros_driver2 fast_lio_livox pointcloud_to_laserscan nav2_amcl; do
        if ros2 pkg prefix \$p >/dev/null 2>&1; then
            echo \"  ok       \$p\"
        else
            echo \"  MISSING  \$p\"
        fi
    done
'" || true

cat <<EOM

== next on the board

  source $REMOTE_WS/scripts/setup_robot_env.sh
  ros2 launch go2_control livox_robot.launch.py replay:=false

onsite.env is not synced - UNITREE_IF names a different card on each machine.
Fill it in once on the board:

  ssh $TARGET 'cd $REMOTE_WS/scripts && cp -n onsite.env.example onsite.env && nano onsite.env'

When something goes wrong, run this while the stack is still up. It is
read-only, and most of what it captures is gone once the terminals close:

  ssh $TARGET '$REMOTE_WS/scripts/collect_logs.sh "what you had just done"' 

If a node cannot see something that is plainly running, clean the stale DDS
segments before debugging anything else - repeated restarts leave them behind
and discovery degrades until subscriptions match QoS and never fire:

  ssh $TARGET 'rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*'   # no ROS running
EOM
