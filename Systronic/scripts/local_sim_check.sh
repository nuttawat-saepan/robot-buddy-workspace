#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/sys20/projects/systonic-2307/Systronic"

cd "$WORKSPACE"
set +u
source /opt/ros/foxy/setup.bash
source install/setup.bash
set -u

ros2 run go2_control local_check
