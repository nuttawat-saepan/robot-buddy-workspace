#!/usr/bin/env bash
# Run obstacle scenarios in Gazebo, each against a freshly started stack.
#
#     scripts/run_sim_obstacle.sh OUT_DIR RUNS scenario [scenario ...]
#
# Headless. Uses its own ROS domain so that it cannot share a graph with a robot
# or a stack on domain 0 or the site domain. Appends one JSON line per run to
# OUT_DIR/results.jsonl. Set DELAY and LINEAR_ACCEL to change the assumed
# actuator.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$1"; RUNS="$2"; shift 2
mkdir -p "$OUT"

source /opt/ros/foxy/setup.bash
source "$WS/install/setup.bash"
source /usr/share/gazebo/setup.sh 2>/dev/null
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/src/go2_control/config/fastdds_udp_only.xml"
export ROS_DOMAIN_ID=77
export GAZEBO_MASTER_URI=http://127.0.0.1:11377
export LIO_SENSOR_X=0.30
export ROS_LOG_DIR="$OUT/ros_logs"

stop_stack() {
    kill -INT -- "-$1" 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8 9 10; do
        kill -0 -- "-$1" 2>/dev/null || return 0
        sleep 1
    done
    kill -KILL -- "-$1" 2>/dev/null
    sleep 1
}

for scenario in "$@"; do
    start=$(python3 -c "
from go2_control.sim_scenarios import SCENARIOS
s = SCENARIOS['$scenario']; print(s.start[0], s.start[1])")
    for run in $(seq 1 "$RUNS"); do
        log="$OUT/${scenario}_${run}.launch.log"
        setsid ros2 launch go2_control sim_obstacle.launch.py \
            "start_x:=${start% *}" "start_y:=${start#* }" \
            "delay:=${DELAY:-0.15}" "linear_accel:=${LINEAR_ACCEL:-1.0}" \
            >"$log" 2>&1 &
        stack=$!
        timeout 300 ros2 run go2_control sim_obstacle_test \
            --scenario "$scenario" 2>>"$log" | tee -a "$OUT/results.jsonl"
        stop_stack "$stack"
    done
done
