#!/usr/bin/env bash
# Measure what the navigation stack costs on this machine.
#
#     ./scripts/measure_board_load.sh [seconds]        default 60
#
# Read-only. It samples `ps` and reads /proc. It starts no node, publishes
# nothing, and cannot move the robot. Run it in its own terminal while the
# stack is already up and, ideally, while a goal is active - an idle stack
# costs far less than one that is planning, and the idle number is the one
# that misleads.
#
# The board has 4 cores. The acceptance figure from LIVOX_DEPLOYMENT_PLAN.md
# section 2.4 is that the robot-side set stays under about 2 of those 4 with
# Unitree's own stack running, so the leg controller is never starved.
#
# Reference, measured on the MiniPC (i5-1235U) with a goal active. ARM cores
# are slower per core, so expect these to be larger here - that is the whole
# reason for measuring rather than assuming:
#
#     planner_server   53.6%    fastlio_mapping          28.8%
#     controller_serv  53.3%    pointcloud_to_laserscan  18.6%
#     amcl             45.4%    lio_odom_relay            9.5%

set -uo pipefail

DURATION="${1:-60}"
INTERVAL=1
OUT="${OUT:-/tmp/board_load_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
SAMPLES="$OUT/samples.csv"

# Everything worth watching: our stack, Nav2, and Unitree's own processes,
# because the headroom left for the leg controller is the point of the exercise.
PATTERN='fastlio|amcl|map_server|planner_server|controller_server|bt_navigator|behavior_server|recoveries|lifecycle_manager|pointcloud_to_laserscan|lio_odom_relay|sensor_watchdog|cmd_vel|unitree|livox|bag_clock|occupancy_grid|camera|april'

CORES="$(nproc)"
echo "machine   $(uname -m), ${CORES} cores"
echo "memory    $(free -m | awk '/^Mem:/ {printf "%d MB total, %d MB available", $2, $7}')"
echo "sampling  ${DURATION}s at ${INTERVAL}s, writing to $OUT"
echo

echo "t,comm,pid,pcpu,rss_kb" > "$SAMPLES"
END=$(( $(date +%s) + DURATION ))
T=0
while [ "$(date +%s)" -lt "$END" ]; do
    ps -eo comm=,pid=,pcpu=,rss= --sort=-pcpu \
        | awk -v t="$T" -v pat="$PATTERN" \
              'tolower($1) ~ pat {printf "%s,%s,%s,%s,%s\n", t, $1, $2, $3, $4}' \
        >> "$SAMPLES"
    # Whole-machine load, so a cost that is not in the pattern above still shows.
    echo "$T $(cut -d' ' -f1-3 /proc/loadavg)" >> "$OUT/loadavg.txt"
    sleep "$INTERVAL"
    T=$(( T + INTERVAL ))
done

python3 - "$SAMPLES" "$CORES" "$OUT" <<'PY'
import csv, sys
from collections import defaultdict

samples_path, cores, out = sys.argv[1], int(sys.argv[2]), sys.argv[3]
by_proc = defaultdict(list)
by_time = defaultdict(float)
rss = defaultdict(float)
with open(samples_path) as handle:
    for row in csv.DictReader(handle):
        cpu = float(row['pcpu'])
        by_proc[row['comm']].append(cpu)
        by_time[row['t']] += cpu
        rss[row['comm']] = max(rss[row['comm']], float(row['rss_kb']) / 1024.0)

if not by_proc:
    print('no matching processes were running - is the stack up?')
    raise SystemExit(1)

print(f"{'process':26}{'mean %cpu':>11}{'peak %cpu':>11}{'peak MB':>10}")
print('-' * 58)
total_mean = 0.0
for name in sorted(by_proc, key=lambda n: -sum(by_proc[n]) / len(by_proc[n])):
    values = by_proc[name]
    mean = sum(values) / len(values)
    total_mean += mean
    print(f'{name:26}{mean:>11.1f}{max(values):>11.1f}{rss[name]:>10.0f}')

peak_total = max(by_time.values()) if by_time else 0.0
print('-' * 58)
print(f"{'TOTAL':26}{total_mean:>11.1f}{peak_total:>11.1f}")
print()
print(f'mean cost  {total_mean / 100.0:.2f} of {cores} cores')
print(f'peak cost  {peak_total / 100.0:.2f} of {cores} cores')
print()

budget = cores * 0.5
if total_mean / 100.0 <= budget:
    print(f'PASS  under {budget:.1f} cores, there is headroom for the leg controller')
else:
    print(f'OVER  above {budget:.1f} cores. Knobs to turn, none needing a rebuild:')
    print('  controller_frequency   nav2_livox_go2.yaml   20 -> 10 Hz or lower.')
    print('                         At 0.05 m/s, 20 Hz replans every 2.5 mm.')
    print('  planner rate           nav2_livox_go2.yaml')
    print('  costmap update_frequency / publish_frequency')
    print('  max_particles          amcl_livox.yaml')
    print('  laser_max_beams        amcl_livox.yaml')
    print('  point_filter_num       fast_lio2_mid360.yaml')
    print('  /scan rate             10 -> 5 Hz')
    print('  run the camera only at capture points, not continuously')

with open(f'{out}/summary.txt', 'w') as handle:
    handle.write(f'mean {total_mean:.1f}%  peak {peak_total:.1f}%  cores {cores}\n')
    for name in sorted(by_proc):
        values = by_proc[name]
        handle.write(f'{name} mean {sum(values)/len(values):.1f} peak {max(values):.1f}\n')
print(f'saved: {out}/summary.txt and samples.csv')
PY
