#!/usr/bin/env bash
# Read-only check of the site setup. Run it on the board once the stack is up:
#     source scripts/setup_robot_env.sh   # or however this shell gets ROS and onsite.env
#     ./scripts/onsite_check.sh
# It reads what is already set and changes nothing, so it needs no editing per site:
# addresses come from onsite.env. Nothing here can move the robot.
set -u
pass=0; warn=0; fail=0
ok()   { echo "  OK    $1"; pass=$((pass + 1)); }
note() { echo "  WARN  $1"; warn=$((warn + 1)); }
bad()  { echo "  FAIL  $1"; fail=$((fail + 1)); }
param() { timeout 8 ros2 param get "$1" "$2" 2>/dev/null | tail -1 | sed 's/.* is: //'; }
port_open() { timeout 3 bash -c "exec 3<>/dev/tcp/$1/$2" 2>/dev/null; }

echo "Clock"
year=$(date +%Y)
if [ "$year" -ge 2025 ]; then ok "$(date '+%F %T')"; else bad "clock reads $year - sync it before starting FAST-LIO"; fi

echo "Environment"
[ -n "${SITE_MAP:-}" ] && [ -f "$SITE_MAP" ] && ok "SITE_MAP $SITE_MAP" || bad "SITE_MAP missing: ${SITE_MAP:-unset}"
[ "${GO2_ENFORCE_SPREAD_LIMIT:-true}" != "false" ] && ok "spread limit enforced" || note "GO2_ENFORCE_SPREAD_LIMIT=false - a wide cloud will not stop a mission"
[ "${UNITREE_AUTO_ARM:-0}" = "1" ] && note "UNITREE_AUTO_ARM=1 - the bridge arms without --robot-ack" || ok "auto arm off"

echo "Network"
if [ -n "${MQTT_BROKER:-}" ]; then
  port_open "$MQTT_BROKER" 1883 && ok "MQTT broker $MQTT_BROKER:1883" || bad "cannot reach MQTT broker $MQTT_BROKER:1883"
else bad "MQTT_BROKER is not set"; fi
port_open 127.0.0.1 9999 && ok "camera signaling :9999" || note "nothing on :9999 here - camera signaling not running on this machine"

echo "Topics"
topics=$(timeout 10 ros2 topic list 2>/dev/null)
for t in /livox/lidar /livox/imu /Odometry /scan /map /amcl_pose /particlecloud /tf /local_costmap/costmap /localization_watch/status; do
  grep -qx "$t" <<<"$topics" && ok "$t" || bad "$t is not published"
done
grep -qx /scan_low <<<"$topics" && ok "/scan_low (low obstacle scan on)" || echo "  info  /scan_low off"

echo "Settings in use"
echo "  alpha3                $(param /amcl alpha3)"
echo "  amcl tf_broadcast     $(param /amcl tf_broadcast)  (False means the guard owns map->odom)"
echo "  planner tolerance     $(param /planner_server GridBased.tolerance)"
echo "  local inflation       $(param /local_costmap/local_costmap inflation_layer.inflation_radius)"
echo "  global inflation      $(param /global_costmap/global_costmap inflation_layer.inflation_radius)"
echo "  costmap sources       $(param /local_costmap/local_costmap obstacle_layer.observation_sources)"
echo "  max_vel_x             $(param /controller_server FollowPath.max_vel_x)"
echo "  max_vel_theta         $(param /controller_server FollowPath.max_vel_theta)"

echo "Result: $pass OK, $warn warnings, $fail failures"
[ "$fail" -eq 0 ]
