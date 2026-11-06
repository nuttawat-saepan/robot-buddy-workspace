#!/usr/bin/env bash
# After a new FE/BE deploy: is what was changed on 2026-11-06 still there?
#
#     scripts/check_web_patches.sh
#
# Those changes were made in the built files under ~/robot-app, not in the
# FE/BE source, so a deploy from source drops any that were not carried over.
# Checks the files, then asks the running services. Changes nothing.
APP="${ROBOT_APP:-$HOME/robot-app}"
FE="$APP/dashboard"
pass=0; fail=0
check() {   # check "what" command...
    if "${@:2}" >/dev/null 2>&1; then echo "  OK    $1"; pass=$((pass+1))
    else echo "  LOST  $1"; fail=$((fail+1)); fi
}
fe_has()     { grep -rqsF --include='*.js' -- "$1" "$FE/client/assets" "$FE/server/assets"; }
fe_lacks()   { ! fe_has "$1"; }
fe_matches() { grep -rqsE --include='*.js' -- "$1" "$FE/client/assets" "$FE/server/assets"; }
fe_no_match(){ ! fe_matches "$1"; }

echo "Frontend ($FE)"
check "Joystick tab shown (was hidden and disabled)" \
      fe_no_match 'value: ?"manual", ?className: ?"hidden"'
check "no 'Mission control' header above the tabs" \
      fe_no_match 'title: ?"Mission control", ?bodyClassName: ?"p-0"'
check "map text in Thai: กำลังรอแผนที่จากหุ่น…" fe_has 'กำลังรอแผนที่จากหุ่น…'
check "map text in Thai: แผนที่จะขึ้นเองภายใน 30 วินาที" fe_has 'แผนที่จะขึ้นเองภายใน 30 วินาที'
check "old map text gone" fe_lacks 'The robot republishes the map every 30 seconds.'
check "Robot settings needs no typed token (demo)" fe_matches 'useState\("demo"\)'

echo "Backend ($APP/backend, $APP/config)"
# The build may be one bundled dist/main.js or one file per module.
check "settings demo switch in the backend build" \
      grep -rqs "ROBOT_SETTINGS_AUTH" "$APP/backend/dist"
check "demo switch turned on in backend.env" \
      grep -qs '^ROBOT_SETTINGS_AUTH=off' "$APP/config/backend.env"

echo "Dashboard server ($APP/manager/serve-dashboard.mjs)"
check "/slam forwarded to map_maker" grep -qs "isSlam" "$APP/manager/serve-dashboard.mjs"
check "page files not cached for a year" \
      bash -c "! grep -qs 'max-age=31536000, immutable' '$APP/manager/serve-dashboard.mjs'"

echo "Running services"
code() { curl -s -o /dev/null -m 15 -w '%{http_code}' "$@"; }
check "dashboard answers on :5173" test "$(code http://127.0.0.1:5173/)" = 200
check "settings read without a token is not refused (not 401/403)" \
      bash -c "c=\$(curl -s -o /dev/null -m 15 -w '%{http_code}' http://127.0.0.1:3101/api/robot/settings); [ \"\$c\" != 401 ] && [ \"\$c\" != 403 ]"
check "arming the bridge still needs the token (401)" \
      test "$(code -X POST -H 'Content-Type: application/json' -d '{"action":"disarm"}' http://127.0.0.1:3101/api/robot/bridge)" = 401
check "/slam reaches map_maker or says it is not running" \
      bash -c "curl -s -m 5 http://127.0.0.1:5173/slam/api/status | grep -q -e state -e 'Map maker unavailable'"

echo
echo "$pass OK, $fail LOST"
[ "$fail" = 0 ]
