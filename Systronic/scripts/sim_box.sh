#!/usr/bin/env bash
# Put a box in the running simulation, or take it away.
#
#     scripts/sim_box.sh add  box1 2.75 -3.6          0.5 m box at x 2.75, y -3.6
#     scripts/sim_box.sh add  box2 1.0 -2.0 0.8       0.8 m box
#     scripts/sim_box.sh remove box1
#
# x, y are map coordinates, the same numbers the web page shows for a waypoint.
# The box is 0.8 m tall, so the scan sees it; anything under 25 cm would not be.
# Use names of a few letters: a one-letter name once hung Gazebo's spawn service.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/foxy/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/src/go2_control/config/fastdds_udp_only.xml"
export ROS_DOMAIN_ID=77

exec python3 - "$@" <<'PY'
import sys
import time

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity

from go2_control.sim_scenarios import box_at, box_sdf

if len(sys.argv) < 3 or sys.argv[1] not in ('add', 'remove') \
        or (sys.argv[1] == 'add' and len(sys.argv) < 5):
    sys.exit('usage: sim_box.sh add NAME X Y [SIZE] | remove NAME')
action, name = sys.argv[1], sys.argv[2]
rclpy.init()
node = rclpy.create_node('sim_box')
if action == 'add':
    size = float(sys.argv[5]) if len(sys.argv) > 5 else 0.5
    client = node.create_client(SpawnEntity, '/spawn_entity')
    request = SpawnEntity.Request(name=name, xml=box_sdf(
        name, box_at(float(sys.argv[3]), float(sys.argv[4]), size)))
else:
    client = node.create_client(DeleteEntity, '/delete_entity')
    request = DeleteEntity.Request(name=name)
if not client.wait_for_service(timeout_sec=10):
    sys.exit('no simulation running (no /spawn_entity on ROS domain 77)')
future = client.call_async(request)
deadline = time.time() + 20
while not future.done() and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
reply = future.result()
print(reply.status_message if reply else 'no reply from Gazebo')
sys.exit(0 if reply and reply.success else 1)
PY
