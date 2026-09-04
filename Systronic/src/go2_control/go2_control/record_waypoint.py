"""Write down where the robot is now, as a waypoint send_mission can replay.

Read-only. It listens to TF and writes a file. It publishes nothing and cannot
move the robot.

    ros2 run go2_control record_waypoint --file missions/site_a.json

Press Enter to record the robot's current pose. Type a name first to label it,
`c` on its own to mark the previous point as a capture point, `u` to undo, and
Ctrl-C to finish. The file is written after every entry, so a session that ends
badly still leaves everything recorded up to that point.

The pose comes from TF `map -> base_link`, which is AMCL's answer composed with
odometry - the same pose Nav2 will use to decide it has arrived. Recording from
`/amcl_pose` instead would drop the odometry since AMCL last updated, which at
`update_min_d` of 0.20 m can be most of the distance between two close
waypoints.

There is an existing waypoint_gen node. It writes a different format into
~/turtlebot3_ws/waypoints, a path from another project, and it is not what
send_mission reads.
"""

import argparse
import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WaypointRecorder(Node):

    def __init__(self, args):
        super().__init__('record_waypoint')
        self.map_frame = args.map_frame
        self.base_frame = args.base_frame
        self.path = args.file
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.waypoints = []

        if os.path.exists(self.path):
            with open(self.path) as handle:
                self.waypoints = json.load(handle).get('waypoints', [])
            print(f'{self.path} already has {len(self.waypoints)} waypoint(s); '
                  f'new ones are appended')

    def pose_now(self):
        tf = self.buffer.lookup_transform(
            self.map_frame, self.base_frame, rclpy.time.Time())
        t = tf.transform.translation
        return t.x, t.y, yaw_of(tf.transform.rotation)

    def save(self):
        directory = os.path.dirname(os.path.abspath(self.path))
        os.makedirs(directory, exist_ok=True)
        with open(self.path, 'w') as handle:
            json.dump({'frame_id': self.map_frame,
                       'waypoints': self.waypoints}, handle, indent=2)
            handle.write('\n')


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Record the robot pose as waypoints for send_mission.')
    parser.add_argument('--file', default='missions/waypoints.json')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_link')
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = WaypointRecorder(args)

    print('\nwaiting for TF map -> base_link')
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            node.pose_now()
            break
        except Exception:                          # noqa: BLE001 - keep waiting
            continue
    else:
        print('no TF map -> base_link. Is AMCL running and localised?')
        node.destroy_node()
        rclpy.shutdown()
        return

    print('ready. Enter to record, a name then Enter to label it, '
          '"c" to mark the last one as a capture point, "u" to undo, Ctrl-C to finish')

    try:
        while True:
            try:
                text = input('> ').strip()
            except EOFError:
                break

            rclpy.spin_once(node, timeout_sec=0.05)

            if text == 'u':
                if node.waypoints:
                    gone = node.waypoints.pop()
                    node.save()
                    print(f'  removed {gone["name"]}')
                else:
                    print('  nothing to undo')
                continue

            if text == 'c':
                if node.waypoints:
                    node.waypoints[-1]['capture'] = True
                    node.save()
                    print(f'  {node.waypoints[-1]["name"]} is now a capture point')
                else:
                    print('  record a waypoint first')
                continue

            try:
                x, y, yaw = node.pose_now()
            except Exception as exc:               # noqa: BLE001 - report it
                print(f'  no pose right now: {exc}')
                continue

            name = text or f'wp{len(node.waypoints) + 1}'
            node.waypoints.append(
                {'name': name, 'x': round(x, 3), 'y': round(y, 3),
                 'yaw': round(yaw, 4)})
            node.save()
            print(f'  {len(node.waypoints):2}. {name:16} x={x:7.2f} y={y:7.2f} '
                  f'yaw={math.degrees(yaw):7.1f} deg')
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        print(f'\n{len(node.waypoints)} waypoint(s) written to {node.path}')
        print(f'run it with: ros2 run go2_control send_mission --file {node.path}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
