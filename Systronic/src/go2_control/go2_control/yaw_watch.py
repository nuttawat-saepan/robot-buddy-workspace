#!/usr/bin/env python3
"""Print the robot's heading in degrees, and how far it is from a target.

tf2_echo prints a quaternion, which is not a number anyone can compare to a
goal while standing next to the robot. This prints the heading in degrees
once a second, and - given --target - the signed error and which way the
robot still has to turn.

Read-only: it subscribes to TF and publishes nothing.

    ros2 run go2_control yaw_watch
    ros2 run go2_control yaw_watch --target -140.8
    ros2 run go2_control yaw_watch --target 3.1416 --radians

Written on site on 2026-09-14, watching a robot sit at its goal position
unable to finish because the heading was outside yaw_goal_tolerance and
there was no way to see by how much.
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node

import tf2_ros


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class YawWatch(Node):

    def __init__(self, target_deg, tol_deg, frames):
        super().__init__('yaw_watch')
        self.target = target_deg
        self.tol = tol_deg
        self.map_frame, self.base_frame = frames
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.create_timer(1.0, self.tick)
        self.last = None

    def tick(self):
        try:
            tf = self.buf.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as exc:                   # noqa: BLE001
            print('no %s -> %s: %s' % (self.map_frame, self.base_frame, exc))
            return
        t = tf.transform.translation
        deg = math.degrees(yaw_of(tf.transform.rotation))

        # Drift while the robot is not moving is the thing worth seeing, and
        # it is invisible in an absolute reading. This shows the change since
        # the previous second alongside it.
        delta = '' if self.last is None else '  d=%+6.2f' % (
            math.degrees(math.atan2(
                math.sin(math.radians(deg - self.last)),
                math.cos(math.radians(deg - self.last)))))
        self.last = deg

        line = 'x=%7.3f y=%7.3f  yaw=%+7.2f deg%s' % (t.x, t.y, deg, delta)

        if self.target is not None:
            err = math.degrees(math.atan2(
                math.sin(math.radians(self.target - deg)),
                math.cos(math.radians(self.target - deg))))
            way = 'left' if err > 0 else 'right'
            mark = 'IN' if abs(err) <= self.tol else 'OUT'
            line += '   target=%+7.2f  err=%+7.2f (%s %s, tol %.1f)' % (
                self.target, err, way, mark, self.tol)
        print(line, flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--target', type=float, default=None,
                        help='Target heading, degrees unless --radians.')
    parser.add_argument('--radians', action='store_true')
    parser.add_argument('--tolerance', type=float, default=17.2,
                        help='Degrees. Default is yaw_goal_tolerance 0.30 rad.')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_link')
    args, ros_args = parser.parse_known_args(
        argv if argv is not None else sys.argv[1:])

    target = args.target
    if target is not None and args.radians:
        target = math.degrees(target)

    rclpy.init(args=ros_args)
    node = YawWatch(target, args.tolerance, (args.map_frame, args.base_frame))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
