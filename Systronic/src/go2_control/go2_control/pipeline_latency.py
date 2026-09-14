#!/usr/bin/env python3
"""How late each stage of the chain into Nav2 is, measured at once.

Read-only: it subscribes and publishes nothing.

    ros2 run go2_control pipeline_latency            # 15 s, then a table
    ros2 run go2_control pipeline_latency --seconds 30

For every stage it reports the age of each message when it arrives - wall
clock now minus the message's header stamp - and how regularly it arrives:

    /livox/lidar             the sensor as the driver hands it over
    /Odometry                after FAST-LIO
    tf odom->base_link       after lio_odom_relay
    /scan                    after pointcloud_to_laserscan
    /amcl_pose               after AMCL
    tf map->odom             AMCL's correction
    /cmd_vel_nav_preview     Nav2's command (no stamp: rate and gaps only)
    /cmd_vel_safe            after the watchdog

Differences between rows are the stages: /Odometry age minus /livox/lidar age
is what FAST-LIO costs, and so on. Run it on the board and every age uses one
clock, so the numbers are real. Run the same thing on the MiniPC with the
clocks set to match and the difference from the board's table is the network.

Two readings mean something other than slowness. An age of thousands of
seconds, or negative, is two clocks disagreeing - the board booted at 1970, or
the LiDAR stamping with its own clock - and every TF lookup downstream of it
will misbehave in ways that look like delay. And tf map->odom is supposed to
be stale by design: AMCL only publishes on update and post-dates by
transform_tolerance, so a large or negative age there is expected.

Written on site on 2026-09-14, when the robot was plainly deciding it had
arrived late and there was no single place to see where the time was going.
"""
import argparse
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage


class Stage:

    def __init__(self, name):
        self.name = name
        self.ages = []
        self.arrivals = []

    def hit(self, stamp=None):
        now = time.time()
        self.arrivals.append(now)
        if stamp is not None:
            t = stamp.sec + stamp.nanosec * 1e-9
            if t > 0:
                self.ages.append(now - t)

    def row(self, seconds):
        n = len(self.arrivals)
        if n == 0:
            return '%-24s  no messages' % self.name
        rate = n / seconds
        gaps = [b - a for a, b in zip(self.arrivals, self.arrivals[1:])]
        gap = max(gaps) if gaps else float('nan')
        if self.ages:
            med = statistics.median(self.ages) * 1000
            worst = max(self.ages) * 1000
            flag = ''
            if abs(med) > 5000:
                flag = '  <- clocks disagree, not a delay'
            elif med < -50:
                flag = '  <- stamped in the future'
            age = '%9.0f %9.0f' % (med, worst)
        else:
            age, flag = '%9s %9s' % ('-', '-'), ''
        return '%-24s %5.1f Hz %7.0f  %s%s' % (
            self.name, rate, gap * 1000, age, flag)


class Latency(Node):

    def __init__(self):
        super().__init__('pipeline_latency')
        best = QoSProfile(depth=10)
        best.reliability = ReliabilityPolicy.BEST_EFFORT
        rel = QoSProfile(depth=10)

        self.s = {k: Stage(k) for k in (
            '/livox/lidar', '/Odometry', 'tf odom->base_link', '/scan',
            '/amcl_pose', 'tf map->odom', '/cmd_vel_nav_preview',
            '/cmd_vel_safe')}

        self.create_subscription(
            PointCloud2, '/livox/lidar',
            lambda m: self.s['/livox/lidar'].hit(m.header.stamp), best)
        self.create_subscription(
            Odometry, '/Odometry',
            lambda m: self.s['/Odometry'].hit(m.header.stamp), rel)
        self.create_subscription(
            LaserScan, '/scan',
            lambda m: self.s['/scan'].hit(m.header.stamp), best)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            lambda m: self.s['/amcl_pose'].hit(m.header.stamp), rel)
        self.create_subscription(
            Twist, '/cmd_vel_nav_preview',
            lambda m: self.s['/cmd_vel_nav_preview'].hit(), rel)
        self.create_subscription(
            Twist, '/cmd_vel_safe',
            lambda m: self.s['/cmd_vel_safe'].hit(), rel)
        self.create_subscription(TFMessage, '/tf', self._on_tf, rel)

    def _on_tf(self, msg):
        for t in msg.transforms:
            pair = (t.header.frame_id.lstrip('/'), t.child_frame_id.lstrip('/'))
            if pair == ('odom', 'base_link'):
                self.s['tf odom->base_link'].hit(t.header.stamp)
            elif pair == ('map', 'odom'):
                self.s['tf map->odom'].hit(t.header.stamp)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--seconds', type=float, default=15.0)
    args, ros_args = parser.parse_known_args(
        argv if argv is not None else sys.argv[1:])

    rclpy.init(args=ros_args)
    node = Latency()
    print('listening %.0f s ... (move the robot or send a goal for the '
          'command rows)' % args.seconds, flush=True)
    end = time.time() + args.seconds
    try:
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    print()
    print('%-24s %8s %8s %9s %9s' % ('stage', 'rate', 'max gap',
                                     'age med', 'age max'))
    print('%-24s %8s %8s %9s %9s' % ('', '', 'ms', 'ms', 'ms'))
    for st in node.s.values():
        print(st.row(args.seconds))
    print()
    print('stage cost = difference in "age med" between consecutive rows.')
    print('host clock now: %s' % time.strftime('%Y-%m-%d %H:%M:%S'))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
