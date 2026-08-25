"""Gate velocity commands on sensor freshness.

This is the second of two independent safety layers, and it is the one that
catches a failure `cmd_vel_node` cannot see:

    layer 1  cmd_vel_node   stops when /cmd_vel itself goes stale
                            - catches a dropped link or a dead planner
    layer 2  this node      stops when /scan or /odom goes stale
                            - catches a dead sensor while the link is fine

Without layer 2, a Mid-360 that stops publishing leaves Nav2 planning happily
against a costmap full of stale obstacles and an AMCL pose that no longer
moves, and `/cmd_vel` keeps arriving perfectly on time. Layer 1 sees nothing
wrong. The robot drives on a picture of the world that has stopped updating.

The node sits in the middle of the command path rather than shouting zeros
onto a shared topic, so there is no race with the planner:

    Nav2 --(input_topic)--> sensor_watchdog --(output_topic)--> cmd_vel_node

Fresh: the command passes through unchanged. Stale: a single zero goes out and
nothing further passes until every watched topic is fresh again.

## Default topics are deliberately inert

`output_topic` defaults to `/cmd_vel_safe`, not `/cmd_vel`, so that merely
running this node cannot put a publisher on the topic the robot listens to.
The standing field check

    ros2 topic info /cmd_vel        expect: Unknown topic '/cmd_vel'

has to keep working while everything except movement is being tested. Wiring
`output_topic:=/cmd_vel` is part of arming the system for the movement stage,
alongside the robot_ack gate on `cmd_vel_node`.

    ros2 run go2_control sensor_watchdog
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class SensorWatchdog(Node):

    def __init__(self):
        super().__init__('sensor_watchdog')

        self.declare_parameter('input_topic', '/cmd_vel_nav_preview')
        self.declare_parameter('output_topic', '/cmd_vel_safe')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        # /scan arrives at 10 Hz and /odom at 10 Hz, so 0.5 s is five missed
        # messages: past ordinary jitter, well inside the distance the robot
        # covers in half a second at the speeds configured for the first
        # movement tests.
        self.declare_parameter('sensor_timeout', 0.5)
        # Nothing has been seen yet at startup. Blocking on that would be
        # correct but noisy, so the gate stays closed silently until the first
        # message of each kind arrives; it is only after a topic has been seen
        # that its disappearance is an event worth reporting.
        self.declare_parameter('report_period', 5.0)

        # This node must never run on simulated time, and the launch files
        # pass use_sim_time: False for it explicitly.
        #
        # Two things break otherwise, and the second is not obvious. Elapsed
        # time is measured against /clock, which on a replay is published from
        # the same bag that feeds /scan and /odom - so when the data stops,
        # time stops and the timeout never fires. Worse, rclpy drives timers
        # from the ROS clock too, so tick() stops being called at all and the
        # node cannot act even in principle. Measured on 02_loop: killing the
        # bag left this node reporting "passing" forever, with its own log
        # frozen at the moment the bag died.
        #
        # A watchdog cannot be driven by the clock of the thing it is watching.
        if self.get_parameter('use_sim_time').value:
            self.get_logger().error(
                'use_sim_time is true on the watchdog: its timer is driven by '
                '/clock, so it will stop firing exactly when the data it '
                'guards stops. Start it with use_sim_time:=false.')

        self.timeout = float(self.get_parameter('sensor_timeout').value)
        out_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self.on_cmd, 10)
        # SensorDataQoS, not the default. pointcloud_to_laserscan publishes
        # /scan BEST_EFFORT, and a RELIABLE subscription simply never connects
        # to it - no error, no warning, just nothing received. The watchdog
        # then reports BLOCKING on scan forever while the sensor is perfectly
        # healthy, which in the field would look like a dead LiDAR and would
        # stop the robot for no reason. AMCL subscribes the same way, which is
        # why it works and this did not.
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            lambda _m: self.mark('scan'), qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            lambda _m: self.mark('odom'), 10)

        self.seen = {}
        self.blocking = False
        self.zero_sent = False
        self.passed = 0
        self.blocked = 0

        self.create_timer(0.05, self.tick)
        self.create_timer(
            float(self.get_parameter('report_period').value), self.report)

        if out_topic in ('/cmd_vel', 'cmd_vel'):
            self.get_logger().warn(
                '*** output_topic is /cmd_vel: this node can command the real '
                'robot. Operator on the remote, emergency stop ready. ***')
        else:
            self.get_logger().info(
                f'gating {self.get_parameter("input_topic").value} -> '
                f'{out_topic}, which the robot does not listen to')

    def now(self):
        """Monotonic wall time, deliberately not the ROS clock.

        A safety timeout must not be measured on a clock that the failure it
        guards against can stop. Under use_sim_time the ROS clock comes from
        /clock, which on a replay is published from the same bag that feeds
        /scan and /odom - so when the data stops, time stops with it, elapsed
        time stays at zero and the timeout never fires. Measured on a replay of
        02_loop: killing the bag left this node reporting "passing"
        indefinitely.

        That would be a false negative in exactly the situation the node
        exists for, and it would also make every bench test of the watchdog
        pass for the wrong reason. Wall time is unaffected by what the sensors
        are doing.
        """
        return time.monotonic()

    def mark(self, name):
        self.seen[name] = self.now()

    def stale(self):
        """Names of watched topics that are missing or too old."""
        now = self.now()
        bad = []
        for name in ('scan', 'odom'):
            last = self.seen.get(name)
            if last is None or now - last > self.timeout:
                bad.append(name)
        return bad

    def on_cmd(self, msg):
        if self.stale():
            self.blocked += 1
            return
        self.pub.publish(msg)
        self.passed += 1
        self.zero_sent = False

    def tick(self):
        bad = self.stale()
        if bad and not self.zero_sent:
            self.pub.publish(Twist())
            self.zero_sent = True
            self.blocking = True
            self.get_logger().warn(
                f'STOP: {", ".join(bad)} stale past {self.timeout:.2f}s - '
                'commands blocked until they return')
        elif not bad and self.blocking:
            self.blocking = False
            self.get_logger().info('sensors fresh again, commands passing')

    def report(self):
        bad = self.stale()
        state = f'BLOCKING on {", ".join(bad)}' if bad else 'passing'
        self.get_logger().info(
            f'{state}  ({self.passed} passed, {self.blocked} blocked)')


def main(args=None):
    rclpy.init(args=args)
    node = SensorWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best effort: if this node is going down while the robot is moving,
        # the last thing it should do is ask for zero.
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
