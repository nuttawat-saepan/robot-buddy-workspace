"""Publish /clock from a replayed bag's own message stamps.

Foxy's `ros2 bag play` has no `--clock` option - it arrived in Galactic - so a
replayed bag gives every node the wall clock while the data carries the
timestamps it was recorded with. On the Livox bags that is a gap of days, and
it is not harmless: a node that stamps its output with `now()` instead of
copying the input stamp puts that output days into the future relative to the
TF tree, and every consumer downstream silently drops it.

That is exactly what happened the first time AMCL was run against 02_loop.
`pointcloud_to_laserscan` stamps /scan with `now()` rather than with the stamp
of the cloud it projected, so on the wall clock every scan came out dated
2026-08-25 while FAST-LIO's camera_init->body transform was dated 2026-08-19.
AMCL logged

    Message Filter dropping message: frame 'base_link' ... for reason 'Unknown'

for every single scan, published no pose at all, and gave no hint that time was
the problem.

This node is the missing `--clock`: it republishes the header stamp of a topic
coming out of the bag as /clock, so any node started with `use_sim_time:=true`
sees the bag's timeline and `now()` means what it should.

    ros2 run go2_control bag_clock
    ros2 run go2_control bag_clock --ros-args -p topic:=/livox/lidar

It publishes /clock and nothing else. It subscribes to one sensor topic. There
is no path from here to robot motion.

Run it only for a replay. On the robot the wall clock is already the right
clock and a second publisher of /clock would fight it.
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, \
    QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2


TYPES = {
    'sensor_msgs/msg/Imu': Imu,
    'sensor_msgs/msg/PointCloud2': PointCloud2,
}


class BagClock(Node):

    def __init__(self):
        super().__init__('bag_clock')

        # The IMU by default: 200 Hz on these bags, against 10 Hz for the
        # lidar. A coarse clock is not merely imprecise - it makes every timer
        # in every sim-time node tick in 100 ms steps.
        self.declare_parameter('topic', '/livox/imu')
        self.declare_parameter('type', 'sensor_msgs/msg/Imu')

        topic = self.get_parameter('topic').value
        type_name = self.get_parameter('type').value
        if type_name not in TYPES:
            raise RuntimeError(
                f'unsupported type {type_name!r}; known: {sorted(TYPES)}')

        # A clock publisher has to be its own clock source. use_sim_time here
        # would make the node wait for the /clock it is itself meant to produce.
        if self.get_parameter('use_sim_time').value:
            self.get_logger().warn(
                'use_sim_time is true on the clock publisher itself; ignoring '
                'it, this node always runs on the system clock')

        # Match what rosbag2 offers: RELIABLE, VOLATILE, depth 256. The default
        # SensorData profile would be BEST_EFFORT and would not connect.
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=256,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(Clock, '/clock', 10)
        self.create_subscription(TYPES[type_name], topic, self.on_msg, qos)

        self.last_ns = 0
        self.count = 0
        self.rewinds = 0
        self.create_timer(5.0, self.report)

        self.get_logger().info(f'publishing /clock from {topic} ({type_name})')

    def on_msg(self, msg):
        stamp = msg.header.stamp
        ns = stamp.sec * 1000000000 + stamp.nanosec

        # Time must not go backwards: rclcpp caches the last /clock value and a
        # rewind makes durations negative across the whole graph. Out-of-order
        # arrivals within a frame are normal, a bag restarting is not, so the
        # first is skipped quietly and the second is reported.
        if ns <= self.last_ns:
            if self.last_ns - ns > 1000000000:
                self.rewinds += 1
                self.get_logger().warn(
                    'source time jumped backwards by '
                    f'{(self.last_ns - ns) / 1e9:.1f}s - a bag restarting? '
                    'Restart this node too, or sim time stalls until the '
                    'replay catches up')
                self.last_ns = ns
                self._publish(stamp)
            return

        self.last_ns = ns
        self._publish(stamp)

    def _publish(self, stamp):
        out = Clock()
        out.clock = stamp
        self.pub.publish(out)
        self.count += 1

    def report(self):
        if self.count == 0:
            self.get_logger().warn(
                'no messages yet - nothing published to /clock, so every '
                'use_sim_time node is still frozen at t=0. Is the bag playing?')
        else:
            extra = f', {self.rewinds} rewinds' if self.rewinds else ''
            self.get_logger().info(
                f'/clock at {self.last_ns / 1e9:.3f}  '
                f'({self.count} published{extra})')


def main(args=None):
    rclpy.init(args=args)
    node = BagClock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
