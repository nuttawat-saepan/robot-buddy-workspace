"""A robot that exists only in the map, so the whole mission flow can be tested.

Replaying a bag proves the sensing and localisation path, but it cannot prove
the mission path, because the robot in a bag has already finished walking. It
never arrives anywhere. Everything that fires on arrival is therefore untested:

    reaching a waypoint          the goal is never met
    the capture spin             fires on arrival
    the photograph upload        follows the capture
    AprilTag correction          requested at a waypoint
    a mission completing         needs every waypoint reached

This node closes that loop without hardware. It integrates the velocity Nav2
sends into a pose, and ray-casts the occupancy map from that pose to produce
the `/scan` AMCL localises against. Nav2 drives, the robot moves, the scan
changes, AMCL follows, waypoints are reached, and `main.py` runs a mission from
end to end.

It is a **simulation** and it is honest about what it is not: no gait, no
slip, no leg dynamics, and a laser that sees the map rather than the world. It
will not tell you whether the Go2 can physically follow a path. It tells you
whether the software flow completes.

## What it replaces

Run this **instead of** `livox_robot.launch.py` and the bag. It publishes
`/odom`, the `odom -> base_link` transform and `/scan`, which are exactly what
the robot side otherwise provides. Running both would put two publishers on all
three, which breaks the TF tree silently.

    ros2 launch go2_control livox_ground.launch.py replay:=false
    ros2 run go2_control fake_base

## It cannot move a real robot

It subscribes to velocity and publishes sensor data. It has no publisher on
`/cmd_vel` and no connection to the Unitree SDK.
"""
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, \
    QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class FakeBase(Node):

    def __init__(self):
        super().__init__('fake_base')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav_preview')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_service', '/map_server/map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('scan_rate', 10.0)
        # Matches pc2scan_livox_lio.yaml so AMCL sees the same beam count and
        # spacing it would from the real projection.
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', 0.0087)
        self.declare_parameter('range_min', 0.5)
        self.declare_parameter('range_max', 20.0)
        # Where the robot starts, in the map frame. The default matches the
        # AMCL seed in amcl_livox.yaml.
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        # Odometry that tracks perfectly would let AMCL sit at zero correction
        # forever, which is not a useful rehearsal: the point of AMCL is to
        # absorb drift. A small systematic scale error gives it something real
        # to do without making the run unrepeatable.
        self.declare_parameter('odom_drift_scale', 1.02)

        self.cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.drift = float(self.get_parameter('odom_drift_scale').value)

        self.angles = np.arange(
            float(self.get_parameter('angle_min').value),
            float(self.get_parameter('angle_max').value),
            float(self.get_parameter('angle_increment').value))
        self.cos_a = np.cos(self.angles)
        self.sin_a = np.sin(self.angles)

        # True pose in the map frame, and the odometry estimate of it. They
        # differ by the drift factor, which is the whole point.
        self.x = float(self.get_parameter('start_x').value)
        self.y = float(self.get_parameter('start_y').value)
        self.yaw = float(self.get_parameter('start_yaw').value)
        self.ox, self.oy, self.oyaw = 0.0, 0.0, 0.0

        self.vx = self.vy = self.wz = 0.0
        self.grid = None
        self.map_info = None
        self.last = self.now()

        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 20)
        self.scan_pub = self.create_publisher(
            LaserScan, self.get_parameter('scan_topic').value,
            qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)

        self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self.on_map, map_qos)

        # The latched topic above is the normal route and usually works, but it
        # is not dependable for a node that starts late. map_server publishes
        # /map once, TRANSIENT_LOCAL, and FastRTPS does not reliably replay
        # that sample to a reader that joins minutes afterwards: the
        # subscription appears in `ros2 topic info /map -v` with matching QoS
        # and the callback simply never fires. Observed repeatedly here, while
        # a freshly started probe on the same machine received it fine.
        #
        # nav2_map_server also serves the map on a request/response service,
        # which has no history semantics to get wrong. Asked for once if the
        # topic has not delivered.
        self.map_client = self.create_client(
            GetMap, self.get_parameter('map_service').value)
        self.map_request_sent = False
        self.create_timer(2.0, self.request_map)

        self.create_timer(1.0 / float(self.get_parameter('rate').value),
                          self.step)
        self.create_timer(1.0 / float(self.get_parameter('scan_rate').value),
                          self.publish_scan)
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f'fake base listening on {self.cmd_topic}, starting at '
            f'({self.x:.2f}, {self.y:.2f}, {math.degrees(self.yaw):.0f} deg)')
        self.get_logger().warn(
            'SIMULATION: this pose is not a real robot and this scan is a '
            'ray-cast of the map, not a sensor')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_cmd(self, msg):
        self.vx, self.vy, self.wz = (
            msg.linear.x, msg.linear.y, msg.angular.z)

    def request_map(self):
        """Fall back to the service if the latched topic never arrived."""
        if self.grid is not None or self.map_request_sent:
            return
        if not self.map_client.service_is_ready():
            return
        self.map_request_sent = True
        future = self.map_client.call_async(GetMap.Request())
        future.add_done_callback(self.on_map_response)
        self.get_logger().info(
            'no map on the topic yet, asking map_server for it directly')

    def on_map_response(self, future):
        try:
            self.take_map(future.result().map)
        except Exception as exc:
            self.get_logger().warn(f'map service call failed: {exc}')
            self.map_request_sent = False

    def on_map(self, msg):
        self.take_map(msg)

    def take_map(self, msg):
        self.map_info = msg.info
        self.grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        free = int((self.grid == 0).sum())
        occupied = int((self.grid >= 65).sum())
        self.get_logger().info(
            f'map received: {msg.info.width} x {msg.info.height} @ '
            f'{msg.info.resolution} m, {free} free / {occupied} occupied cells')

    def step(self):
        now = self.now()
        dt = now - self.last
        self.last = now
        if dt <= 0.0 or dt > 1.0:
            return

        # True motion.
        self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        self.yaw = math.atan2(math.sin(self.yaw + self.wz * dt),
                              math.cos(self.yaw + self.wz * dt))

        # Odometry's version of it, deliberately slightly wrong.
        d = self.drift
        self.ox += (self.vx * d * math.cos(self.oyaw)
                    - self.vy * d * math.sin(self.oyaw)) * dt
        self.oy += (self.vx * d * math.sin(self.oyaw)
                    + self.vy * d * math.cos(self.oyaw)) * dt
        self.oyaw = math.atan2(math.sin(self.oyaw + self.wz * d * dt),
                               math.cos(self.oyaw + self.wz * d * dt))

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quat_from_yaw(self.oyaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.ox
        odom.pose.pose.position.y = self.oy
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.ox
        tf.transform.translation.y = self.oy
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)

    def publish_scan(self):
        if self.grid is None:
            return

        info = self.map_info
        res = info.resolution
        # March every beam together rather than one at a time: 723 beams over
        # 20 m at 5 cm is 290k samples per scan, which pure Python cannot do at
        # 10 Hz but numpy can do in a few milliseconds.
        steps = int((self.range_max - self.range_min) / res)
        dist = self.range_min + np.arange(steps) * res
        world = self.yaw + self.angles
        # (beams, steps)
        px = self.x + np.outer(np.cos(world), dist)
        py = self.y + np.outer(np.sin(world), dist)

        col = ((px - info.origin.position.x) / res).astype(np.int32)
        row = ((py - info.origin.position.y) / res).astype(np.int32)
        inside = ((col >= 0) & (col < info.width)
                  & (row >= 0) & (row < info.height))
        np.clip(col, 0, info.width - 1, out=col)
        np.clip(row, 0, info.height - 1, out=row)

        # Only occupied cells stop a beam. Unknown does not: it means the
        # mapping run never looked there, not that something is standing there,
        # and a real laser would pass straight through and hit whatever is
        # beyond.
        #
        # Treating unknown as blocking was the first attempt and it made the
        # robot immobile. This map is about 65% unknown, so every beam
        # terminated a few centimetres out, the costmap filled with obstacles
        # at the footprint edge, and Nav2 spun in place issuing recoveries
        # while never commanding any forward velocity - which reads like a
        # controller fault rather than a scan fault.
        blocked = (self.grid[row, col] >= 65) & inside
        # Leaving the map is the end of the beam.
        blocked |= ~inside

        first = np.argmax(blocked, axis=1)
        hit = blocked.any(axis=1)
        ranges = np.where(hit, self.range_min + first * res, np.inf)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.base_frame
        scan.angle_min = float(self.angles[0])
        scan.angle_max = float(self.angles[-1])
        scan.angle_increment = float(self.angles[1] - self.angles[0])
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(scan)

    def report(self):
        if self.grid is None:
            self.get_logger().warn(
                'no map yet - is map_server running on the ground station?')
        else:
            self.get_logger().info(
                f'true ({self.x:+.2f}, {self.y:+.2f}, '
                f'{math.degrees(self.yaw):+.0f} deg)  '
                f'odom ({self.ox:+.2f}, {self.oy:+.2f})  '
                f'cmd v={self.vx:+.2f} w={self.wz:+.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = FakeBase()
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
