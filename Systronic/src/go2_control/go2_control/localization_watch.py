"""Compare AMCL with FAST-LIO, warn when they part and optionally refuse AMCL.

With guard off, the default, this only measures. It listens to TF and the
particle cloud, writes a CSV and publishes one status string for the web.

With guard on, AMCL must run with tf_broadcast false, and this node publishes
map->odom in its place. An AMCL correction beyond the limit is not used: the
last accepted one stays in force, and if AMCL stays out for hold_seconds the
status asks the mission bridge to pause. It never publishes velocity.

## What is compared

FAST-LIO is taken as good over a short distance and AMCL as the only thing tied
to the map. When a pose is set, the map->odom correction at that moment becomes
the anchor. From then on two positions are worked out for the same
odom->base_link:

    expected = (anchor map->odom)  * (odom->base_link)
    actual   = (AMCL's map->odom)  * (odom->base_link)

The distance between them is how far AMCL has moved the robot on the map beyond
what FAST-LIO measured. Comparing the map->odom translation alone would miss
this: a small yaw change in the correction moves a robot metres from the odom
origin by a long lever arm while the translation hardly changes.

The anchor is deliberately not moved by accepted updates. Comparing against the
last accepted value would let AMCL creep any distance in small steps, which is
exactly how it slides towards the table.

## What this cannot catch

A pose that was wrong from the start, because the anchor is wrong with it. And
FAST-LIO's own slow drift counts against the limit, so a long enough run stops
and asks for the pose to be set again even if AMCL did nothing wrong.
"""

import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import (
    PoseArray, PoseWithCovarianceStamped, TransformStamped)
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    """Apply 2D transform b in the frame of a. Each is (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2]))


def disagreement(anchor, correction, odom_to_base):
    """Metres and radians between expected and actual pose on the map."""
    expected = compose(anchor, odom_to_base)
    actual = compose(correction, odom_to_base)
    return (math.hypot(actual[0] - expected[0], actual[1] - expected[1]),
            wrap(actual[2] - expected[2]))


def count_clusters(points, cell=0.5, min_share=0.05):
    """Separate groups of particles, ignoring groups under min_share of them.

    Grid cells that touch, diagonals included, belong to one group. This is
    only logged for now, to learn how often the cloud splits on a real site.
    """
    if not points:
        return 0
    counts = {}
    for x, y in points:
        key = (math.floor(x / cell), math.floor(y / cell))
        counts[key] = counts.get(key, 0) + 1
    seen = set()
    groups = 0
    for start in counts:
        if start in seen:
            continue
        seen.add(start)
        stack = [start]
        size = 0
        while stack:
            cx, cy = stack.pop()
            size += counts[(cx, cy)]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    near = (cx + dx, cy + dy)
                    if near in counts and near not in seen:
                        seen.add(near)
                        stack.append(near)
        if size >= min_share * len(points):
            groups += 1
    return groups


class Gate:
    """Decides which corrections to use. Holds no ROS, so it can be tested."""

    def __init__(self, limit, hold_seconds):
        self.limit = limit
        # Below the limit by a margin, so a value sitting on it does not flip
        # the warning on and off every sample.
        self.clear_below = 0.8 * limit
        self.hold_seconds = hold_seconds
        self.reset()

    def reset(self):
        self.anchor = None
        self.accepted = None
        self.warning = False
        self.rejected_since = None
        self.distance = 0.0
        self.yaw = 0.0

    def update(self, correction, odom_to_base, now):
        if self.anchor is None:
            self.anchor = correction
        self.distance, self.yaw = disagreement(
            self.anchor, correction, odom_to_base)
        if self.distance <= self.limit:
            self.accepted = correction
            self.rejected_since = None
        elif self.rejected_since is None:
            self.rejected_since = now
        if self.distance > self.limit:
            self.warning = True
        elif self.distance < self.clear_below:
            self.warning = False

    @property
    def rejecting(self):
        return self.rejected_since is not None

    def lost(self, now):
        return (self.rejected_since is not None
                and now - self.rejected_since >= self.hold_seconds)


class LocalizationWatch(Node):

    def __init__(self):
        super().__init__('localization_watch')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('period', 0.5)
        self.declare_parameter('distance_limit', 1.0)
        self.declare_parameter('hold_seconds', 3.0)
        self.declare_parameter('guard', False)
        self.declare_parameter('tf_future_seconds', 0.5)
        self.declare_parameter('csv_dir', os.path.expanduser('~/go2_logs'))

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.guard = bool(self.get_parameter('guard').value)
        self.limit = float(self.get_parameter('distance_limit').value)
        self.tf_future = Duration(
            seconds=float(self.get_parameter('tf_future_seconds').value))
        self.gate = Gate(self.limit,
                         float(self.get_parameter('hold_seconds').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.spread = None
        self.clusters = None
        self.lookup_failures = 0
        self.was_warning = False
        self.was_lost = False

        csv_dir = self.get_parameter('csv_dir').value
        os.makedirs(csv_dir, exist_ok=True)
        # The board's clock can read 1970 until it syncs, so the process id
        # keeps two runs of the same "time" apart.
        path = os.path.join(csv_dir, 'localization_watch_%s_%d.csv' % (
            time.strftime('%Y%m%d_%H%M%S'), os.getpid()))
        self.csv = open(path, 'w')
        self.csv.write('wall_time,map_x,map_y,map_yaw_deg,distance_m,yaw_deg,'
                       'spread_m,clusters,warning,rejecting,paused\n')
        self.csv.flush()

        self.status_pub = self.create_publisher(
            String, '/localization_watch/status', 10)
        # Best effort, matching AMCL - a reliable subscription receives nothing.
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            PoseArray, '/particlecloud', self.on_particles, qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, 10)
        if self.guard:
            # The seed AMCL itself starts from. Nav2 will not finish coming up
            # without a map frame, and AMCL may publish no pose until the robot
            # moves or a pose is set.
            self.gate.accepted = (0.0, 0.0, 0.0)
            self.tf_broadcaster = TransformBroadcaster(self)
            self.create_subscription(
                PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, 10)
            self.create_timer(0.05, self.publish_correction)
        self.create_timer(
            float(self.get_parameter('period').value), self.on_timer)
        self.get_logger().info(
            '%s AMCL against FAST-LIO, limit %.2f m, csv %s' % (
                'guarding' if self.guard else 'watching', self.limit, path))

    def on_particles(self, msg):
        n = len(msg.poses)
        if n < 2:
            self.spread = None
            self.clusters = None
            return
        points = [(p.position.x, p.position.y) for p in msg.poses]
        mx = sum(p[0] for p in points) / n
        my = sum(p[1] for p in points) / n
        self.spread = math.sqrt(
            sum((x - mx) ** 2 + (y - my) ** 2 for x, y in points) / n)
        self.clusters = count_clusters(points)

    def on_initial_pose(self, _msg):
        # The operator has said where the robot is, so the next correction is
        # the new anchor whatever it is. map->odom holds its last value until
        # AMCL answers.
        accepted = self.gate.accepted
        self.gate.reset()
        self.gate.accepted = accepted
        self.was_warning = False
        self.was_lost = False
        self.get_logger().info('new pose set - anchor will be taken again')

    def lookup(self, parent, child, stamp=None):
        tf = self.tf_buffer.lookup_transform(
            parent, child, rclpy.time.Time() if stamp is None else stamp)
        t = tf.transform.translation
        return (t.x, t.y, yaw_of(tf.transform.rotation))

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_amcl_pose(self, msg):
        # The odometry AMCL used for this pose, not the newest: pairing the
        # pose with a later odometry would read the robot's own motion as a
        # correction.
        p = msg.pose.pose
        try:
            odom_to_base = self.lookup(
                self.odom_frame, self.base_frame,
                rclpy.time.Time.from_msg(msg.header.stamp))
        except TransformException:
            try:
                odom_to_base = self.lookup(self.odom_frame, self.base_frame)
            except TransformException as exc:
                self.note_lookup_failure(exc)
                return
        pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        correction = compose(pose, inverse(odom_to_base))
        self.gate.update(correction, odom_to_base, self.now_seconds())

    def publish_correction(self):
        accepted = self.gate.accepted
        if accepted is None:
            return
        msg = TransformStamped()
        # Post-dated as AMCL does, because the transform is only sent when it
        # is republished and consumers look it up at their own, later time.
        msg.header.stamp = (self.get_clock().now() + self.tf_future).to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x = accepted[0]
        msg.transform.translation.y = accepted[1]
        msg.transform.rotation.z = math.sin(accepted[2] / 2.0)
        msg.transform.rotation.w = math.cos(accepted[2] / 2.0)
        self.tf_broadcaster.sendTransform(msg)

    def note_lookup_failure(self, exc):
        # Every tenth failure: no samples and no message would look like a
        # healthy watcher on site.
        self.lookup_failures += 1
        if self.lookup_failures % 10 == 1:
            self.get_logger().warn(
                'no transform to compare, %d so far: %s'
                % (self.lookup_failures, exc))

    def on_timer(self):
        try:
            odom_to_base = self.lookup(self.odom_frame, self.base_frame)
            if not self.guard:
                self.gate.update(
                    self.lookup(self.map_frame, self.odom_frame),
                    odom_to_base, self.now_seconds())
        except TransformException as exc:
            self.note_lookup_failure(exc)
            return
        gate = self.gate
        if gate.accepted is None:
            return

        pause = self.guard and gate.lost(self.now_seconds())
        if gate.warning and not self.was_warning:
            self.get_logger().warn(
                'AMCL is %.2f m from where FAST-LIO puts the robot '
                '(limit %.2f m) - %s' % (
                    gate.distance, self.limit,
                    'not using it' if self.guard
                    else 'check the robot against the map'))
        elif self.was_warning and not gate.warning:
            self.get_logger().info(
                'AMCL back to %.2f m from FAST-LIO' % gate.distance)
        if pause and not self.was_lost:
            self.get_logger().error(
                'AMCL has stayed out for %.1f s - asking to pause the mission'
                % gate.hold_seconds)
        self.was_warning = gate.warning
        self.was_lost = pause

        pose = compose(gate.accepted, odom_to_base)
        self.csv.write('%.3f,%.3f,%.3f,%.2f,%.3f,%.2f,%s,%s,%d,%d,%d\n' % (
            time.time(), pose[0], pose[1], math.degrees(pose[2]),
            gate.distance, math.degrees(gate.yaw),
            '' if self.spread is None else '%.3f' % self.spread,
            '' if self.clusters is None else self.clusters,
            gate.warning, gate.rejecting, pause))
        self.csv.flush()

        msg = String()
        msg.data = json.dumps({
            'ok': not gate.warning, 'distance': round(gate.distance, 3),
            'yawDeg': round(math.degrees(gate.yaw), 1), 'limit': self.limit,
            'guard': self.guard, 'pause': pause, 'clusters': self.clusters})
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.csv.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
