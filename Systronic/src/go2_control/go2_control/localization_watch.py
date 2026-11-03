"""Watch whether AMCL still agrees with FAST-LIO, and say so when it stops.

Read-only for the robot. It listens to TF and the particle cloud, writes a CSV
and publishes one status string for the web. Nothing here changes the pose,
pauses a mission or touches /cmd_vel: this is the measuring step that has to
come before any of those, so the limit can be chosen from real runs.

## What is compared

FAST-LIO is taken as good over a short distance and AMCL as the only thing tied
to the map. When a pose is set, the map->odom correction at that moment is kept
as the baseline. From then on two positions are worked out for the same
odom->base_link:

    expected = (baseline map->odom) * (odom->base_link)
    actual   = (current  map->odom) * (odom->base_link)

The distance between them is how far AMCL has moved the robot on the map beyond
what FAST-LIO measured. Comparing the map->odom translation alone would miss
this: a small yaw change in the correction moves a robot metres from the odom
origin by a long lever arm while the translation hardly changes.

A pose that was wrong from the start cannot be seen here, because the baseline
is wrong with it. FAST-LIO's own slow drift also counts against the limit, so a
long run can warn without AMCL having done anything wrong.
"""

import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformException
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


def disagreement(baseline, current, odom_to_base):
    """Metres and radians between expected and actual pose on the map."""
    expected = compose(baseline, odom_to_base)
    actual = compose(current, odom_to_base)
    return (math.hypot(actual[0] - expected[0], actual[1] - expected[1]),
            wrap(actual[2] - expected[2]))


class LocalizationWatch(Node):

    def __init__(self):
        super().__init__('localization_watch')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('period', 0.5)
        self.declare_parameter('distance_limit', 1.0)
        self.declare_parameter('csv_dir', os.path.expanduser('~/go2_logs'))

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.limit = float(self.get_parameter('distance_limit').value)
        # Below the limit by a margin, so a value sitting on it does not flip
        # the warning on and off every sample.
        self.clear_below = 0.8 * self.limit

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.baseline = None
        self.warning = False
        self.spread = None
        self.lookup_failures = 0

        csv_dir = self.get_parameter('csv_dir').value
        os.makedirs(csv_dir, exist_ok=True)
        # The board's clock can read 1970 until it syncs, so the process id
        # keeps two runs of the same "time" apart.
        path = os.path.join(csv_dir, 'localization_watch_%s_%d.csv' % (
            time.strftime('%Y%m%d_%H%M%S'), os.getpid()))
        self.csv = open(path, 'w')
        self.csv.write('wall_time,map_x,map_y,map_yaw_deg,distance_m,'
                       'yaw_deg,spread_m,warning\n')
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
        self.create_timer(
            float(self.get_parameter('period').value), self.on_timer)
        self.get_logger().info(
            'watching AMCL against FAST-LIO, warning above %.2f m, csv %s'
            % (self.limit, path))

    def on_particles(self, msg):
        n = len(msg.poses)
        if n < 2:
            self.spread = None
            return
        xs = [p.position.x for p in msg.poses]
        ys = [p.position.y for p in msg.poses]
        mx, my = sum(xs) / n, sum(ys) / n
        self.spread = math.sqrt(
            sum((x - mx) ** 2 + (y - my) ** 2 for x, y in zip(xs, ys)) / n)

    def on_initial_pose(self, _msg):
        # AMCL has not applied it yet, so take the new baseline on the next
        # sample. The old one no longer means anything either way.
        self.baseline = None
        self.warning = False
        self.get_logger().info('new pose set - baseline will be taken again')

    def lookup(self, parent, child):
        tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        t = tf.transform.translation
        return (t.x, t.y, yaw_of(tf.transform.rotation))

    def on_timer(self):
        try:
            current = self.lookup(self.map_frame, self.odom_frame)
            odom_to_base = self.lookup(self.odom_frame, self.base_frame)
        except TransformException as exc:
            # Every tenth failure: no samples and no message would look like a
            # healthy watcher on site.
            self.lookup_failures += 1
            if self.lookup_failures % 10 == 1:
                self.get_logger().warn(
                    'no transform to compare, %d so far: %s'
                    % (self.lookup_failures, exc))
            return
        if self.baseline is None:
            self.baseline = current

        distance, dyaw = disagreement(self.baseline, current, odom_to_base)
        if not self.warning and distance > self.limit:
            self.warning = True
            self.get_logger().warn(
                'AMCL is %.2f m from where FAST-LIO puts the robot '
                '(limit %.2f m) - check the robot against the map'
                % (distance, self.limit))
        elif self.warning and distance < self.clear_below:
            self.warning = False
            self.get_logger().info(
                'AMCL back to %.2f m from FAST-LIO' % distance)

        pose = compose(current, odom_to_base)
        self.csv.write('%.3f,%.3f,%.3f,%.2f,%.3f,%.2f,%s,%d\n' % (
            time.time(), pose[0], pose[1], math.degrees(pose[2]), distance,
            math.degrees(dyaw),
            '' if self.spread is None else '%.3f' % self.spread,
            self.warning))
        self.csv.flush()

        msg = String()
        msg.data = json.dumps({
            'ok': not self.warning, 'distance': round(distance, 3),
            'yawDeg': round(math.degrees(dyaw), 1), 'limit': self.limit})
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
