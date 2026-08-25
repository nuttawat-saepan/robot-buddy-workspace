"""Measure how far AMCL has to pull FAST-LIO's odometry to match the map.

Read-only. It subscribes and listens to TF and prints numbers; it publishes
nothing at all, least of all /cmd_vel.

## What is actually being measured

The obvious reading of "how far is /amcl_pose from /Odometry" is a trap. AMCL
does not estimate a pose independently and then get compared against odometry -
it estimates the map->odom correction, and the pose it publishes is that
correction composed with the odometry:

    amcl_pose = (map->odom) * (odom->base_link)

Comparing amcl_pose against the odometry it was built from would come out as
zero by construction, for any amount of drift.

What is worth measuring is the correction itself: express the LIO pose in the
frame the map was built in, and see how far AMCL has moved it. That is the
distance the odometry has drifted away from the map, and it is exactly the
number that decides whether AMCL alone is enough on this robot or whether
AprilTag becomes a requirement.

That comparison is only meaningful because map and odom coincide at t=0: the
map was rasterised by pcd_to_map straight out of FAST-LIO's own levelled world
frame, so an undrifted run would hold the correction at zero for its whole
length. Point this at a map from a different run and the numbers mean nothing.

The odom->base_link transform is taken from TF rather than recomputed from
/Odometry on purpose. It is the same chain AMCL itself consumed - the two
static bridges in livox_amcl.launch.py either side of FAST-LIO - so a bridge
that is wired up wrongly shows up here as a large correction instead of being
silently cancelled out on both sides of the subtraction.

/Odometry is still subscribed, for the distance travelled. Drift is only
interpretable as a fraction of how far the sensor went: the 1.65 m closing
error on 02_loop is 4.72% of a 35 m walk, and it is the percentage that is
comparable with FAST-LIO's usual sub-1%.

    ros2 run go2_control amcl_drift_check
    ros2 run go2_control amcl_drift_check --ros-args -p csv_path:=/tmp/drift.csv
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_of(q):
    """Yaw from a quaternion. Roll and pitch are irrelevant to a 2D filter."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class AmclDriftCheck(Node):

    def __init__(self):
        super().__init__('amcl_drift_check')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('report_period', 5.0)
        self.declare_parameter('csv_path', '')

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.samples = []          # (t, dx, dy, dist, dyaw, path_len)
        self.lookup_failures = 0
        self.last_failure = ''
        self.path_len = 0.0
        self.last_odom_xy = None
        self.odom_count = 0
        self.first_stamp = None

        csv_path = self.get_parameter('csv_path').value
        self.csv = None
        if csv_path:
            self.csv = open(csv_path, 'w')
            self.csv.write('t,path_len,amcl_x,amcl_y,lio_x,lio_y,'
                           'dx,dy,dist,dyaw_deg\n')

        self.create_subscription(
            Odometry, '/Odometry', self.on_odom, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 20)

        self.create_timer(
            float(self.get_parameter('report_period').value), self.report)

        self.get_logger().info(
            f'comparing /amcl_pose in {self.map_frame} against '
            f'{self.odom_frame}->{self.base_frame} from TF')

    def on_odom(self, msg):
        """Accumulate distance travelled straight from FAST-LIO."""
        self.odom_count += 1
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.last_odom_xy is not None:
            self.path_len += math.hypot(xy[0] - self.last_odom_xy[0],
                                        xy[1] - self.last_odom_xy[1])
        self.last_odom_xy = xy

    def on_amcl(self, msg):
        stamp = msg.header.stamp
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time.from_msg(stamp))
        except TransformException as exc:
            self.lookup_failures += 1
            self.last_failure = str(exc)
            return

        ax = msg.pose.pose.position.x
        ay = msg.pose.pose.position.y
        ayaw = yaw_of(msg.pose.pose.orientation)

        lx = tf.transform.translation.x
        ly = tf.transform.translation.y
        lyaw = yaw_of(tf.transform.rotation)

        dx, dy = ax - lx, ay - ly
        dist = math.hypot(dx, dy)
        dyaw = wrap(ayaw - lyaw)

        t = stamp.sec + stamp.nanosec * 1e-9
        if self.first_stamp is None:
            self.first_stamp = t
        rel = t - self.first_stamp

        self.samples.append((rel, dx, dy, dist, dyaw, self.path_len))
        if self.csv:
            self.csv.write(
                f'{rel:.3f},{self.path_len:.3f},{ax:.4f},{ay:.4f},'
                f'{lx:.4f},{ly:.4f},{dx:.4f},{dy:.4f},{dist:.4f},'
                f'{math.degrees(dyaw):.3f}\n')
            self.csv.flush()

    def report(self):
        if not self.samples:
            # Distinguish the three ways this produces nothing, because they
            # need three different fixes.
            if self.odom_count == 0:
                self.get_logger().warn(
                    'no /Odometry yet - is FAST-LIO running, and is the bag '
                    'playing?')
            elif self.lookup_failures:
                self.get_logger().warn(
                    f'{self.lookup_failures} TF lookups failed, latest: '
                    f'{self.last_failure}')
            else:
                self.get_logger().warn(
                    'no /amcl_pose yet - AMCL publishes only after it has '
                    'moved update_min_d, so drive or replay a little further')
            return

        rel, _, _, _, _, _ = self.samples[-1]
        dists = [s[3] for s in self.samples]
        yaws = [abs(math.degrees(s[4])) for s in self.samples]
        current = self.samples[-1]
        pct = (100.0 * current[3] / self.path_len) if self.path_len > 1e-6 else 0.0

        self.get_logger().info(
            f't={rel:6.1f}s  walked={self.path_len:6.2f}m  '
            f'correction now={current[3]:5.3f}m ({pct:.2f}%)  '
            f'mean={sum(dists) / len(dists):5.3f}m  max={max(dists):5.3f}m  '
            f'yaw now={math.degrees(current[4]):+6.2f}deg  '
            f'n={len(self.samples)}')

    def summary(self):
        print()
        print('=== amcl_drift_check summary ===')
        print(f'/Odometry messages    {self.odom_count}')
        print(f'/amcl_pose samples    {len(self.samples)}')
        print(f'TF lookup failures    {self.lookup_failures}')
        if self.lookup_failures and self.last_failure:
            print(f'  latest failure      {self.last_failure}')
        print(f'path walked           {self.path_len:.2f} m')

        if not self.samples:
            print('no paired samples - nothing to measure')
            return

        dists = [s[3] for s in self.samples]
        yaws = [math.degrees(s[4]) for s in self.samples]
        n = len(dists)
        mean = sum(dists) / n
        rms = math.sqrt(sum(d * d for d in dists) / n)
        final = dists[-1]
        peak = max(dists)
        peak_at = self.samples[dists.index(peak)][0]

        print()
        print('AMCL correction to the FAST-LIO pose, metres:')
        print(f'  mean                {mean:.3f}')
        print(f'  rms                 {rms:.3f}')
        print(f'  max                 {peak:.3f}  at t={peak_at:.1f}s')
        print(f'  final               {final:.3f}')
        if self.path_len > 1e-6:
            print(f'  final as % of path  {100.0 * final / self.path_len:.2f}%')
            print(f'  max   as % of path  {100.0 * peak / self.path_len:.2f}%')
        print()
        print('Heading correction, degrees:')
        print(f'  mean abs            {sum(abs(y) for y in yaws) / n:.2f}')
        print(f'  max abs             {max(abs(y) for y in yaws):.2f}')
        print(f'  final               {yaws[-1]:+.2f}')

        if self.csv:
            self.csv.close()


def main(args=None):
    rclpy.init(args=args)
    node = AmclDriftCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # The summary is the entire output of a replay run, so it has to
        # survive Ctrl-C rather than being lost with the node.
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
