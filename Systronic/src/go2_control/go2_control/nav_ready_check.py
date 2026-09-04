"""Answer 'is Nav2 actually ready to be given a goal' without opening RViz.

Read-only. It subscribes and listens to TF. It publishes nothing at all, least
of all /cmd_vel, and there is no path from this node to robot motion.

The four things that have to hold before a goal means anything are normally
checked by eye in RViz: the particle cloud has tightened, the transform tree
is joined up, the local costmap has something in it, and the controller is
running. On the board there is no RViz, and on site there is rarely time to
squint at three displays, so each one is measured here and printed as a line
that says PASS or FAIL and why.

    ros2 run go2_control nav_ready_check

## What each check means

particle spread  Standard deviation of /particlecloud positions. AMCL starts
                 with the cloud spread over the initial covariance and pulls
                 it in as scans agree with the map. Tight means it has
                 committed to a pose; wide means it has not, and a goal given
                 now is a goal in the wrong frame of reference. Note that
                 tight and *wrong* is also possible - this check cannot see
                 that, which is what amcl_drift_check is for.

tf map->odom     Published by AMCL. Missing means localisation is not running
                 or has never had enough to publish.

tf odom->base    Published by lio_odom_relay out of FAST-LIO. Missing means
                 odometry is not reaching TF, and Nav2 will not plan at all.

local costmap    Occupied cells in /local_costmap/costmap. Zero means the
                 costmap is receiving no scan, so the robot is blind to
                 anything not already in the map. An empty room legitimately
                 reads zero, which is why the count is printed rather than
                 just a verdict.

controller       Publisher count on the controller's velocity topic. Nav2's
                 controller_server and its three recovery behaviours each
                 create one, so four is the healthy number and zero means the
                 controller server is not active however green the lifecycle
                 manager's log looked.
"""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from tf2_ros import Buffer, TransformListener

# Above this the cloud is still hunting rather than tracking. AMCL on the
# 02_loop replay settles well below it within a few metres of walking.
SPREAD_LIMIT_M = 0.35


class NavReadyCheck(Node):

    def __init__(self, node_name='nav_ready_check', periodic=True):
        super().__init__(node_name)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.cmd_topic = self.declare_parameter(
            'controller_cmd_topic', '/cmd_vel_nav_preview').value
        costmap_topic = self.declare_parameter(
            'costmap_topic', '/local_costmap/costmap').value
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.period = self.declare_parameter('report_period', 3.0).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 publishes the particle cloud with SensorDataQoS - best effort.
        # A default reliable subscription is incompatible with it and receives
        # nothing at all, silently, which looks exactly like AMCL not running.
        self.spread = None
        self.particles = 0
        self.create_subscription(
            PoseArray, '/particlecloud', self.on_particles,
            qos_profile_sensor_data)

        # The costmap is latched, so a plain subscription can sit empty forever
        # while the costmap is perfectly healthy.
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.occupied = None
        self.costmap_cells = 0
        self.create_subscription(
            OccupancyGrid, costmap_topic, self.on_costmap, latched)

        if periodic:
            self.create_timer(self.period, self.report)
            self.get_logger().info(
                f'checking every {self.period:.0f}s - '
                f'nothing is published by this node')

    def on_particles(self, msg):
        n = len(msg.poses)
        self.particles = n
        if n < 2:
            self.spread = None
            return
        xs = [p.position.x for p in msg.poses]
        ys = [p.position.y for p in msg.poses]
        mx, my = sum(xs) / n, sum(ys) / n
        var = sum((x - mx) ** 2 + (y - my) ** 2 for x, y in zip(xs, ys)) / n
        self.spread = math.sqrt(var)

    def on_costmap(self, msg):
        self.occupied = sum(1 for value in msg.data if value >= 50)
        self.costmap_cells = len(msg.data)

    def has_tf(self, parent, child):
        try:
            self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time())
            return True, ''
        except Exception as exc:                  # noqa: BLE001 - report the text
            return False, str(exc).split('\n')[0]

    def readiness(self):
        """Return (ok, [(label, verdict, detail), ...]).

        Separated from report() so send_mission can refuse to send a goal on the
        same evidence a human would refuse on, rather than duplicating the
        thresholds and getting them subtly different.
        """
        lines = []
        ok = True

        if self.spread is None:
            ok = False
            lines.append(('particle spread', 'FAIL', 'no /particlecloud yet'))
        elif self.spread <= SPREAD_LIMIT_M:
            lines.append(('particle spread', 'PASS',
                          f'{self.spread:.3f} m over {self.particles} particles'))
        else:
            ok = False
            lines.append(('particle spread', 'FAIL',
                          f'{self.spread:.3f} m, above {SPREAD_LIMIT_M} m - '
                          f'AMCL has not settled'))

        for label, parent, child in (
                ('tf map->odom', self.map_frame, self.odom_frame),
                ('tf odom->base', self.odom_frame, self.base_frame)):
            good, why = self.has_tf(parent, child)
            if good:
                lines.append((label, 'PASS', ''))
            else:
                ok = False
                lines.append((label, 'FAIL', why))

        if self.occupied is None:
            ok = False
            lines.append(('local costmap', 'FAIL', 'no costmap message yet'))
        elif self.occupied == 0:
            lines.append(('local costmap', 'WARN',
                          f'0 of {self.costmap_cells} cells occupied - either '
                          f'genuinely clear, or /scan is not reaching it'))
        else:
            lines.append(('local costmap', 'PASS',
                          f'{self.occupied} of {self.costmap_cells} cells occupied'))

        scans = self.count_publishers(self.scan_topic)
        if scans:
            lines.append(('scan source', 'PASS',
                          f'{scans} publisher(s) on {self.scan_topic}'))
        else:
            ok = False
            lines.append(('scan source', 'FAIL',
                          f'nothing publishes {self.scan_topic}, so the local '
                          f'costmap can never see an obstacle'))

        publishers = self.count_publishers(self.cmd_topic)
        if publishers >= 1:
            lines.append(('controller', 'PASS',
                          f'{publishers} publisher(s) on {self.cmd_topic}'))
        else:
            ok = False
            lines.append(('controller', 'FAIL',
                          f'nothing publishes {self.cmd_topic} - '
                          f'controller_server is not active'))

        return ok, lines

    def report(self):
        ok, lines = self.readiness()
        print()
        for label, verdict, detail in lines:
            print(f'  {label:16} {verdict:5} {detail}')
        print(f'  {"":16} {"READY" if ok else "NOT READY"}'
              f'{"" if ok else " - do not send a goal"}')
        sys.stdout.flush()
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = NavReadyCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
