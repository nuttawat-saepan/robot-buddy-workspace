#!/usr/bin/env python3
"""Record one test case and print the numbers it produced.

Read-only. It subscribes, listens to TF, and starts a bag. It creates no
publisher of any kind, so no bug in it can move the robot.

    ros2 run go2_control testcase_recorder --ros-args -p case:=03_square
    ros2 run go2_control testcase_recorder --ros-args -p case:=01_straight \\
        -p goal_x:=4.0 -p goal_y:=0.0 -p goal_yaw:=0.0

Ctrl-C ends the run, writes the bag, the CSV and a JSON summary, and prints
the summary.

## Three different errors, and only two of them can be measured from here

The distinction matters more than any single number, and confusing them is
how a test day produces figures nobody can act on.

**Goal error** is where the robot stopped against where it was asked to stop,
both in the map frame. It is the controller's and the goal checker's number.
It is measured here, and it is always smaller than xy_goal_tolerance by
construction - the goal checker stops the moment it is met - so the useful
reading is not one run but the spread over ten. That spread is the answer to
"can the tolerance come down".

**Drift correction** is how far AMCL has had to pull the odometry to keep it
on the map: the translation of map->odom. It is honest, needs no tape measure,
and is the number that says whether AMCL alone is enough on this robot. It is
measured here, and amcl_drift_check has the longer explanation of why the
naive amcl_pose-minus-odometry comparison is zero by construction.

**Localisation error** is where the robot actually is against where it thinks
it is. **It cannot be measured from inside the robot at all.** Every topic
here is downstream of the belief being checked, so a robot that is confidently
wrong reports a small number. This one comes off the floor: mark the start
with tape, mark where the robot stops, and measure between them. The scripted
numbers narrow down which runs are worth measuring by hand; they do not
replace the tape.

## Why map->base_link from TF and not /amcl_pose

AMCL publishes a pose only when it updates, and it updates after
update_min_d metres of movement - 0.20 in the config on this robot. A robot
that has stopped therefore holds a pose that can be a fifth of a metre stale,
which is the same size as the error being measured. TF is interpolated and
continuous, so the stopped pose is the current one.

For a measurement run, put update_min_d and update_min_a down to 0.05 in
amcl_livox.yaml before launching. They are not live-settable: Foxy's
nav2_amcl reads them once in on_configure, so `ros2 param set` answers
success and changes nothing.
"""
import csv
import json
import math
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

import tf2_ros

# Topics worth having afterwards. /tf_static is separate from /tf and a bag
# without it replays with no sensor mounting at all; /scan and /livox/lidar
# are best effort, which the bag records faithfully but RViz will not show
# unless its own Reliability is set to match.
BAG_TOPICS = [
    '/tf', '/tf_static', '/scan', '/map', '/amcl_pose', '/particlecloud',
    '/Odometry', '/cmd_vel', '/cmd_vel_safe', '/plan', '/local_plan',
    '/local_costmap/costmap', '/rosout',
]

# Words that appear in the log line when Nav2 falls back on a recovery. A run
# that reached its goal after three spins is not the same result as a run
# that drove there, and the difference is invisible in the final pose.
RECOVERY_WORDS = ('Running Spin', 'Running BackUp', 'Running Wait',
                  'backup', 'recovery', 'Recovery')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    """Fold an angle into (-pi, pi] so 359 degrees reads as -1, not 359."""
    return math.atan2(math.sin(a), math.cos(a))


class TestcaseRecorder(Node):

    def __init__(self):
        super().__init__('testcase_recorder')

        self.case = self.declare_parameter('case', 'unnamed').value
        out_root = os.path.expanduser(
            self.declare_parameter(
                'out_dir', '~/go2_testruns').value)
        self.rate = float(self.declare_parameter('rate', 10.0).value)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter(
            'base_frame', 'base_link').value
        self.want_bag = bool(self.declare_parameter('record_bag', True).value)
        # The goal checker's xy_goal_tolerance. Used only to decide when a
        # run counts as having arrived, for the settling figure below.
        self.declare_parameter('arrive_radius', 0.35)

        # Optional. Given, the summary carries a goal error and an overshoot;
        # absent, everything else is still measured. /goal_pose fills these in
        # when the goal came from RViz.
        self.goal = None
        gx = float(self.declare_parameter('goal_x', float('nan')).value)
        gy = float(self.declare_parameter('goal_y', float('nan')).value)
        gyaw = float(self.declare_parameter('goal_yaw', float('nan')).value)
        if not math.isnan(gx) and not math.isnan(gy):
            self.goal = (gx, gy, 0.0 if math.isnan(gyaw) else gyaw)

        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(out_root, '%s_%s' % (stamp, self.case))
        os.makedirs(self.run_dir, exist_ok=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        best = QoSProfile(depth=10)
        best.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self._on_scan, best)
        self.create_subscription(Odometry, '/Odometry', self._on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel_safe', self._on_cmd, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self.create_subscription(Log, '/rosout', self._on_log, 10)

        self.rows = []
        self.start_pose = None
        self.last_pose = None
        self.path_map = 0.0
        self.path_odom = 0.0
        self.last_odom_xy = None
        self.yaw_unwrapped = 0.0
        self.last_yaw = None
        self.min_scan = float('inf')
        self.min_scan_at = None
        self.max_lin = 0.0
        self.max_ang = 0.0
        self.recoveries = 0
        self.t0 = time.time()

        self.bag = None
        if self.want_bag:
            self._start_bag()

        self.create_timer(1.0 / self.rate, self._sample)
        self.get_logger().info(
            'recording case %r into %s - Ctrl-C to finish'
            % (self.case, self.run_dir))

    # ------------------------------------------------------------- inputs

    def _start_bag(self):
        path = os.path.join(self.run_dir, 'bag')
        try:
            self.bag = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', path] + BAG_TOPICS,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            self.get_logger().error(
                'ros2 bag not found - the run is measured but not recorded')

    def _on_scan(self, msg):
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            if r < self.min_scan:
                self.min_scan = r
                self.min_scan_at = math.degrees(
                    msg.angle_min + i * msg.angle_increment)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        if self.last_odom_xy is not None:
            self.path_odom += math.hypot(p.x - self.last_odom_xy[0],
                                         p.y - self.last_odom_xy[1])
        self.last_odom_xy = (p.x, p.y)

    def _on_cmd(self, msg):
        self.max_lin = max(self.max_lin, abs(msg.linear.x))
        self.max_ang = max(self.max_ang, abs(msg.angular.z))

    def _on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y,
                     yaw_of(msg.pose.orientation))
        self.get_logger().info('goal seen on /goal_pose: %.3f %.3f'
                               % (self.goal[0], self.goal[1]))

    def _on_log(self, msg):
        if any(w in msg.msg for w in RECOVERY_WORDS):
            self.recoveries += 1

    # ------------------------------------------------------------ sampling

    def _lookup(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time())
        except Exception:                          # noqa: BLE001
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_of(tf.transform.rotation)

    def _sample(self):
        pose = self._lookup(self.map_frame, self.base_frame)
        if pose is None:
            return
        x, y, yaw = pose

        if self.start_pose is None:
            self.start_pose = pose
            self.last_yaw = yaw
        else:
            self.path_map += math.hypot(x - self.last_pose[0],
                                        y - self.last_pose[1])
            # Unwrapped, so a full turn reads 6.28 rather than folding back
            # to zero. Case 2 is exactly the run where the folded number is
            # useless.
            self.yaw_unwrapped += wrap(yaw - self.last_yaw)
            self.last_yaw = yaw
        self.last_pose = pose

        corr = self._lookup(self.map_frame, self.odom_frame)
        corr_d = math.hypot(corr[0], corr[1]) if corr else float('nan')

        goal_d = float('nan')
        if self.goal:
            goal_d = math.hypot(x - self.goal[0], y - self.goal[1])

        self.rows.append({
            't': round(time.time() - self.t0, 3),
            'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4),
            'path_map': round(self.path_map, 4),
            'path_odom': round(self.path_odom, 4),
            'correction': round(corr_d, 4),
            'goal_dist': round(goal_d, 4),
            'min_scan': round(self.min_scan, 3),
        })

    # ------------------------------------------------------------- summary

    def finish(self):
        if self.bag is not None:
            self.bag.send_signal(signal.SIGINT)
            try:
                self.bag.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.bag.kill()

        csv_path = os.path.join(self.run_dir, 'samples.csv')
        if self.rows:
            with open(csv_path, 'w', newline='') as fh:
                w = csv.DictWriter(fh, fieldnames=list(self.rows[0]))
                w.writeheader()
                w.writerows(self.rows)

        s = {'case': self.case, 'run_dir': self.run_dir,
             'duration_s': round(time.time() - self.t0, 1),
             'samples': len(self.rows)}

        if self.start_pose and self.last_pose:
            sx, sy, syaw = self.start_pose
            ex, ey, eyaw = self.last_pose
            s['start'] = [round(v, 4) for v in (sx, sy, syaw)]
            s['end'] = [round(v, 4) for v in (ex, ey, eyaw)]
            # For any route that ends where it began - the square, the loop,
            # the 360 - this is the closing error, and it is the headline
            # number of those cases.
            s['closing_error_m'] = round(math.hypot(ex - sx, ey - sy), 4)
            s['closing_yaw_deg'] = round(math.degrees(wrap(eyaw - syaw)), 2)
            s['rotated_deg'] = round(math.degrees(self.yaw_unwrapped), 1)

        s['path_map_m'] = round(self.path_map, 3)
        s['path_odom_m'] = round(self.path_odom, 3)

        corr = [r['correction'] for r in self.rows
                if math.isfinite(r['correction'])]
        if corr:
            s['correction_mean_m'] = round(sum(corr) / len(corr), 4)
            s['correction_max_m'] = round(max(corr), 4)
            # The jump, not the mean, is what makes the controller lurch -
            # lowering AMCL's alphas halves this while leaving the mean alone.
            s['correction_max_jump_m'] = round(
                max((abs(b - a) for a, b in zip(corr, corr[1:])), default=0.0),
                4)

        if self.goal and self.last_pose:
            ex, ey, eyaw = self.last_pose
            s['goal'] = [round(v, 4) for v in self.goal]
            s['goal_error_m'] = round(
                math.hypot(ex - self.goal[0], ey - self.goal[1]), 4)
            s['goal_yaw_error_deg'] = round(
                math.degrees(wrap(eyaw - self.goal[2])), 2)
            # The furthest it got from the goal after first reaching it. A
            # run that settles at 0.20 m having first sailed out to 0.45 m is
            # not the same run as one that crept in, and the final pose hides
            # the difference. Named for what it is rather than "overshoot":
            # a robot that simply stops 0.20 m short reports 0.20, which is
            # the residual, and only a figure larger than the final distance
            # means it actually went past and came back.
            arrive = float(self.get_parameter('arrive_radius').value)
            after = None
            for i, r in enumerate(self.rows):
                if math.isfinite(r['goal_dist']) and r['goal_dist'] <= arrive:
                    after = i + 1
                    break
            s['arrived'] = after is not None
            s['max_dist_after_arrival_m'] = round(
                max((r['goal_dist'] for r in self.rows[after:]
                     if math.isfinite(r['goal_dist'])), default=0.0), 4
            ) if after is not None else None

        s['min_scan_range_m'] = (round(self.min_scan, 3)
                                 if math.isfinite(self.min_scan) else None)
        s['min_scan_bearing_deg'] = (round(self.min_scan_at, 1)
                                     if self.min_scan_at is not None else None)
        s['max_cmd_linear'] = round(self.max_lin, 3)
        s['max_cmd_angular'] = round(self.max_ang, 3)
        s['recovery_log_lines'] = self.recoveries
        # Not measurable from inside the robot. Left empty on purpose so the
        # field sheet has somewhere to put the tape measure reading.
        s['ground_truth_error_m'] = None

        with open(os.path.join(self.run_dir, 'summary.json'), 'w') as fh:
            json.dump(s, fh, indent=2)

        print('\n--- %s ---' % self.case)
        for k, v in s.items():
            print('%-24s %s' % (k, v))
        print('\nmeasure the floor and put it in ground_truth_error_m:')
        print('  %s' % os.path.join(self.run_dir, 'summary.json'))


def main(argv=None):
    rclpy.init(args=argv)
    node = TestcaseRecorder()

    # SIGTERM as well as Ctrl-C. A run ended by a launch shutdown, or by
    # timeout in a script, would otherwise lose its summary - and losing the
    # summary of a run that already happened is not recoverable.
    def _term(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
