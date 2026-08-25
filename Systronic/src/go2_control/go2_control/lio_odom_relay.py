"""Publish /odom for Nav2 and main.py from the FAST-LIO pose.

Read-only with respect to the robot. It publishes one topic, nav_msgs/Odometry
on /odom, and broadcasts no TF. Nothing here can move anything.

## Why this node has to exist

FAST-LIO publishes /Odometry as `camera_init -> body`. Every consumer in this
workspace expects `odom -> base_link`:

    nav2 controller_server   odom_topic, for velocity feedback
    nav2 bt_navigator        odom_topic
    main.py                  subscribes /odom for mission state

`camera_init` is tilted by the sensor mount and `body` is the IMU frame, so
neither the frame ids nor the pose are what those consumers mean. The TF tree
already carries the corrected chain, built by the two static bridges in
livox_amcl.launch.py:

    odom --(static, levelling)--> camera_init --(FAST-LIO)--> body
                                              --(static)--> base_link

so this node reads `odom -> base_link` straight out of TF rather than redoing
the arithmetic. One definition of the mount pose, in one place; if the bridges
are wrong, this is wrong in the same way instead of disagreeing with them.

## Why the velocity is differentiated rather than copied

Upstream FAST-LIO never fills `odomAftMapped.twist` - there is no assignment to
it anywhere in `laserMapping.cpp` - so /Odometry carries an all-zero twist.
Passing that through would leave Nav2's velocity feedback permanently reading
"stopped", which the progress checker and DWB both use. So the twist here is
differentiated from consecutive poses and expressed in base_link, which is the
convention nav_msgs/Odometry specifies for child_frame_id.

Differentiating a 10 Hz pose is noisy, hence the smoothing below. It is good
enough for feedback and is not a substitute for a real wheel or joint odometry
source, which the Go2 does publish on its own topics if that is ever wanted.

    ros2 run go2_control lio_odom_relay
"""
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_of(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LioOdomRelay(Node):

    def __init__(self):
        super().__init__('lio_odom_relay')

        self.declare_parameter('source_topic', '/Odometry')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # One pole of exponential smoothing on the differentiated velocity.
        # 0.0 passes the raw difference through, 0.9 is heavy smoothing.
        self.declare_parameter('velocity_smoothing', 0.5)
        # Covariance is not estimated. Nav2 does not read it, but a consumer
        # that does should be told the numbers are nominal rather than
        # measured, and a zero covariance would claim perfect knowledge.
        self.declare_parameter('pose_covariance', 0.05)
        self.declare_parameter('twist_covariance', 0.1)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.alpha = float(self.get_parameter('velocity_smoothing').value)
        self.pose_cov = float(self.get_parameter('pose_covariance').value)
        self.twist_cov = float(self.get_parameter('twist_covariance').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 20)
        self.create_subscription(
            Odometry, self.get_parameter('source_topic').value,
            self.on_source, 20)

        self.last = None          # (t, x, y, yaw)
        self.vx = self.vy = self.wz = 0.0
        self.published = 0
        self.lookup_failures = 0
        self.last_failure = ''
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f'relaying {self.get_parameter("source_topic").value} as '
            f'{self.get_parameter("odom_topic").value} '
            f'({self.odom_frame} -> {self.base_frame}), pose from TF')

    def on_source(self, msg):
        stamp = msg.header.stamp
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time.from_msg(stamp))
        except TransformException as exc:
            self.lookup_failures += 1
            self.last_failure = str(exc)
            return

        t = stamp.sec + stamp.nanosec * 1e-9
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_of(tf.transform.rotation)

        if self.last is not None:
            dt = t - self.last[0]
            # A repeated or out-of-order stamp would divide by ~zero and throw
            # a huge velocity into Nav2's feedback; hold the last estimate.
            if dt > 1e-3:
                dx = x - self.last[1]
                dy = y - self.last[2]
                # nav_msgs/Odometry specifies twist in child_frame_id, so the
                # world-frame difference is rotated into base_link.
                c, s = math.cos(yaw), math.sin(yaw)
                raw_vx = (dx * c + dy * s) / dt
                raw_vy = (-dx * s + dy * c) / dt
                raw_wz = wrap(yaw - self.last[3]) / dt
                a = self.alpha
                self.vx = a * self.vx + (1.0 - a) * raw_vx
                self.vy = a * self.vy + (1.0 - a) * raw_vy
                self.wz = a * self.wz + (1.0 - a) * raw_wz
        self.last = (t, x, y, yaw)

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = tf.transform.translation.z
        out.pose.pose.orientation = tf.transform.rotation
        out.twist.twist.linear.x = self.vx
        out.twist.twist.linear.y = self.vy
        out.twist.twist.angular.z = self.wz

        for i, value in ((0, self.pose_cov), (7, self.pose_cov),
                         (14, self.pose_cov), (21, self.pose_cov),
                         (28, self.pose_cov), (35, self.pose_cov)):
            out.pose.covariance[i] = value
        for i in (0, 7, 14, 21, 28, 35):
            out.twist.covariance[i] = self.twist_cov

        self.pub.publish(out)
        self.published += 1

    def report(self):
        if self.published == 0:
            if self.lookup_failures:
                self.get_logger().warn(
                    f'{self.lookup_failures} TF lookups failed, latest: '
                    f'{self.last_failure}. Is livox_amcl.launch.py running? '
                    'It owns the odom->camera_init and body->base_link '
                    'bridges this node reads through.')
            else:
                self.get_logger().warn(
                    'nothing received yet - is FAST-LIO running?')
        else:
            self.get_logger().info(
                f'{self.published} published, {self.lookup_failures} TF '
                f'failures, v=({self.vx:+.2f}, {self.vy:+.2f}) m/s '
                f'w={self.wz:+.2f} rad/s')


def main(args=None):
    rclpy.init(args=args)
    node = LioOdomRelay()
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
