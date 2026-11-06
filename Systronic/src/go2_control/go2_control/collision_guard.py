"""Stop a velocity command that would take the body into something close.

Inflation only makes a path expensive, and Nav2's own collision check works
from a scan that cannot see within 0.40 m of the robot's centre. Neither of
them stops the robot. This does, and it is the last thing before the relay:

    Nav2 -> sensor_watchdog -> /cmd_vel_safe -> this node -> /cmd_vel_guarded
         -> cmd_vel_udp_relay -> unitree_udp_bridge

It reads the raw /livox/lidar rather than /scan, for three reasons. The raw
cloud reaches 0.1 m from the sensor; it does not depend on FAST-LIO, so it keeps
working when FAST-LIO has run away; and leaving /scan alone leaves AMCL and the
costmap exactly as they were tuned.

If this node dies nothing reaches the relay, and the bridge stops the robot on
its own command timeout. See collision_zone.py for what counts as in the way.

    ros2 run go2_control collision_guard
"""
import json
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from go2_control.collision_zone import (
    Geometry, GuardState, obstacle_points, to_base)


def cloud_xyz(msg):
    """x, y, z of a PointCloud2 as an Nx3 float array, without a Python loop.

    The 20,000 points of a Mid-360 frame arrive ten times a second, and
    sensor_msgs_py is not available on Foxy in any case.
    """
    offsets = {field.name: field.offset for field in msg.fields}
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': ['<f4', '<f4', '<f4'],
        'offsets': [offsets['x'], offsets['y'], offsets['z']],
        'itemsize': msg.point_step,
    })
    count = msg.width * msg.height
    points = np.frombuffer(bytes(msg.data), dtype=dtype, count=count)
    xyz = np.stack([points['x'], points['y'], points['z']], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)
               & (np.abs(xyz).sum(axis=1) > 1e-3)]


class CollisionGuard(Node):

    def __init__(self):
        super().__init__('collision_guard')
        sensor_x = float(os.environ.get('LIO_SENSOR_X', '0.0') or 0.0)
        declare = self.declare_parameter
        self.input_topic = declare('input_topic', '/cmd_vel_safe').value
        self.output_topic = declare('output_topic', '/cmd_vel_guarded').value
        cloud_topic = declare('cloud_topic', '/livox/lidar').value
        self.pitch = float(declare('mount_pitch_deg', 13.0).value)
        self.roll = float(declare('mount_roll_deg', 0.0).value)
        self.sensor_height = float(declare('sensor_height', 0.35).value)
        self.sensor_x = float(declare('sensor_x', sensor_x).value)
        geometry = Geometry(
            body_x=float(declare('body_x', 0.35).value),
            body_y=float(declare('body_y', 0.25).value),
            stop_x=float(declare('stop_x', 0.55).value),
            stop_y=float(declare('stop_y', 0.45).value),
            strip_y=float(declare('strip_y', 0.30).value),
            min_z=float(declare('min_height', 0.25).value),
            max_z=float(declare('max_height', 1.05).value),
        )
        self.state = GuardState(
            geometry,
            min_points=int(declare('min_points', 5).value),
            release_seconds=float(declare('release_seconds', 0.5).value),
            frame_timeout=float(declare('frame_timeout', 0.30).value))

        # Same reasoning as sensor_watchdog: this cannot run on a clock that
        # the failure it guards against can stop.
        if self.get_parameter('use_sim_time').value:
            self.get_logger().error(
                'use_sim_time is true on collision_guard. Its timer stops '
                'with /clock. Start it with use_sim_time:=false.')

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.status_pub = self.create_publisher(
            String, '/collision_guard/status', 10)
        self.create_subscription(
            Twist, self.input_topic, self.on_cmd, 10)
        self.create_subscription(
            PointCloud2, cloud_topic, self.on_cloud, qos_profile_sensor_data)

        self.last_cmd = None
        self.last_cmd_at = None
        self.stopped_because = None
        self.passed = 0
        self.stopped = 0
        self.frame_ms = 0.0
        self.create_timer(0.5, self.report)
        self.get_logger().info(
            f'{self.input_topic} -> {self.output_topic}, stopping at '
            f'+-{geometry.stop_x} long and +-{geometry.stop_y} wide; mount '
            f'pitch {self.pitch} roll {self.roll} height '
            f'{self.sensor_height} forward {self.sensor_x}')
        if abs(self.sensor_x) < 1e-6:
            self.get_logger().warn(
                'sensor_x is 0: LIO_SENSOR_X is not set, so the stop line is '
                'measured from the sensor and not the middle of the robot')

    def on_cloud(self, msg):
        started = time.monotonic()
        base = to_base(cloud_xyz(msg), self.pitch, self.roll,
                       self.sensor_x, self.sensor_height)
        self.state.update(obstacle_points(base, self.state.geometry), started)
        self.frame_ms = (time.monotonic() - started) * 1000.0

        # Do not leave a moving command standing until the next one arrives.
        if self.last_cmd is not None and started - self.last_cmd_at < 0.3:
            why = self.state.refusal(self.last_cmd.linear.x,
                                     self.last_cmd.angular.z, started)
            if why:
                self.stop(why)

    def on_cmd(self, msg):
        now = time.monotonic()
        self.last_cmd = msg
        self.last_cmd_at = now
        why = self.state.refusal(msg.linear.x, msg.angular.z, now)
        if why:
            self.stop(why)
            return
        if self.stopped_because is not None:
            self.get_logger().info('clear, commands passing again')
            self.stopped_because = None
        self.pub.publish(msg)
        self.passed += 1

    def stop(self, why):
        self.pub.publish(Twist())
        self.stopped += 1
        if why != self.stopped_because:
            self.get_logger().warn(f'STOP: {why}')
            self.stopped_because = why

    def report(self):
        status = {
            'stopped': self.stopped_because,
            'counts': self.state.counts,
            'passed': self.passed,
            'stopCommands': self.stopped,
            'frameMs': round(self.frame_ms, 1),
        }
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = CollisionGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()


if __name__ == '__main__':
    main()
