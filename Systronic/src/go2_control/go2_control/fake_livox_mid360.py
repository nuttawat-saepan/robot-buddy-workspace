#!/usr/bin/env python3
"""Fake Livox Mid-360 for local rehearsal. No hardware, no robot motion."""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2, PointField

# livox_ros_driver2 LivoxPointXyzrtlt layout (xfer_format:=0).
POINT_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp'],
    'formats': ['<f4', '<f4', '<f4', '<f4', 'u1', 'u1', '<f8'],
    'offsets': [0, 4, 8, 12, 16, 17, 18],
    'itemsize': 26,
})

POINT_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='tag', offset=16, datatype=PointField.UINT8, count=1),
    PointField(name='line', offset=17, datatype=PointField.UINT8, count=1),
    PointField(name='timestamp', offset=18, datatype=PointField.FLOAT64, count=1),
]


class FakeLivoxMid360(Node):
    def __init__(self):
        super().__init__('fake_livox_mid360')

        self.declare_parameter('topic', '/livox/lidar')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('frame_id', 'livox_frame')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('imu_rate', 200.0)
        self.declare_parameter('points_per_frame', 20000)
        self.declare_parameter('elevation_min_deg', -7.0)
        self.declare_parameter('elevation_max_deg', 52.0)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 40.0)
        self.declare_parameter('publish_imu', True)

        self.frame_id = self.get_parameter('frame_id').value
        self.points_per_frame = int(self.get_parameter('points_per_frame').value)
        self.el_min = math.radians(float(self.get_parameter('elevation_min_deg').value))
        self.el_max = math.radians(float(self.get_parameter('elevation_max_deg').value))
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        imu_rate = float(self.get_parameter('imu_rate').value)

        self.frame_period = 1.0 / publish_rate
        self.elapsed = 0.0

        self.cloud_pub = self.create_publisher(
            PointCloud2, self.get_parameter('topic').value, 10)
        self.create_timer(self.frame_period, self.publish_cloud)

        if self.get_parameter('publish_imu').value:
            self.imu_pub = self.create_publisher(
                Imu, self.get_parameter('imu_topic').value, 50)
            self.create_timer(1.0 / imu_rate, self.publish_imu)

        self.get_logger().info(
            f'Fake Livox Mid-360 on {self.get_parameter("topic").value} '
            f'frame={self.frame_id} at {publish_rate} Hz, '
            f'{self.points_per_frame} pts/frame'
        )

    def publish_cloud(self):
        now = self.get_clock().now()
        # Sample times spread across the frame so per-point timestamps and the
        # scan pattern advance together, as they do on real hardware.
        t = self.elapsed + np.linspace(
            0.0, self.frame_period, self.points_per_frame, endpoint=False)

        azimuth, elevation = self.scan_pattern(t)
        distance = self.synthetic_range(azimuth, elevation)

        horizontal = distance * np.cos(elevation)
        cloud = np.empty(self.points_per_frame, dtype=POINT_DTYPE)
        cloud['x'] = horizontal * np.cos(azimuth)
        cloud['y'] = horizontal * np.sin(azimuth)
        cloud['z'] = distance * np.sin(elevation)
        cloud['intensity'] = 80.0 + 40.0 * np.sin(3.0 * azimuth)
        cloud['tag'] = 0
        # Mid-360 reports 4 laser lines.
        cloud['line'] = (np.arange(self.points_per_frame) % 4).astype(np.uint8)
        # Driver fills this from offset_time, which is nanoseconds. FAST-LIO's
        # timestamp_unit must agree with this.
        cloud['timestamp'] = (t - self.elapsed) * 1e9

        msg = PointCloud2()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = self.points_per_frame
        msg.fields = POINT_FIELDS
        msg.is_bigendian = False
        msg.point_step = POINT_DTYPE.itemsize
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = cloud.tobytes()

        self.cloud_pub.publish(msg)
        self.elapsed += self.frame_period

    def scan_pattern(self, t):
        """Risley-prism style non-repetitive pattern.

        The two rates are deliberately incommensurate so successive frames never
        sample the same directions. This is the property that breaks a thin
        pointcloud_to_laserscan height slice, so the rehearsal must reproduce it
        rather than fake a clean spinning ring.
        """
        spin_hz = 47.0
        nod_hz = spin_hz * 0.6180339887

        azimuth = 2.0 * math.pi * spin_hz * t
        el_center = 0.5 * (self.el_max + self.el_min)
        el_half = 0.5 * (self.el_max - self.el_min)
        elevation = el_center + el_half * np.sin(2.0 * math.pi * nod_hz * t)
        return np.mod(azimuth + math.pi, 2.0 * math.pi) - math.pi, elevation

    def synthetic_range(self, azimuth, elevation):
        room = 5.0 + 1.2 * np.sin(2.0 * azimuth)
        obstacle = np.where(np.abs(azimuth - 0.7) < 0.18, -2.2, 0.0)
        obstacle += np.where(np.abs(azimuth + 1.1) < 0.15, -1.6, 0.0)
        wall = room + obstacle

        # Upward rays hit the ceiling instead of the wall.
        ceiling_height = 2.4
        with np.errstate(divide='ignore', invalid='ignore'):
            ceiling = np.where(
                elevation > 0.05, ceiling_height / np.sin(elevation), np.inf)

        distance = np.minimum(wall, ceiling)
        return np.clip(distance, self.range_min, self.range_max)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance[0] = -1.0
        msg.linear_acceleration.z = 9.81
        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    node = FakeLivoxMid360()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
