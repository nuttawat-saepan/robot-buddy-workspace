#!/usr/bin/env python3
import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class FakeHesaiXT16(Node):
    def __init__(self):
        super().__init__('fake_hesai_xt16')

        self.declare_parameter('topic', '/lidar_points')
        self.declare_parameter('frame_id', 'hesai_lidar')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('range_min', 0.8)
        self.declare_parameter('range_max', 12.0)

        self.topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.publisher = self.create_publisher(PointCloud2, self.topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_cloud)
        self.phase = 0.0

        self.get_logger().info(
            f'Fake Hesai XT16 publishing {self.topic} frame={self.frame_id}'
        )

    def publish_cloud(self):
        stamp = self.get_clock().now().to_msg()
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.frame_id

        points = self.create_xt16_points()
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 18
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = b''.join(
            struct.pack('<ffffH', x, y, z, intensity, ring)
            for x, y, z, intensity, ring in points
        )

        self.publisher.publish(cloud)
        self.phase += 0.04

    def create_xt16_points(self):
        vertical_angles = [
            -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0,
            -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0,
        ]
        points = []
        azimuth_steps = 720

        for azimuth_index in range(azimuth_steps):
            yaw = -math.pi + (2.0 * math.pi * azimuth_index / azimuth_steps)
            base_range = self.synthetic_range(yaw)

            for ring, vertical_deg in enumerate(vertical_angles):
                pitch = math.radians(vertical_deg)
                ring_offset = 0.02 * math.sin(ring + self.phase)
                distance = max(
                    self.range_min,
                    min(self.range_max, base_range + ring_offset),
                )

                horizontal = distance * math.cos(pitch)
                x = horizontal * math.cos(yaw)
                y = horizontal * math.sin(yaw)
                z = distance * math.sin(pitch)
                intensity = 80.0 + 40.0 * math.sin(yaw * 3.0 + ring)

                points.append((x, y, z, intensity, ring))

        return points

    def synthetic_range(self, yaw):
        front_wall = 5.0 + 0.2 * math.sin(4.0 * yaw + self.phase)
        side_shape = 1.2 * math.sin(2.0 * yaw)
        obstacle = 0.0

        if abs(yaw - 0.7) < 0.18:
            obstacle = -2.2
        elif abs(yaw + 1.1) < 0.15:
            obstacle = -1.6

        return front_wall + side_shape + obstacle


def main():
    rclpy.init()
    node = FakeHesaiXT16()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
