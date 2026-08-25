import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


class ScanToPointCloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')
        self.input_topic = self.declare_parameter('input_topic', '/scan').value
        self.output_topic = self.declare_parameter('output_topic', '/utlidar/cloud').value
        self.z = float(self.declare_parameter('z', 0.05).value)

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.create_subscription(LaserScan, self.input_topic, self.scan_cb, 10)
        self.get_logger().info(
            f'Scan to PointCloud bridge: {self.input_topic} -> {self.output_topic}'
        )

    def scan_cb(self, scan):
        points = []
        angle = scan.angle_min
        for range_value in scan.ranges:
            if math.isfinite(range_value) and scan.range_min <= range_value <= scan.range_max:
                points.append((
                    range_value * math.cos(angle),
                    range_value * math.sin(angle),
                    self.z,
                ))
            angle += scan.angle_increment

        cloud = PointCloud2()
        cloud.header = scan.header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = struct.pack('<' + 'fff' * len(points), *sum(points, ()))
        self.publisher.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
