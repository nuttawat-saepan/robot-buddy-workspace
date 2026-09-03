import math
import struct

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class OccupancyGridPoints(Node):
    def __init__(self):
        super().__init__('occupancy_grid_points')
        self.declare_parameter('input_topic', '/map')
        self.declare_parameter('output_topic', '/map_points')
        self.declare_parameter('min_value', 50)
        self.declare_parameter('z', 0.02)
        self.declare_parameter('publish_free', False)
        self.declare_parameter('stride', 1)
        self.declare_parameter('transient_local', False)

        self.input_topic = self.get_parameter(
            'input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter(
            'output_topic').get_parameter_value().string_value
        self.min_value = self.get_parameter(
            'min_value').get_parameter_value().integer_value
        self.z = self.get_parameter('z').get_parameter_value().double_value
        self.publish_free = self.get_parameter(
            'publish_free').get_parameter_value().bool_value
        self.stride = max(1, self.get_parameter(
            'stride').get_parameter_value().integer_value)
        transient_local = self.get_parameter(
            'transient_local').get_parameter_value().bool_value

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        if transient_local:
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub = self.create_publisher(PointCloud2, self.output_topic, 1)
        self.sub = self.create_subscription(
            OccupancyGrid, self.input_topic, self.on_grid, qos)
        self.get_logger().info(
            f'{self.input_topic} -> {self.output_topic} as PointCloud2')

    def on_grid(self, msg):
        info = msg.info
        origin = info.origin.position
        yaw = _yaw_from_quaternion(info.origin.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        res = info.resolution
        width = info.width
        height = info.height

        points = []
        for y in range(0, height, self.stride):
            row = y * width
            for x in range(0, width, self.stride):
                value = msg.data[row + x]
                if value < 0:
                    continue
                if self.publish_free:
                    keep = value == 0
                else:
                    keep = value >= self.min_value
                if not keep:
                    continue

                lx = (x + 0.5) * res
                ly = (y + 0.5) * res
                wx = origin.x + cos_yaw * lx - sin_yaw * ly
                wy = origin.y + sin_yaw * lx + cos_yaw * ly
                points.append((wx, wy, self.z))

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32,
                       count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32,
                       count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32,
                       count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = b''.join(struct.pack('<fff', *point) for point in points)
        self.pub.publish(cloud)


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPoints()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
