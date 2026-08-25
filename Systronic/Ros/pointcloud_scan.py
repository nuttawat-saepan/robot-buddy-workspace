import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from pointcloud_to_laserscan import PointCloudToLaserScanNode

class CloudToScan(Node):
    def __init__(self):
        super().__init__('cloud_to_scan')

        self.declare_parameter('target_frame', 'base_link')

        self.node = PointCloudToLaserScanNode(
            node_name='cloud_to_scan_internal',
            parameters={
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': -0.3,
                'max_height': 0.3,
                'angle_min': -3.14,
                'angle_max': 3.14,
                'angle_increment': 0.01,
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 20.0,
            }
        )