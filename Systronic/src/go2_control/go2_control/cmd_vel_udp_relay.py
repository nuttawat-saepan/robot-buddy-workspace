"""Forward safe local ROS velocity commands to the Unitree-only process.

The LiDAR/Nav2 graph needs Fast DDS on loopback, while Unitree's SDK needs
CycloneDDS on Wi-Fi.  Keeping the two middleware implementations in separate
processes avoids their environment variables interfering with each other.
"""

import json
import socket

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelUdpRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_udp_relay')
        self.port = int(self.declare_parameter('port', 32123).value)
        self.input_topic = self.declare_parameter('input_topic', '/cmd_vel_safe').value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.create_subscription(Twist, self.input_topic, self.callback, 10)
        self.get_logger().info(f'Relaying {self.input_topic} to 127.0.0.1:{self.port}')

    def callback(self, msg):
        payload = json.dumps({
            'x': msg.linear.x,
            'y': msg.linear.y,
            'z': msg.angular.z,
        }).encode('ascii')
        self.sock.sendto(payload, ('127.0.0.1', self.port))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelUdpRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
