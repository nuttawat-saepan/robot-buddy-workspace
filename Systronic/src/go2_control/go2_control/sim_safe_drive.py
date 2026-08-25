#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SimSafeDrive(Node):
    def __init__(self):
        super().__init__('sim_safe_drive')
        self.declare_parameter('topic', '/sim_cmd_vel')
        self.declare_parameter('linear_x', 0.08)
        self.declare_parameter('angular_z', 0.0)
        self.declare_parameter('duration', 3.0)
        self.declare_parameter('rate', 10.0)

        self.topic = self.get_parameter('topic').value
        self.linear_x = float(self.get_parameter('linear_x').value)
        self.angular_z = float(self.get_parameter('angular_z').value)
        self.duration = float(self.get_parameter('duration').value)
        rate = float(self.get_parameter('rate').value)

        self.publisher = self.create_publisher(Twist, self.topic, 10)
        self.start_time = self.get_clock().now()
        self.stop_sent = False
        self.timer = self.create_timer(1.0 / rate, self.publish_command)

        self.get_logger().info(
            f'Publishing sim-only Twist to {self.topic} for {self.duration:.1f}s'
        )

    def publish_command(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        msg = Twist()

        if elapsed <= self.duration:
            msg.linear.x = self.linear_x
            msg.angular.z = self.angular_z
            self.publisher.publish(msg)
            return

        if not self.stop_sent:
            self.publisher.publish(msg)
            self.stop_sent = True
            self.get_logger().info('Sim-only drive test complete; stop command sent')


def main():
    rclpy.init()
    node = SimSafeDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
