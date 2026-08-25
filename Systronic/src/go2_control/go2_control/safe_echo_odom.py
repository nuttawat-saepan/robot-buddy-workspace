#!/usr/bin/env python3
"""Small /odom echo tool that avoids the heavy generic ros2 topic echo path."""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class SafeOdomEcho(Node):
    def __init__(self) -> None:
        super().__init__("safe_odom_echo")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.create_subscription(Odometry, "/odom", self.on_odom, qos)
        self.get_logger().info("Listening to /odom with BEST_EFFORT depth=1")

    def on_odom(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        self.get_logger().info(
            f"pos=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) "
            f"quat=({ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f}) "
            f"lin=({lin.x:.3f}, {lin.y:.3f}, {lin.z:.3f}) "
            f"ang_z={ang.z:.3f}"
        )


def main() -> None:
    rclpy.init()
    node = SafeOdomEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
