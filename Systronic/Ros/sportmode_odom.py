import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

# สมมุติ message จาก Go2W
from your_pkg.msg import SportModeState   # <-- แก้ตามจริง

class Go2Odom(Node):

    def __init__(self):
        super().__init__('go2_odom')

        self.sub = self.create_subscription(
            SportModeState,
            '/sportmodestate',
            self.callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def callback(self, msg):
        now = self.get_clock().now()

        # 📍 position
        x = msg.position[0]
        y = msg.position[1]

        # 🧭 yaw (สำคัญ!)
        yaw = msg.rpy[2]

        # 🚀 velocity
        vx = msg.velocity[0]
        vy = msg.velocity[1]
        wz = msg.yaw_speed   # หรือคำนวณเอง

        # quaternion
        qz = math.sin(yaw/2)
        qw = math.cos(yaw/2)

        # 🧾 ODOM
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y

        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        # 🔗 TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = Go2Odom()
    rclpy.spin(node)