#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from unitree_go.msg import LowState, SportModeState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, BatteryState, JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from go2_interfaces.msg import MotorTemperature
from tf2_ros import TransformBroadcaster

# Motor index order
MOTOR_INDICES = (3, 4, 5, 13, 0, 1, 2, 12, 9, 10, 11, 15, 6, 7, 8, 14)

JOINT_SUFFIXES = (
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 'FL_foot_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 'FR_foot_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint', 'RL_foot_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint', 'RR_foot_joint',
)


class RobotRead(Node):

    def __init__(self):
        super().__init__('robot_read_node')

        # ===== Names =====
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        
        self._joint_names = list(JOINT_SUFFIXES)
        self._motor_names = list(JOINT_SUFFIXES)

        self.sim_mode = self.declare_parameter('sim_mode', False).value

        # Odometry ownership. The Go2's own leg odometry and the Livox
        # FAST-LIO path both describe odom -> base_link, and two publishers of
        # one transform break the TF tree silently: tf2 keeps whichever arrived
        # last, so the robot pose flips between the two sources with nothing
        # logged to say why. /odom has the same problem, with Nav2 and main.py
        # receiving an interleaved mixture.
        #
        # Exactly one source must own each. On the Livox stack FAST-LIO owns
        # both, through lio_odom_relay and the static bridges in
        # livox_amcl.launch.py, so this node is started with both false and
        # keeps publishing only what nothing else provides: imu, battery,
        # joint states, motor temperatures and the Unitree lidar.
        #
        # Defaults stay true so the existing Hesai and simulation paths, which
        # have no other odometry source, are unaffected.
        self.publish_odom = self.declare_parameter('publish_odom', True).value
        self.publish_tf = self.declare_parameter('publish_tf', True).value

        # ===== Messages =====
        self._imu_msg = Imu()

        self._imu_temp_msg = Float32()

        self._odom_msg = Odometry()
        


        self._tf_msg = TransformStamped()
        self._battery_msg = BatteryState()
        self._battery_msg.header.frame_id = 'base_link'

        self._joint_state_msg = JointState()
        self._joint_state_msg.name = self._joint_names

        self._motor_temp_msg = MotorTemperature()
        self._motor_temp_msg.motor_name = self._motor_names

        if self.sim_mode:
            self._imu_msg.header.frame_id = 'imu_link'
            self._odom_msg.header.frame_id = 'odom'
            self._odom_msg.child_frame_id = 'base_footprint'
            self._tf_msg.header.frame_id = 'odom'
            self._tf_msg.child_frame_id = 'base_footprint'
        else:
            self._imu_msg.header.frame_id = 'imu'
            self._odom_msg.header.frame_id = 'odom'
            self._odom_msg.child_frame_id = 'base_link'
            self._tf_msg.header.frame_id = 'odom'
            self._tf_msg.child_frame_id = 'base_link'

        # ===== TF =====
        self.tf_broadcaster = TransformBroadcaster(self)

        # ===== Subscribers (native ROS2) =====
        self.create_subscription(
            SportModeState,
            '/lf/sportmodestate',
            self._sport_callback,
            qos_best_effort
        )

        self.create_subscription(
            LowState,
            '/lf/lowstate',
            self.low_state_callback,
            qos_best_effort
        )

        self.create_subscription(
            PointCloud2,
            '/utlidar/cloud',
            self.lidar_callback,
            qos_best_effort
        )

        # ===== Publishers =====
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, 'pointcloud', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 1)
        self.imu_temp_pub = self.create_publisher(Float32, 'imu_temp', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.motor_temp_pub = self.create_publisher(MotorTemperature, 'motor_temp', 10)

        self.get_logger().info(" RobotRead (ROS2 native) ready")

    # ================= SPORT =================
    def _sport_callback(self, msg):
        stamp = self.get_clock().now().to_msg()

        # IMU
        imu = self._imu_msg
        imu.header.stamp = stamp
        imu.orientation.x = float(msg.imu_state.quaternion[1])
        imu.orientation.y = float(msg.imu_state.quaternion[2])
        imu.orientation.z = float(msg.imu_state.quaternion[3])
        imu.orientation.w = float(msg.imu_state.quaternion[0])
        imu.angular_velocity.x = float(msg.imu_state.gyroscope[0])
        imu.angular_velocity.y = float(msg.imu_state.gyroscope[1])
        imu.angular_velocity.z = float(msg.imu_state.gyroscope[2])
        imu.linear_acceleration.x = float(msg.imu_state.accelerometer[0])
        imu.linear_acceleration.y = float(msg.imu_state.accelerometer[1])
        imu.linear_acceleration.z = float(msg.imu_state.accelerometer[2])
        self.imu_pub.publish(imu)

        self._imu_temp_msg.data = float(msg.imu_state.temperature)
        self.imu_temp_pub.publish(self._imu_temp_msg)

        # ODOM
        odom = self._odom_msg
        odom.header.stamp = stamp
        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = float(msg.position[1])
        odom.pose.pose.position.z = float(msg.position[2])
        odom.pose.pose.orientation.x = float(msg.imu_state.quaternion[1])
        odom.pose.pose.orientation.y = float(msg.imu_state.quaternion[2])
        odom.pose.pose.orientation.z = float(msg.imu_state.quaternion[3])
        odom.pose.pose.orientation.w = float(msg.imu_state.quaternion[0])

        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])
        odom.twist.twist.angular.z = float(msg.yaw_speed)

        if self.publish_odom:
            self.odom_pub.publish(odom)

        # TF
        if self.publish_tf:
            tf = self._tf_msg
            tf.header.stamp = stamp
            tf.transform.translation.x = float(odom.pose.pose.position.x)
            tf.transform.translation.y = float(odom.pose.pose.position.y)
            tf.transform.translation.z = float(odom.pose.pose.position.z)
            tf.transform.rotation = odom.pose.pose.orientation

            self.tf_broadcaster.sendTransform(tf)

    # ================= LIDAR =================
    def lidar_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.sim_mode:
            msg.header.frame_id = 'base_scan'
        else:
            msg.header.frame_id = 'radar'
        self.lidar_pub.publish(msg)

    # ================= LOW STATE =================
    def low_state_callback(self, msg):
        stamp = self.get_clock().now().to_msg()
        motors = msg.motor_state

        # Battery
        bat = self._battery_msg
        bat.header.stamp = stamp
        bat.voltage = float(msg.power_v)
        bat.current = float(msg.bms_state.current)
        bat.percentage = float(msg.bms_state.soc) / 100.0
        self.battery_pub.publish(bat)

        # Joint
        js = self._joint_state_msg
        js.header.stamp = stamp
        if self.sim_mode:
            #  sim  joint   ( Gazebo )
            #   0   TB3  
            #  Joint  URDF  TurtleBot3 
            #  TB3  2 joints 
            js.name = ['wheel_left_joint', 'wheel_right_joint']
            
            #  0  1  LowState 
            #   0.0 
            js.position = [float(motors[0].q), float(motors[1].q)]
            js.velocity = [float(motors[0].dq), float(motors[1].dq)]
            
            # :  'robot_state_publisher'  TB3  
            #  joint_states  RViz  (Jitter)
            #   publish 
        else:
            js.position = [float(motors[i].q) for i in MOTOR_INDICES]
            js.velocity = [float(motors[i].dq) for i in MOTOR_INDICES]
        self.joint_state_pub.publish(js)

        # Motor temp
        mt = self._motor_temp_msg
        mt.header.stamp = stamp
        mt.temperature = [float(motors[i].temperature) for i in MOTOR_INDICES]
        self.motor_temp_pub.publish(mt)


def main():
    rclpy.init()
    node = RobotRead()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()