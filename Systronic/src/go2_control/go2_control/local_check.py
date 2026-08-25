import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu, JointState, PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from unitree_go.msg import LowState, SportModeState


CHECK_TOPICS = [
    ('/lf/lowstate', LowState),
    ('/lf/sportmodestate', SportModeState),
    ('/utlidar/cloud', PointCloud2),
    ('/pointcloud', PointCloud2),
    ('/odom', Odometry),
    ('/imu/data', Imu),
    ('/battery', BatteryState),
    ('/joint_states', JointState),
]

OPTIONAL_TRANSIENT_TOPICS = [
    ('/map', OccupancyGrid),
]


class LocalCheck(Node):
    def __init__(self):
        super().__init__('go2_local_check')
        self.timeout_sec = float(self.declare_parameter('timeout_sec', 5.0).value)
        self.counts = {topic: 0 for topic, _ in CHECK_TOPICS + OPTIONAL_TRANSIENT_TOPICS}
        self.first_seen = {}

        normal_qos = QoSProfile(depth=10)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        for topic, msg_type in CHECK_TOPICS:
            self.create_subscription(
                msg_type,
                topic,
                lambda msg, topic=topic: self._seen(topic),
                normal_qos,
            )

        for topic, msg_type in OPTIONAL_TRANSIENT_TOPICS:
            self.create_subscription(
                msg_type,
                topic,
                lambda msg, topic=topic: self._seen(topic),
                map_qos,
            )

    def _seen(self, topic):
        self.counts[topic] += 1
        self.first_seen.setdefault(topic, time.monotonic())

    def report(self):
        print('\nGO2 local simulation readiness check')
        print('Read-only check: no /cmd_vel publish, no robot commands.\n')

        node_names = set(self.get_node_names())
        expected_nodes = [
            'gazebo_convert',
            'go2w_read',
            'scan_to_pointcloud',
            'map_server',
            'amcl',
        ]
        for node_name in expected_nodes:
            status = 'OK' if node_name in node_names else 'WARN'
            print(f'[{status}] node {node_name}')

        cmd_vel_publishers = self.get_publishers_info_by_topic('/cmd_vel')
        publisher_names = sorted({info.node_name for info in cmd_vel_publishers})
        if publisher_names:
            print(f'[INFO] /cmd_vel publishers in graph: {publisher_names}')
            print('[SAFE] local_check does not publish /cmd_vel; do not send Nav2 goals during safety-only checks')
        else:
            print('[SAFE] no /cmd_vel publishers detected')

        for topic, _ in CHECK_TOPICS:
            status = 'OK' if self.counts[topic] > 0 else 'FAIL'
            print(f'[{status}] topic {topic}: {self.counts[topic]} messages')

        for topic, _ in OPTIONAL_TRANSIENT_TOPICS:
            status = 'OK' if self.counts[topic] > 0 else 'WARN'
            print(f'[{status}] topic {topic}: {self.counts[topic]} messages')

        missing = [topic for topic, _ in CHECK_TOPICS if self.counts[topic] == 0]
        if missing:
            print('\nResult: FAIL')
            print('Missing required data on: ' + ', '.join(missing))
            return 1

        print('\nResult: OK')
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = LocalCheck()
    deadline = time.monotonic() + node.timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    exit_code = node.report()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
