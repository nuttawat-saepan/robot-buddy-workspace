import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import json
import os

REQUIRED_ROBOT_ACK = 'I_UNDERSTAND_THIS_CAN_MOVE_THE_REAL_ROBOT'

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.robot_ack = self.declare_parameter('robot_ack', '').value
        if self.robot_ack != REQUIRED_ROBOT_ACK:
            self.get_logger().fatal(
                'Refusing to start cmd_vel_node: robot_ack is missing or invalid. '
                f'Expected robot_ack: {REQUIRED_ROBOT_ACK}'
            )
            raise RuntimeError('cmd_vel_node is not armed')

        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.sport.sport_client import SportClient

        # ===== INIT UNITREE =====
        # Which interface the SDK talks to the robot over. There is no default
        # that is right everywhere: on the Unitree board it is the robot's own
        # wired network, on the MiniPC it is whichever card reaches the robot
        # (measured as wlp4s0 on 2026-09-03). Hardcoding either one is how a
        # run ends up arming successfully while no command reaches the legs,
        # with nothing logged to say so.
        #
        # Precedence, most specific first:
        #   the network_interface launch/CLI parameter
        #   the UNITREE_IF environment variable, set by scripts/onsite.env
        #   eth0, which is correct when this runs on the board
        self.network_interface = self.declare_parameter(
            'network_interface',
            # os.environ.get('UNITREE_IF', 'eth0'),
            os.environ.get('UNITREE_IF', 'enp5s0'),
        ).value
        self.get_logger().info(
            f'Unitree SDK interface: {self.network_interface} '
            f'(UNITREE_IF={os.environ.get("UNITREE_IF", "<unset>")})')
        ChannelFactoryInitialize(0, self.network_interface)

        self.client = SportClient()
        self.client.SetTimeout(1.0)
        self.client.Init()

        # ===== LIMIT =====
        # This is the last clamp before the robot's own controller, so it is
        # the one that has to hold when something other than Nav2 publishes to
        # /cmd_vel - a teleop left running, a hand test, a stale node. It used
        # to be 1.0 m/s, twenty times what nav2_livox_go2.yaml allows, which
        # meant the crawl limit was enforced only by Nav2's configuration and
        # by nothing at all downstream of it.
        #
        # Defaults match nav2_livox_go2.yaml. Raise both together, and only
        # after a run has shown that localisation, obstacle sensing and the
        # stop path hold at the current speed.
        self.max_vx = float(self.declare_parameter('max_vx', 0.05).value)
        self.max_vy = float(self.declare_parameter('max_vy', 0.05).value)
        self.max_vyaw = float(self.declare_parameter('max_vyaw', 0.20).value)
        self.get_logger().info(
            f'velocity clamp: vx +/-{self.max_vx} m/s, vy +/-{self.max_vy} m/s, '
            f'wz +/-{self.max_vyaw} rad/s')

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        # ===== STATE =====
        self.last_cmd = None
        self.last_time = self.get_clock().now()
        self.stopped = False
        self.zero_sent = False
        self.cmd_seen = False

        # Command staleness timeout. Nav2's controller runs at 5 Hz here, so
        # 0.5 s is five missed commands - long enough not to trip on ordinary
        # scheduling jitter, short enough that a dropped link stops the robot
        # within half a metre at the configured speeds.
        #
        # This matters more than it looks. In the deployment architecture Nav2
        # runs on a ground station and /cmd_vel crosses Wi-Fi, so a stalled
        # link means no new commands arrive while the robot is mid-stride.
        # Without this check the loop below would keep resending the last
        # velocity forever.
        self.cmd_timeout = float(
            self.declare_parameter('cmd_timeout', 0.5).value)

        # ===== SUB =====
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            qos
        )

        self.sub_cmd = self.create_subscription(
            String,
            '/unitree_cmd',
            self.cmd_callback,
            qos
        )

        # ===== TIMER (auto stop only) =====
        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info("CmdVel Direct + AutoStop + ZeroOnce started")

    # =========================
    # CMD_VEL → MOVE
    # =========================
    def callback(self, msg: Twist):
        try:
            self.last_time = self.get_clock().now()
            self.cmd_seen = True

            self.target_vx = max(-self.max_vx, min(self.max_vx, msg.linear.x))
            self.target_vy = max(-self.max_vy, min(self.max_vy, msg.linear.y))
            self.target_wz = max(-self.max_vyaw, min(self.max_vyaw, msg.angular.z))

        except Exception as e:
            self.get_logger().error(f"cmd_vel error: {e}")

    def update(self):
        """Repeat the last command to the robot, or stop it if that command
        has gone stale.

        The staleness branch used to live in a second `def update` further down
        the file, commented out. Uncommenting it would not have worked: Python
        keeps the last definition of a name, so it would have replaced this one
        and removed the Move() call entirely, leaving a node that could only
        ever stop the robot. The two are merged here instead, staleness first.
        """
        try:
            dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9

            # Stale command: stop, and stay stopped until something new
            # arrives. Checked before anything else so no other branch can
            # issue a Move() on data this old.
            if self.cmd_seen and dt > self.cmd_timeout:
                if not self.stopped:
                    self.client.Move(0.0, 0.0, 0.0)
                    self.get_logger().warn(
                        f'STOP: no /cmd_vel for {dt:.2f}s '
                        f'(timeout {self.cmd_timeout:.2f}s)')
                    self.stopped = True
                    self.zero_sent = True
                    self.target_vx = 0.0
                    self.target_vy = 0.0
                    self.target_wz = 0.0
                return

            if abs(self.target_vx) < 1e-3 and abs(self.target_wz) < 1e-3 and abs(self.target_vy) < 1e-3:
                if not self.zero_sent:
                    self.client.Move(0.0, 0.0, 0.0)
                    self.get_logger().info("🛑 STOP (zero velocity)")
                    self.zero_sent = True
                return

            self.zero_sent = False
            self.stopped = False

            self.client.Move(self.target_vx, self.target_vy, self.target_wz)
            self.get_logger().info("MOVE cmd_vel: vx=%.2f, wz=%.2f" % (self.target_vx, self.target_wz))
        except Exception as e:
            self.get_logger().error(f"update error: {e}")

    # =========================
    # COMMAND
    # =========================
    def cmd_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            cmd = data.get("cmd", "")

            if cmd == self.last_cmd:
                return
            self.last_cmd = cmd

            if cmd == "stand":
                self.get_logger().info("STAND")
                self.client.StandUp()

            elif cmd == "sit":
                self.get_logger().info("SIT")
                self.client.Sit()

            elif cmd == "stop":
                self.get_logger().warn("STOP")
                self.client.StopMove()

            else:
                self.get_logger().warn(f"Unknown cmd: {cmd}")

        except Exception as e:
            self.get_logger().error(f"cmd parse error: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CmdVelBridge()
    except RuntimeError:
        rclpy.shutdown()
        raise SystemExit(1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
