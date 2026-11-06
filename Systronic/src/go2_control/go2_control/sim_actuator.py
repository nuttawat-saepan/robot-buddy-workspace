"""Stand in for the part of the robot a simulated box does not have.

Gazebo's planar_move plugin takes a velocity at once, so a box in Gazebo stops
dead the instant it is told to, and any stop line would look good. This delays
commands and limits how quickly velocity can change before they reach the box.

    /cmd_vel_guarded -> this node -> /sim/cmd_vel -> the box

The delay and acceleration are assumptions, not measurements of a Go2W. The
result is only as good as they are, which is why braking distance still has to
be measured on the robot. The 0.30 s command timeout is the real bridge's.
"""
from collections import deque
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def step_towards(current, target, limit):
    """Move current to target by at most limit."""
    return current + max(-limit, min(limit, target - current))


class SimActuator(Node):

    def __init__(self):
        super().__init__('sim_actuator')
        declare = self.declare_parameter
        self.delay = float(declare('delay', 0.15).value)
        self.linear_accel = float(declare('linear_accel', 1.0).value)
        self.angular_accel = float(declare('angular_accel', 2.0).value)
        self.command_timeout = float(declare('command_timeout', 0.30).value)
        # The bridge's clamps, from onsite.env.example. Without them a recovery
        # spin would run at Nav2's 1.5 rad/s, which the robot never sees.
        self.max_linear = float(declare('max_linear', 0.45).value)
        self.max_angular = float(declare('max_angular', 0.60).value)
        self.queue = deque()
        self.target = (0.0, 0.0)
        self.target_at = None
        self.vx = 0.0
        self.wz = 0.0
        self.last_tick = time.monotonic()
        self.pub = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.create_subscription(
            Twist, '/cmd_vel_guarded', self.on_cmd, 10)
        self.create_timer(0.02, self.tick)

    def on_cmd(self, msg):
        self.queue.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        while self.queue and now - self.queue[0][0] >= self.delay:
            sent, vx, wz = self.queue.popleft()
            self.target = (
                max(-self.max_linear, min(self.max_linear, vx)),
                max(-self.max_angular, min(self.max_angular, wz)))
            self.target_at = sent + self.delay
        if (self.target_at is None
                or now - self.target_at > self.command_timeout):
            self.target = (0.0, 0.0)
        self.vx = step_towards(self.vx, self.target[0], self.linear_accel * dt)
        self.wz = step_towards(self.wz, self.target[1], self.angular_accel * dt)
        out = Twist()
        out.linear.x = self.vx
        out.angular.z = self.wz
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimActuator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
