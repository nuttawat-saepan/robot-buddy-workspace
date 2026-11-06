"""Run one obstacle scenario against sim_obstacle.launch.py and print the result.

    ros2 run go2_control sim_obstacle_test --scenario popup_close

Prints one JSON line. Refuses to run on anything but the simulated stack: it
needs /clock and the simulated actuator, neither of which exists on the robot.
"""
import argparse
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from go2_control.sim_scenarios import SCENARIOS, WALLS, body_gap, box_sdf

# How near counts as touching. The outline is checked every 2 cm.
CONTACT_M = 0.02


def verdict(scenario, arrived, min_gap, stop_commands):
    """PASS, FAIL or KNOWN, and why."""
    if scenario.known_contact and min_gap < CONTACT_M:
        # What happens after the hit is the physics of two boxes, not a result.
        return 'KNOWN', 'hit it, as expected for this limit'
    problems = []
    if min_gap < CONTACT_M:
        problems.append(f'touched: gap {min_gap:.2f} m')
    if arrived != scenario.arrives:
        problems.append('arrived' if arrived else 'did not arrive')
    if scenario.name in ('clear', 'door') and stop_commands:
        problems.append(f'guard stopped it {stop_commands} times')
    if problems:
        return 'FAIL', '; '.join(problems)
    return 'PASS', ''


class Runner(Node):

    def __init__(self, scenario, timeout):
        super().__init__('sim_obstacle_test')
        self.scenario = scenario
        self.timeout = timeout
        self.rectangles = list(WALLS) + list(scenario.boxes)
        self.pose = None
        self.min_gap = math.inf
        self.gap_at = None
        self.max_speed = 0.0
        self.track = []
        self.status = {}
        self.stop_reasons = {}
        self.popped = False
        self.popup_at = None
        self.first_stop_after_popup = None
        self.sim_now = None
        self.sim_samples = []
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(
            String, '/collision_guard/status', self.on_status, 10)
        self.create_subscription(
            Twist, '/cmd_vel_guarded', self.on_guarded, 20)
        self.guarded_zero_after_moving = 0
        self.last_command_moving = False

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.max_speed = max(self.max_speed, speed)
        gap = body_gap(p.x, p.y, yaw, self.rectangles)
        if gap < self.min_gap:
            self.min_gap = gap
            self.gap_at = (round(p.x, 2), round(p.y, 2))
        self.sim_samples.append(
            (time.monotonic(),
             msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))
        self.maybe_popup(p.x)

    def on_status(self, msg):
        self.status = json.loads(msg.data)
        why = self.status.get('stopped')
        if why:
            self.stop_reasons[why] = self.stop_reasons.get(why, 0) + 1
            if self.popup_at and self.first_stop_after_popup is None:
                self.first_stop_after_popup = round(
                    time.monotonic() - self.popup_at, 2)

    def on_guarded(self, msg):
        moving = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01
        if self.last_command_moving and not moving:
            self.guarded_zero_after_moving += 1
        self.last_command_moving = moving

    def maybe_popup(self, x):
        popup = self.scenario.popup
        if popup is None or self.popped or x < popup[0]:
            return
        self.popped = True
        self.popup_at = time.monotonic()
        self.rectangles.append(popup[1])
        self.spawn_box('popup', popup[1])

    def spawn_box(self, name, rectangle):
        request = SpawnEntity.Request()
        request.name = name
        request.xml = box_sdf(name, rectangle)
        return self.spawn.call_async(request)

    def wait(self, condition, seconds, what):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if condition():
                return
        raise RuntimeError(f'timed out waiting for {what}')

    def run(self):
        self.wait(lambda: self.pose is not None, 40, '/odom from Gazebo')
        if not self.spawn.wait_for_service(timeout_sec=20):
            raise RuntimeError('no /spawn_entity: is this the simulation?')
        for index, box in enumerate(self.scenario.boxes):
            done = self.spawn_box(f'box_{index}', box)
            self.wait(done.done, 15, 'box spawn')
        self.wait(lambda: self.tf_buffer.can_transform(
            'map', 'base_link', rclpy.time.Time()), 40, 'TF map -> base_link')
        if not self.nav.wait_for_server(timeout_sec=60):
            raise RuntimeError('no navigate_to_pose action server')
        # Costmaps need a few scans to take in boxes that were there at start.
        settle = time.monotonic() + 5.0
        self.wait(lambda: time.monotonic() > settle, 6, 'settling')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = self.scenario.goal[0]
        goal.pose.pose.position.y = self.scenario.goal[1]
        goal.pose.pose.orientation.w = 1.0
        handle = None
        deadline = time.monotonic() + 40
        while handle is None or not handle.accepted:
            if time.monotonic() > deadline:
                raise RuntimeError('Nav2 did not accept the goal')
            sent = self.nav.send_goal_async(goal)
            self.wait(sent.done, 10, 'goal response')
            handle = sent.result()
            if not handle.accepted:
                retry = time.monotonic() + 2.0
                self.wait(lambda: time.monotonic() > retry, 3, 'retry')

        started = time.monotonic()
        result = handle.get_result_async()
        timed_out = False
        while not result.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - started > self.timeout:
                timed_out = True
                handle.cancel_goal_async()
                break
        elapsed = time.monotonic() - started
        code = None if timed_out else result.result().status
        arrived = code == GoalStatus.STATUS_SUCCEEDED

        # Coasting after the end still counts towards the nearest approach.
        after = time.monotonic() + 2.0
        self.wait(lambda: time.monotonic() > after, 3, 'coasting')
        state, why = verdict(self.scenario, arrived, self.min_gap,
                             self.status.get('stopCommands', 0))
        rate = None
        if len(self.sim_samples) > 10:
            wall = self.sim_samples[-1][0] - self.sim_samples[0][0]
            rate = round((self.sim_samples[-1][1] - self.sim_samples[0][1])
                         / max(wall, 1e-6), 2)
        return {
            'scenario': self.scenario.name,
            'verdict': state,
            'why': why,
            'arrived': arrived,
            'navStatus': 'timeout' if timed_out else code,
            'seconds': round(elapsed, 1),
            'minGapM': round(self.min_gap, 3),
            'minGapAt': self.gap_at,
            'finalPose': [round(v, 2) for v in self.pose],
            'maxSpeed': round(self.max_speed, 2),
            'guardStopCommands': self.status.get('stopCommands'),
            'guardPassed': self.status.get('passed'),
            'guardFrameMs': self.status.get('frameMs'),
            'stopReasons': self.stop_reasons,
            'guardStopAfterPopupS': self.first_stop_after_popup,
            'realTimeFactor': rate,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True, choices=sorted(SCENARIOS))
    parser.add_argument('--timeout', type=float, default=150.0)
    args = parser.parse_args()
    rclpy.init()
    runner = Runner(SCENARIOS[args.scenario], args.timeout)
    try:
        print(json.dumps(runner.run()), flush=True)
    except RuntimeError as error:
        print(json.dumps({'scenario': args.scenario, 'verdict': 'ERROR',
                          'why': str(error)}), flush=True)
    finally:
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
