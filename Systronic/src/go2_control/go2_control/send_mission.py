"""Send a goal or a waypoint list to Nav2 from the command line.

The two existing ways to start a navigation run both need something that is not
available on the robot: RViz needs a screen, and main.py needs an MQTT broker
and the web front end. On the Unitree board there is neither. This is the third
way, and it is the one to use on site.

    ros2 run go2_control send_mission --goal 1.0 0.0 0.0
    ros2 run go2_control send_mission --file missions/site_a.json
    ros2 run go2_control send_mission --file missions/site_a.json --dry-run

THIS MOVES THE ROBOT, if the rest of the stack is armed. It publishes nothing
itself - it sends a NavigateToPose goal, and whether that reaches the legs
depends on cmd_vel_node, the watchdog and the Unitree bridge, each of which has
its own gate. A dry run sends nothing at all.

## Why it walks the list itself rather than using FollowWaypoints

Nav2 has a waypoint_follower, and it is running. Sending it the whole list
would be less code here. But the mission this project needs is a waypoint list
*with capture points*: stop, photograph, report, continue, and be interruptible
at any point. Driving NavigateToPose one pose at a time is what main.py already
does, and it leaves the pause between waypoints under our control instead of
inside a Nav2 plugin. Keeping the two the same means what is proven here is
proven for the mission executor too.

## Mission file

    {
      "frame_id": "map",
      "waypoints": [
        {"name": "door",    "x": 1.0,  "y": 0.0,  "yaw": 0.0},
        {"name": "corner",  "x": 3.2,  "y": -1.5, "yaw": 1.57, "capture": true}
      ]
    }

`yaw` is radians. `capture` marks a photograph point; this tool pauses there and
says so, because the camera and upload path are not written yet. The field is
in the format now so the files written on site do not have to be rewritten
later.

Write one with `ros2 run go2_control record_waypoint`.
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from go2_control.nav_ready_check import NavReadyCheck


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Send a goal or waypoint list to Nav2.')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        help='One pose in the map frame. YAW in radians.')
    source.add_argument('--file', help='Mission JSON, see the module docstring.')
    parser.add_argument('--dry-run', action='store_true',
                        help='Print what would be sent and exit. Sends nothing.')
    parser.add_argument('--skip-ready-check', action='store_true',
                        help='Send even if the stack is not ready. The checks '
                             'exist because a goal given to an unlocalised '
                             'AMCL drives the robot confidently to the wrong '
                             'place; skip them only when you know why.')
    parser.add_argument('--ready-timeout', type=float, default=20.0,
                        help='Seconds to wait for the readiness checks to pass.')
    parser.add_argument('--capture-pause', type=float, default=3.0,
                        help='Seconds to hold at a waypoint marked capture.')
    parser.add_argument('--frame', default='map')
    parser.add_argument('--controller-topic', default='/cmd_vel_nav_preview',
                        help='Topic the controller publishes velocity on, used '
                             'only to tell whether it is active. The Livox '
                             'launch files remap it away from /cmd_vel so a '
                             'planning run cannot reach the robot; a stock Nav2 '
                             'bringup, including the Gazebo sim, leaves it as '
                             '/cmd_vel.')
    return parser.parse_args(argv)


def load_mission(args):
    """Return (frame_id, [waypoint dicts])."""
    if args.goal:
        x, y, yaw = args.goal
        return args.frame, [{'name': 'goal', 'x': x, 'y': y, 'yaw': yaw}]

    with open(args.file) as handle:
        data = json.load(handle)
    waypoints = data.get('waypoints', [])
    if not waypoints:
        raise SystemExit(f'{args.file} has no waypoints')
    for i, wp in enumerate(waypoints):
        for key in ('x', 'y'):
            if key not in wp:
                raise SystemExit(f'waypoint {i} has no "{key}"')
        wp.setdefault('yaw', 0.0)
        wp.setdefault('name', f'wp{i + 1}')
    return data.get('frame_id', args.frame), waypoints


def describe(frame, waypoints):
    print(f'{len(waypoints)} waypoint(s) in frame {frame}')
    total = 0.0
    prev = None
    for i, wp in enumerate(waypoints, 1):
        here = (wp['x'], wp['y'])
        if prev is not None:
            total += math.hypot(here[0] - prev[0], here[1] - prev[1])
        prev = here
        mark = '  [capture]' if wp.get('capture') else ''
        print(f'  {i:2}. {wp["name"]:16} x={wp["x"]:7.2f}  y={wp["y"]:7.2f}  '
              f'yaw={math.degrees(wp["yaw"]):7.1f} deg{mark}')
    print(f'  path length between waypoints: {total:.2f} m '
          f'(excludes the leg from wherever the robot is now)')


def to_pose(frame, wp, clock):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = clock.now().to_msg()
    pose.pose.position.x = float(wp['x'])
    pose.pose.position.y = float(wp['y'])
    pose.pose.orientation.z = math.sin(float(wp['yaw']) / 2.0)
    pose.pose.orientation.w = math.cos(float(wp['yaw']) / 2.0)
    return pose


def wait_ready(check, timeout):
    """Spin the readiness node until every check passes, or give up."""
    print(f'checking whether the stack is ready (up to {timeout:.0f}s)')
    deadline = time.monotonic() + timeout
    ok = False
    while time.monotonic() < deadline:
        rclpy.spin_once(check, timeout_sec=0.2)
        ok, _ = check.readiness()
        if ok:
            break
    check.report()
    return ok


class MissionRunner:

    def __init__(self, node, frame, capture_pause):
        self.node = node
        self.frame = frame
        self.capture_pause = capture_pause
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None
        self.result = None
        self.last_print = 0.0

    def feedback(self, msg):
        # One line a second, not one per feedback message: the point is to see
        # that the number is falling, and a wall of text hides that.
        now = time.monotonic()
        if now - self.last_print < 1.0:
            return
        self.last_print = now
        fb = msg.feedback
        print(f'    {fb.distance_remaining:6.2f} m to go, '
              f'{fb.navigation_time.sec:3d}s elapsed, '
              f'{fb.number_of_recoveries} recovery/ies',
              flush=True)

    def send(self, wp):
        goal = NavigateToPose.Goal()
        goal.pose = to_pose(self.frame, wp, self.node.get_clock())

        send_future = self.client.send_goal_async(
            goal, feedback_callback=self.feedback)
        rclpy.spin_until_future_complete(self.node, send_future)
        self.goal_handle = send_future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            print('    goal REJECTED by Nav2')
            return False

        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        self.goal_handle = None
        status = result_future.result().status
        # 4 is STATUS_SUCCEEDED in action_msgs/GoalStatus.
        if status == 4:
            print('    reached')
            return True
        print(f'    did NOT reach, status {status}')
        return False

    def close(self):
        """Destroy the action client before its node.

        Left to the garbage collector it is destroyed after the node, and
        rclpy raises InvalidHandle from __del__ - a traceback on a clean exit,
        which on site reads as a crash.
        """
        self.client.destroy()

    def cancel(self):
        """Cancel whatever is running. Called on Ctrl-C."""
        if self.goal_handle is None:
            return
        print('\ncancelling the active goal')
        future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        print('cancel sent - the robot should stop')


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    frame, waypoints = load_mission(args)

    describe(frame, waypoints)
    if args.dry_run:
        print('\ndry run: nothing was sent')
        return

    rclpy.init()
    check = NavReadyCheck(node_name='send_mission_ready', periodic=False,
                          overrides={'controller_cmd_topic': args.controller_topic})
    node = rclpy.create_node('send_mission')
    runner = MissionRunner(node, frame, args.capture_pause)

    try:
        if args.skip_ready_check:
            print('readiness checks SKIPPED')
        elif not wait_ready(check, args.ready_timeout):
            print('\nnot ready - no goal was sent.')
            print('Fix what is failing above, or pass --skip-ready-check if '
                  'you know why it is failing and want to send anyway.')
            return

        print('\nwaiting for the navigate_to_pose action server')
        if not runner.client.wait_for_server(timeout_sec=10.0):
            print('no action server - is Nav2 running and its lifecycle active?')
            return

        for i, wp in enumerate(waypoints, 1):
            print(f'\n[{i}/{len(waypoints)}] {wp["name"]} '
                  f'-> ({wp["x"]:.2f}, {wp["y"]:.2f})')
            if not runner.send(wp):
                print('stopping the mission here')
                break
            if wp.get('capture'):
                print(f'    capture point: holding {args.capture_pause:.0f}s')
                print('    (the camera and upload path are not wired up yet)')
                time.sleep(args.capture_pause)
        else:
            print('\nmission complete')
    except KeyboardInterrupt:
        runner.cancel()
    finally:
        runner.close()
        node.destroy_node()
        check.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
