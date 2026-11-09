#!/usr/bin/env python3
"""Read and change the robot's settings for the web, restarting the stack when needed.

Most of what is worth tuning cannot be changed live. AMCL, the costmaps and
pointcloud_to_laserscan read their parameters once, so applying a value means a
restart, and mqtt_mission_bridge cannot do that because it is restarted too.
This runs beside the stack and owns it.

    ros2 run go2_control stack_supervisor
    GET   /robot/settings   what the nodes use now, read back from them
    PATCH /robot/settings   apply, verify by reading back, roll back on failure
    GET   /robot/preflight  why motion must not start now
    POST  /robot/bridge     {"action": "arm", "confirm": ...} or {"action": "disarm"}

The backend reaches this with x-robot-settings-token. Set
STACK_SUPERVISOR_TOKEN on the board; without it every request is refused.

Applying settings starts no motion: it can stop unitree_udp_bridge but never
starts it, and rearmRequired stays true until the operator arms it again. Arming
is a separate request carrying the operator's confirmation, or with --auto-arm
the web starting a mission, and is refused unless every preflight check passes.
The bridge is stopped again when the mission ends,
after a minute with no mission, and at once if FAST-LIO runs away. The gates that
allow motion and the addresses the page depends on are not settings.
A restart is refused during a mission and unless the IMU shows the robot still,
because FAST-LIO starts from wherever the robot is and needs it still.
"""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
import shlex
import shutil
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import yaml

# name: (type, minimum, maximum, how it is applied)
FIELDS = {
    'mission_max_linear': (float, 0.05, 0.40, 'live'),
    'mission_max_angular': (float, 0.05, 0.60, 'live'),
    'bridge_max_linear': (float, 0.05, 0.45, 'restart'),
    'bridge_max_angular': (float, 0.05, 0.60, 'restart'),
    'scan_min_height': (float, 0.0, 2.5, 'restart'),
    'scan_max_height': (float, 0.1, 3.0, 'restart'),
    'amcl_scan_min_height': (float, 0.0, 2.5, 'restart'),
    'amcl_scan_max_height': (float, 0.1, 3.0, 'restart'),
    'alpha1': (float, 0.001, 2.0, 'restart'),
    'alpha2': (float, 0.001, 2.0, 'restart'),
    'alpha3': (float, 0.001, 2.0, 'restart'),
    'alpha4': (float, 0.001, 2.0, 'restart'),
    'planner_tolerance': (float, 0.0, 1.0, 'restart'),
    'xy_goal_tolerance': (float, 0.1, 1.0, 'restart'),
    'yaw_goal_tolerance': (float, 0.05, 1.0, 'restart'),
    'localization_spread_limit': (float, 0.1, 1.5, 'restart'),
}
BRIDGE_ONLY = {'bridge_max_linear', 'bridge_max_angular'}
# Where a value can be read back from the node using it.
READBACK = {
    'mission_max_linear': ('/controller_server', 'FollowPath.max_vel_x'),
    'mission_max_angular': ('/controller_server', 'FollowPath.max_vel_theta'),
    'scan_min_height': ('/pc2scan_livox_lio', 'min_height'),
    'scan_max_height': ('/pc2scan_livox_lio', 'max_height'),
    'amcl_scan_min_height': ('/pc2scan_livox_amcl', 'min_height'),
    'amcl_scan_max_height': ('/pc2scan_livox_amcl', 'max_height'),
    'alpha1': ('/amcl', 'alpha1'),
    'alpha2': ('/amcl', 'alpha2'),
    'alpha3': ('/amcl', 'alpha3'),
    'alpha4': ('/amcl', 'alpha4'),
    'planner_tolerance': ('/planner_server', 'GridBased.tolerance'),
    'xy_goal_tolerance': ('/controller_server',
                          'goal_checker.xy_goal_tolerance'),
    'yaw_goal_tolerance': ('/controller_server',
                           'goal_checker.yaw_goal_tolerance'),
}
BAG_TOPICS = ('/livox/lidar /livox/imu /scan /scan_amcl /map /amcl_pose '
              '/particlecloud /initialpose /tf /tf_static /Odometry '
              '/localization_watch/status /cmd_vel_nav_preview '
              '/cmd_vel_safe /cmd_vel_guarded /collision_guard/status').split()
READY_LINE = 'Managed nodes are active'
# An idle robot on its IMU: about 1 g and almost no rotation. Measured on the
# board at 1.02 g and 0.03 rad/s; the shaking before a FAST-LIO runaway reached
# 1.8 g and 0.69 rad/s.
STILL_GYRO = 0.10
STILL_ACCEL_G = 0.08
STALE_S = 5.0
# The bridge caps the robot well under this; only a runaway estimate reaches it.
RUNAWAY_MPS = 1.0
END_DISARM_S = 3.0
IDLE_DISARM_S = 60.0
ARM_CONFIRM = 'AREA_CLEAR_REMOTE_IN_HAND'


class ApplyError(Exception):
    """A request that cannot be applied. status is the HTTP code to return."""

    def __init__(self, status, message):
        super().__init__(message)
        self.status = status
        self.message = message


def validate_changes(changes, current, allow_restart):
    """Return (merged settings, applied keys). Raise ApplyError on anything else."""
    if not isinstance(changes, dict) or not changes:
        raise ApplyError(422, 'changes must be a non-empty object')
    merged = dict(current)
    for key, value in changes.items():
        if key not in FIELDS:
            raise ApplyError(422, '%s is not a setting' % key)
        kind, low, high, _ = FIELDS[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ApplyError(422, '%s must be a number' % key)
        if not math.isfinite(value):
            raise ApplyError(422, '%s must be finite' % key)
        if not low <= value <= high:
            raise ApplyError(422, '%s must be %s to %s' % (key, low, high))
        merged[key] = kind(value)
    for low_key, high_key in (('scan_min_height', 'scan_max_height'),
                              ('amcl_scan_min_height', 'amcl_scan_max_height')):
        if (merged.get(low_key) is not None and merged.get(high_key) is not None
                and merged[low_key] >= merged[high_key]):
            raise ApplyError(422, '%s must be below %s' % (low_key, high_key))
    for mission, bridge in (('mission_max_linear', 'bridge_max_linear'),
                            ('mission_max_angular', 'bridge_max_angular')):
        if merged.get(mission) is not None and merged.get(bridge) is not None \
                and merged[mission] > merged[bridge] + 1e-9:
            raise ApplyError(422, '%s must not exceed %s' % (mission, bridge))
    needs_restart = any(FIELDS[k][3] == 'restart' for k in changes)
    if needs_restart and not allow_restart:
        raise ApplyError(409, 'these settings need a restart: allowRestart is false')
    return merged, sorted(changes)


def revision_of(settings):
    """Return a version that changes only when the configuration does."""
    text = json.dumps(settings, sort_keys=True)
    return 'config-' + hashlib.sha256(text.encode()).hexdigest()[:12]


def amcl_config(base, settings):
    """Copy base with the alphas written into its named section."""
    config = copy.deepcopy(base)
    params = config['amcl']['ros__parameters']
    for key in ('alpha1', 'alpha2', 'alpha3', 'alpha4'):
        params[key] = settings[key]
    return config


def nav2_config(base, settings):
    """Copy base with speed, tolerance and goal settings written in."""
    config = copy.deepcopy(base)
    ctrl = config['controller_server']['ros__parameters']
    ctrl['FollowPath']['max_vel_x'] = settings['mission_max_linear']
    ctrl['FollowPath']['max_vel_theta'] = settings['mission_max_angular']
    ctrl['goal_checker']['xy_goal_tolerance'] = settings['xy_goal_tolerance']
    ctrl['goal_checker']['yaw_goal_tolerance'] = settings['yaw_goal_tolerance']
    config['planner_server']['ros__parameters']['GridBased']['tolerance'] = \
        settings['planner_tolerance']
    return config


def defaults_from(share, env=None):
    """Return the settings the installed config files describe."""
    env = os.environ if env is None else env
    with open(os.path.join(share, 'config', 'amcl_livox.yaml')) as stream:
        amcl = yaml.safe_load(stream)['amcl']['ros__parameters']
    with open(os.path.join(share, 'config', 'nav2_livox_go2.yaml')) as stream:
        nav2 = yaml.safe_load(stream)
    ctrl = nav2['controller_server']['ros__parameters']
    return {
        'mission_max_linear': 0.20,
        'mission_max_angular': float(ctrl['FollowPath']['max_vel_theta']),
        'bridge_max_linear': 0.20, 'bridge_max_angular': 0.35,
        'scan_min_height': 0.25, 'scan_max_height': 1.05,
        'amcl_scan_min_height': 0.60, 'amcl_scan_max_height': 1.10,
        'alpha1': amcl['alpha1'], 'alpha2': amcl['alpha2'],
        'alpha3': amcl['alpha3'], 'alpha4': amcl['alpha4'],
        'planner_tolerance': nav2['planner_server']['ros__parameters'][
            'GridBased']['tolerance'],
        'xy_goal_tolerance': ctrl['goal_checker']['xy_goal_tolerance'],
        'yaw_goal_tolerance': ctrl['goal_checker']['yaw_goal_tolerance'],
        'localization_spread_limit': float(
            env.get('GO2_SPREAD_LIMIT_M', '0.50')),
    }


def launch_args(settings, map_yaml, amcl_file, nav2_file, extra=()):
    """Return the ros2 launch command for these settings."""
    args = ['ros2', 'launch', 'go2_control', 'onsite_stack.launch.py',
            'map:=%s' % map_yaml, 'amcl_params_file:=%s' % amcl_file,
            'nav2_params_file:=%s' % nav2_file,
            'scan_min_height:=%s' % settings['scan_min_height'],
            'scan_max_height:=%s' % settings['scan_max_height']]
    if settings.get('amcl_scan_min_height') is not None:
        args += ['amcl_scan_min_height:=%s' % settings['amcl_scan_min_height'],
                 'amcl_scan_max_height:=%s' % settings['amcl_scan_max_height']]
    return args + list(extra)


def bridge_caps(command_line):
    """Return (linear, angular) from a running bridge's arguments, or None."""
    words = command_line.split()
    try:
        return (float(words[words.index('--max-linear') + 1]),
                float(words[words.index('--max-angular') + 1]))
    except (ValueError, IndexError):
        return None


def is_still(samples):
    """Decide from (gyro rad/s, accel g) samples whether the robot is still."""
    if not samples:
        return False
    return (max(g for g, _ in samples) < STILL_GYRO and
            max(abs(a - 1.0) for _, a in samples) < STILL_ACCEL_G)


def preflight_problems(state):
    """Return why motion must not start; empty means every check passed."""
    problems = []
    now = state['now']
    if not state['stack_up']:
        problems.append('the stack is not running')
    if state['other_lidar_user']:
        problems.append('another system is using the LiDAR')
    if state['mission_active']:
        problems.append('a mission or Nav2 goal is already active')
    if not state['stationary']:
        problems.append('the IMU does not show the robot still')
    speed = state['lio_speed']
    if speed is None:
        problems.append('no FAST-LIO pose')
    elif speed > 0.2:
        problems.append('FAST-LIO reports %.2f m/s on a still robot' % speed)
    if not state['pose_set_at']:
        problems.append('the pose has not been set since the stack started')
    if now - state['position_at'] > STALE_S:
        problems.append('no recent localization data')
    else:
        spread = state['position'].get('localizationSpread')
        if spread is not None and spread > state['spread_limit']:
            problems.append('spread %.2f m is above %.2f m'
                            % (spread, state['spread_limit']))
        check = state['position'].get('localizationCheck') or {}
        if check.get('ok') is False:
            problems.append('AMCL is %.2f m from FAST-LIO, limit %.2f m'
                            % (check.get('distance', 0), check.get('limit', 0)))
    return problems


def localization_state(state):
    """Say where confirming the pose stands, for the page to guide the operator.

    The pose is only confirmed by driving a little: AMCL publishes no cloud until
    the robot has moved, so straight after setting it there is nothing to judge.
    """
    if not state['pose_set_at']:
        return {'state': 'not_set', 'message': 'set the pose first'}
    if state['now'] - state['position_at'] > STALE_S:
        return {'state': 'failed', 'message': 'no recent localization data'}
    position = state['position']
    check = position.get('localizationCheck') or {}
    if check.get('ok') is False:
        return {'state': 'failed',
                'message': 'AMCL is %.2f m from FAST-LIO: stop and check the '
                           'scan against the map' % check.get('distance', 0)}
    spread = position.get('localizationSpread')
    if spread is None:
        return {'state': 'searching',
                'message': 'drive about 0.5 m with the remote so AMCL can update'}
    if spread > state['spread_limit']:
        return {'state': 'searching',
                'message': 'spread %.2f m, keep driving slowly' % spread}
    return {'state': 'ready',
            'message': 'check the position on the map against the robot'}


def iso(stamp):
    """Format a time.time() value as ISO8601 UTC."""
    return datetime.fromtimestamp(stamp, timezone.utc).isoformat().replace(
        '+00:00', 'Z')


def command_lines(pattern):
    """Return (pid, command line) for processes matching pattern from the start."""
    done = subprocess.run(['pgrep', '-af', '^' + pattern],
                          stdout=subprocess.PIPE, universal_newlines=True)
    out = []
    for line in done.stdout.splitlines():
        pid, _, text = line.partition(' ')
        if int(pid) != os.getpid():
            out.append((int(pid), text))
    return out


def descendants(pid):
    """Return every process below pid."""
    done = subprocess.run(['pgrep', '-P', str(pid)],
                          stdout=subprocess.PIPE, universal_newlines=True)
    found = []
    for child in (int(p) for p in done.stdout.split()):
        found += [child] + descendants(child)
    return found


def signal_all(pids, sig):
    """Send sig to whichever of pids are still there."""
    for pid in pids:
        try:
            os.kill(pid, sig)
        except OSError:
            pass


def alive(pids):
    """Return the pids still running."""
    return [p for p in pids if os.path.exists('/proc/%d' % p)]


class Observer:
    """What the robot is doing, from its own topics and parameter services."""

    def __init__(self, broker):
        import rclpy
        from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
        from action_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from sensor_msgs.msg import Imu
        from tf2_msgs.msg import TFMessage
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node('stack_supervisor')
        self.lock = threading.Lock()
        self.imu = []
        self.imu_at = 0.0
        self.goal_active = False
        self.goal_at = 0.0
        self.lio = []
        self.values = {}
        self.values_at = 0.0
        self.bridge_state = {}
        self.bridge_state_at = 0.0
        best = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(Imu, '/livox/imu', self._on_imu, best)
        self.node.create_subscription(TFMessage, '/tf', self._on_tf, 50)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._on_goal, latched)
        self.node.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._on_pose, 10)
        self.pose_set_at = 0.0
        self.position = {}
        self.position_at = 0.0
        self.clients = {}
        self.mqtt = self._connect_mqtt(broker)
        threading.Thread(target=rclpy.spin, args=(self.node,),
                         daemon=True).start()
        threading.Thread(target=self._poll, daemon=True).start()

    def _connect_mqtt(self, broker):
        import paho.mqtt.client as paho
        if not broker:
            return None

        def on_connect(client, _userdata, _flags, _rc):
            client.subscribe('/missions/state')
            client.subscribe('/missions/pose')

        def on_message(_client, _userdata, msg):
            try:
                data = json.loads(msg.payload)
            except ValueError:
                return
            if msg.topic == '/missions/pose':
                self.position = data
                self.position_at = time.time()
            elif 'hasMission' in data:
                self.bridge_state = data
                self.bridge_state_at = time.time()

        client = paho.Client()
        client.on_connect = on_connect
        client.on_message = on_message
        client.connect_async(broker, int(os.environ.get('MQTT_PORT', 1883)), 30)
        client.loop_start()
        return client

    def _on_imu(self, msg):
        a = msg.linear_acceleration
        g = msg.angular_velocity
        accel = math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2)
        gyro = math.sqrt(g.x ** 2 + g.y ** 2 + g.z ** 2)
        now = time.time()
        with self.lock:
            self.imu.append((now, gyro, accel))
            self.imu = [s for s in self.imu if now - s[0] < 3.0]
            self.imu_at = now

    def _on_tf(self, msg):
        now = time.time()
        for t in msg.transforms:
            if t.child_frame_id == 'body':
                p = t.transform.translation
                with self.lock:
                    self.lio.append((now, p.x, p.y, p.z))
                    self.lio = [s for s in self.lio if now - s[0] < 15.0]

    def _on_pose(self, _msg):
        self.pose_set_at = time.time()

    def lio_speed(self, seconds=1.0):
        """Return FAST-LIO's speed over the window, None without enough data."""
        now = time.time()
        with self.lock:
            points = [s for s in self.lio if now - s[0] < seconds]
        if len(points) < 3 or points[-1][0] <= points[0][0]:
            return None
        travelled = sum(math.dist(a[1:], b[1:])
                        for a, b in zip(points, points[1:]))
        return travelled / (points[-1][0] - points[0][0])

    def _on_goal(self, msg):
        self.goal_active = any(s.status in (1, 2, 3) for s in msg.status_list)
        self.goal_at = time.time()

    def stationary(self):
        """Return True only on fresh IMU data showing no motion."""
        with self.lock:
            samples = [(g, a) for _, g, a in self.imu]
            fresh = time.time() - self.imu_at < 1.0
        return fresh and len(samples) > 50 and is_still(samples)

    def lio_drift(self, seconds=8.0):
        """Return how far FAST-LIO moved in the window, None without enough data."""
        now = time.time()
        with self.lock:
            points = [s[1:] for s in self.lio if now - s[0] < seconds]
        if len(points) < 20:
            return None
        return max(math.dist(p, points[0]) for p in points)

    def _client(self, node_name, kind, service):
        key = (node_name, service)
        if key not in self.clients:
            self.clients[key] = self.node.create_client(
                kind, '%s/%s' % (node_name, service))
        return self.clients[key]

    def _wait(self, future, timeout):
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def read(self, node_name, names, timeout=1.5):
        """Return {name: value} read from a node, or {} if it did not answer."""
        from rcl_interfaces.srv import GetParameters
        client = self._client(node_name, GetParameters, 'get_parameters')
        if not client.service_is_ready():
            return {}
        reply = self._wait(client.call_async(GetParameters.Request(names=names)),
                           timeout)
        if reply is None:
            return {}
        out = {}
        for name, value in zip(names, reply.values):
            if value.type == 3:
                out[name] = value.double_value
            elif value.type == 2:
                out[name] = float(value.integer_value)
        return out

    def write(self, node_name, name, number, timeout=2.0):
        """Set one double on a node. Return True if the node accepted it."""
        from rcl_interfaces.msg import Parameter, ParameterValue
        from rcl_interfaces.srv import SetParameters
        client = self._client(node_name, SetParameters, 'set_parameters')
        if not client.service_is_ready():
            return False
        request = SetParameters.Request(parameters=[Parameter(
            name=name, value=ParameterValue(type=3, double_value=float(number)))])
        reply = self._wait(client.call_async(request), timeout)
        return bool(reply and reply.results and reply.results[0].successful)

    def _poll(self):
        groups = {}
        for key, (node_name, name) in READBACK.items():
            groups.setdefault(node_name, []).append((key, name))
        while True:
            values = {}
            for node_name, pairs in groups.items():
                got = self.read(node_name, [n for _, n in pairs])
                for key, name in pairs:
                    if name in got:
                        values[key] = got[name]
            if self.mqtt:
                self.mqtt.publish('/missions/control',
                                  json.dumps({'action': 'get_state'}))
            limit = self.bridge_state.get('spreadLimit')
            if limit is not None and time.time() - self.bridge_state_at < STALE_S:
                values['localization_spread_limit'] = float(limit)
            if values:
                self.values = values
                self.values_at = time.time()
            time.sleep(2.0)


class Supervisor:
    """Own one stack and one bag; apply settings and prove they took."""

    def __init__(self, args, observer):
        self.args = args
        self.obs = observer
        self.lock = threading.Lock()
        self.busy = ''
        self.last_error = ''
        self.rearm_required = False
        os.makedirs(args.state_dir, exist_ok=True)
        self.profile_file = os.path.join(args.state_dir, 'profile.json')
        self.settings = self._load()
        self.armed_at = 0.0
        self.run_seen = False
        self.idle_since = None
        self.armed_run = None
        threading.Thread(target=self._guard_loop, daemon=True).start()

    def _guard_loop(self):
        """Disarm when the mission is over, and at once if FAST-LIO runs away."""
        # A mission already running when this starts was not started for it to
        # arm: starting the supervisor must never be what makes the robot move.
        time.sleep(STALE_S)
        self.armed_run = self.obs.bridge_state.get('runId')
        while True:
            time.sleep(0.25)
            if not self._bridge():
                self.run_seen = False
                self.idle_since = None
                if self.args.auto_arm:
                    self._arm_for_mission()
                continue
            speed = self.obs.lio_speed()
            if speed is not None and speed > RUNAWAY_MPS:
                self._emergency('FAST-LIO reports %.1f m/s' % speed)
                continue
            if self.mission_active():
                self.run_seen = True
                self.idle_since = None
                continue
            now = time.time()
            self.idle_since = self.idle_since or now
            idle = now - self.idle_since
            if (self.run_seen and idle > END_DISARM_S) or idle > IDLE_DISARM_S:
                self.last_error = 'bridge stopped: no mission running'
                self._stop_bridge()

    def _arm_for_mission(self):
        """Arm once for a mission the web started, or stop it and say why."""
        state = self.obs.bridge_state
        run_id = state.get('runId')
        fresh = time.time() - self.obs.bridge_state_at < STALE_S
        if (not fresh or not state.get('missionRunning') or state.get('paused')
                or state.get('stopping') or not run_id
                or run_id == self.armed_run):
            return
        self.armed_run = run_id
        try:
            self.arm(ARM_CONFIRM, for_mission=True)
        except ApplyError as exc:
            self.last_error = 'mission not started: ' + exc.message
            print('[stack_supervisor] ' + self.last_error, flush=True)
            if self.obs.mqtt:
                self.obs.mqtt.publish('/missions/control',
                                      json.dumps({'action': 'stop'}))

    def _emergency(self, reason):
        self.last_error = 'bridge stopped: ' + reason
        if self.obs.mqtt:
            self.obs.mqtt.publish('/missions/control',
                                  json.dumps({'action': 'stop'}))
        self._stop_bridge()

    def preflight(self, for_mission=False):
        """Return the reasons motion must not start, measured just now."""
        state = self._state()
        if for_mission:
            # The mission being armed for is the one that is running.
            state['mission_active'] = False
        return preflight_problems(state)

    def localization(self):
        """Return how far confirming the pose has got."""
        return localization_state(self._state())

    def _state(self):
        return ({
            'now': time.time(),
            'stack_up': bool(self._stack()),
            'stationary': self.obs.stationary(),
            'lio_speed': self.obs.lio_speed(),
            'pose_set_at': self.obs.pose_set_at,
            'position': self.obs.position,
            'position_at': self.obs.position_at,
            'mission_active': self.mission_active(),
            'other_lidar_user': bool(
                command_lines(r'.*/mid360_driver') +
                command_lines(r'\S*(python3?|bash) \S*go2_conf/')),
            'spread_limit': self.settings['localization_spread_limit'],
        })

    def arm(self, confirm, for_mission=False):
        if self.args.sim:
            raise ApplyError(409, 'simulation: there is no motion bridge to arm')
        """Start the bridge after checks. It moves the robot: stand, balance."""
        if confirm != ARM_CONFIRM:
            raise ApplyError(422, 'confirm must be ' + ARM_CONFIRM)
        if not self.lock.acquire(blocking=False):
            raise ApplyError(409, 'an apply is running: ' + self.busy)
        try:
            if self._bridge():
                return self.snapshot()
            problems = self.preflight(for_mission)
            if problems:
                raise ApplyError(409, '; '.join(problems))
            stamp = time.strftime('%Y%m%d_%H%M%S')
            log = os.path.join(self.args.log_dir, 'bridge_%s.log' % stamp)
            clean = {k: os.environ[k] for k in ('HOME', 'USER', 'LOGNAME')
                     if k in os.environ}
            clean['PATH'] = '/usr/local/bin:/usr/bin:/bin'
            with open(log, 'w') as out:
                subprocess.Popen(['bash', self.args.bridge_script], stdout=out,
                                 stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL, env=clean,
                                 start_new_session=True)
            deadline = time.time() + 25
            while time.time() < deadline:
                time.sleep(0.5)
                with open(log, errors='replace') as source:
                    text = source.read()
                if 'prereqs:' in text and self._bridge():
                    self.rearm_required = False
                    self.idle_since = None
                    self.last_error = ''
                    return self.snapshot()
                if 'No executable' in text or 'Traceback' in text:
                    break
            self._stop_bridge()
            raise ApplyError(503, 'bridge did not arm, see %s' % log)
        finally:
            self.lock.release()

    def disarm(self):
        """Stop the bridge. Always allowed: this is the safe direction."""
        self._stop_bridge()
        self.rearm_required = False
        return self.snapshot()

    def _load(self):
        try:
            with open(self.profile_file) as stream:
                return json.load(stream)
        except (OSError, ValueError):
            return defaults_from(self.args.share)

    def _save(self, settings):
        if os.path.exists(self.profile_file):
            shutil.copy2(self.profile_file, self.profile_file + '.previous')
        temp = self.profile_file + '.tmp'
        with open(temp, 'w') as stream:
            json.dump(settings, stream, indent=1, sort_keys=True)
        os.replace(temp, self.profile_file)

    # ------------------------------------------------------------ what is true

    def _bridge(self):
        return command_lines(r'/\S*python3?\S* /\S*(ros2 run go2_control |'
                             r'lib/go2_control/)unitree_udp_bridge')

    def mission_active(self):
        """Return True if a goal or a mission run is live, or it cannot be told."""
        obs = self.obs
        if obs.goal_active:
            return True
        stack_up = bool(self._stack())
        if not stack_up:
            return False
        if time.time() - obs.bridge_state_at > STALE_S:
            return True
        return bool(obs.bridge_state.get('missionRunning',
                                         obs.bridge_state.get('hasMission')))

    def snapshot(self):
        """Return the state the page reads. Raise ApplyError if it is unknown."""
        obs = self.obs
        values = dict(obs.values)
        armed = self._bridge()
        caps = next((c for c in (bridge_caps(t) for _, t in armed) if c), None)
        if caps:
            values['bridge_max_linear'], values['bridge_max_angular'] = caps
        else:
            values['bridge_max_linear'] = self.settings['bridge_max_linear']
            values['bridge_max_angular'] = self.settings['bridge_max_angular']
        if armed:
            self.rearm_required = False
        observed = obs.imu_at or obs.values_at
        if not observed:
            raise ApplyError(503, 'no data from the robot yet')
        warnings = []
        if self.busy:
            warnings.append('applying: ' + self.busy)
        if self.last_error:
            warnings.append(self.last_error)
        drift = obs.lio_drift()
        if drift is not None and drift > 5.0 and obs.stationary():
            warnings.append('FAST-LIO moved %.0f m while the IMU shows the '
                            'robot still: restart the stack' % drift)
        return {
            'schemaVersion': 1, 'role': 'admin',
            'revision': revision_of(self.settings),
            'observedAt': iso(min(observed, time.time())),
            'missionActive': self.mission_active(),
            'stationary': self._still(),
            'bridgeArmed': bool(armed),
            'rearmRequired': self.rearm_required,
            'values': {k: round(v, 4) for k, v in values.items()},
            'applyModes': {k: FIELDS[k][3] for k in values if k in FIELDS},
            'warnings': warnings,
        }

    # ------------------------------------------------------------ doing it

    def _still(self):
        # FAST-LIO needs a still robot to start; the simulation's stack has none.
        return self.args.sim or self.obs.stationary()

    def _stack(self):
        launch = 'sim_obstacle' if self.args.sim else 'onsite_stack'
        return command_lines(r'/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch '
                             r'go2_control ' + launch)

    def _start_sim(self, settings, files, log):
        """Restart the simulation with these settings instead of the robot's stack."""
        env = ('export GO2_SPREAD_LIMIT_M=%s DISPLAY=:0 '
               'XAUTHORITY=/run/user/%d/gdm/Xauthority;'
               % (settings['localization_spread_limit'], os.getuid()))
        self._spawn(['bash', self.args.sim_script, self.args.sim_room, '',
                     'nav2_params_file:=' + files['nav2'],
                     'amcl_params_file:=' + files['amcl']], log, env)
        deadline = time.time() + self.args.ready_timeout
        while time.time() < deadline:
            time.sleep(1)
            with open(log, errors='replace') as source:
                if source.read().count(READY_LINE) >= 2:
                    # The bridge reports the limit only once it has started.
                    time.sleep(5)
                    return ''
        return 'not ready after %d s, see %s' % (self.args.ready_timeout, log)

    def _stop_bag(self):
        for pid, _ in command_lines(r'/usr/bin/python3 /opt/ros/foxy/bin/ros2 '
                                    r'bag record'):
            signal_all([pid], signal.SIGINT)
            for _ in range(20):
                if not alive([pid]):
                    break
                time.sleep(0.5)
            signal_all(alive([pid]), signal.SIGKILL)

    def _stop_stack(self):
        self._stop_bag()
        for pid, _ in self._stack():
            family = descendants(pid)
            signal_all([pid], signal.SIGINT)
            time.sleep(6)
            signal_all(alive(family), signal.SIGTERM)
            time.sleep(3)
            signal_all(alive([pid] + family), signal.SIGKILL)

    def _stop_bridge(self):
        running = self._bridge()
        if not running:
            return False
        pids = [p for p, _ in running]
        signal_all(pids, signal.SIGINT)
        for _ in range(16):
            if not alive(pids):
                break
            time.sleep(0.25)
        signal_all(alive(pids), signal.SIGKILL)
        self.rearm_required = True
        return True

    def _write_bridge_script(self, settings):
        """Keep the operator's start script on the caps in the profile."""
        path = self.args.bridge_script
        if not os.path.exists(path):
            return
        with open(path) as stream:
            lines = stream.read().splitlines()
        out = []
        for line in lines:
            words = line.split()
            for flag, key in (('--max-linear', 'bridge_max_linear'),
                              ('--max-angular', 'bridge_max_angular')):
                if flag in words:
                    words[words.index(flag) + 1] = str(settings[key])
                    line = ' '.join(words)
            out.append(line)
        with open(path, 'w') as stream:
            stream.write('\n'.join(out) + '\n')

    def _spawn(self, argv, log_path, extra_env=''):
        command = '%s source %s >/dev/null 2>&1; exec %s' % (
            extra_env, shlex.quote(self.args.env_script),
            ' '.join(shlex.quote(a) for a in argv))
        clean = {k: os.environ[k] for k in ('HOME', 'USER', 'LOGNAME')
                 if k in os.environ}
        clean['PATH'] = '/usr/local/bin:/usr/bin:/bin'
        with open(log_path, 'w') as log:
            subprocess.Popen(['bash', '-c', command], stdout=log,
                             stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                             env=clean, start_new_session=True)

    def _map_yaml(self):
        env = self.args.map_yaml or os.environ.get('SITE_MAP', '')
        if not os.path.exists(env):
            raise ApplyError(503, 'map %r not found' % env)
        return env

    def _start_stack(self, settings):
        """Start the stack; return '' once it is proven up, else why not."""
        files = {}
        for name, builder, source in (
                ('amcl', amcl_config, 'amcl_livox.yaml'),
                ('nav2', nav2_config, self.args.nav2_base)):
            with open(os.path.join(self.args.share, 'config', source)) as stream:
                base = yaml.safe_load(stream)
            files[name] = os.path.join(self.args.state_dir, name + '.yaml')
            with open(files[name], 'w') as stream:
                yaml.safe_dump(builder(base, settings), stream)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        log = os.path.join(self.args.log_dir, 'onsite_stack_%s.log' % stamp)
        if self.args.sim:
            return self._start_sim(settings, files, log)
        env = 'export GO2_SPREAD_LIMIT_M=%s;' % settings[
            'localization_spread_limit']
        self._spawn(launch_args(settings, self._map_yaml(), files['amcl'],
                                files['nav2'], shlex.split(self.args.extra)),
                    log, env)
        deadline = time.time() + self.args.ready_timeout
        ready = False
        while time.time() < deadline and not ready:
            time.sleep(1)
            with open(log, errors='replace') as source:
                text = source.read()
            if 'process has died' in text:
                return 'a node died, see %s' % log
            ready = text.count(READY_LINE) >= 2
        if not ready:
            return 'not ready after %d s, see %s' % (self.args.ready_timeout, log)
        time.sleep(10)
        drift = self.obs.lio_drift()
        if drift is None:
            return 'no FAST-LIO pose after starting'
        if drift > 0.2:
            return 'FAST-LIO moved %.2f m while it should be still' % drift
        if self.args.bag_dir:
            self._spawn(['ros2', 'bag', 'record', '-o',
                         os.path.join(self.args.bag_dir, 'stack_' + stamp)]
                        + BAG_TOPICS,
                        os.path.join(self.args.log_dir, 'bag_%s.log' % stamp))
        return ''

    def _verify(self, settings, keys):
        """Return the keys whose live value differs from settings."""
        deadline = time.time() + 15
        if self.args.sim:
            # The simulation has no second scan for AMCL to read back from.
            keys = [k for k in keys if k in self.obs.values]
        wrong = list(keys)
        while time.time() < deadline and wrong:
            time.sleep(2.2)
            wrong = [k for k in keys if k in READBACK or
                     k == 'localization_spread_limit']
            wrong = [k for k in wrong if settings.get(k) is not None and abs(
                self.obs.values.get(k, float('inf')) - settings[k]) > 1e-4]
        return wrong

    def _apply_restart(self, settings, keys, previous):
        bridge_only = set(keys) <= BRIDGE_ONLY
        self.busy = 'stopping the bridge'
        self._stop_bridge()
        self._write_bridge_script(settings)
        self._save(settings)
        if bridge_only:
            self.settings = settings
            return
        self.busy = 'restarting the stack, keep the robot still'
        self._stop_stack()
        problem = self._start_stack(settings)
        if not problem:
            self.busy = 'reading values back'
            wrong = self._verify(settings, list(READBACK) +
                                 ['localization_spread_limit'])
            if wrong:
                problem = 'not applied: ' + ', '.join(wrong)
        if not problem:
            self.settings = settings
            return
        self.busy = 'restoring the previous settings'
        self._write_bridge_script(previous)
        self._save(previous)
        self.settings = previous
        self._stop_stack()
        again = self._start_stack(previous)
        raise ApplyError(503, '%s; %s' % (problem, 'previous settings restored, '
                                          'set the pose again' if not again else
                                          'previous settings failed too: ' + again))

    def _apply_live(self, settings, keys):
        applied = []
        for key in keys:
            node_name, name = READBACK[key]
            if not self.obs.write(node_name, name, settings[key]):
                for done in applied:
                    self.obs.write(*READBACK[done], self.settings[done])
                raise ApplyError(503, '%s was not accepted by %s' % (key, node_name))
            applied.append(key)
        wrong = self._verify(settings, keys)
        if wrong:
            for done in applied:
                self.obs.write(*READBACK[done], self.settings[done])
            raise ApplyError(503, 'not applied: ' + ', '.join(wrong))
        self._save(settings)
        self.settings = settings

    def patch(self, body):
        """Apply a request, proving it took. Return the new snapshot."""
        if not isinstance(body, dict):
            raise ApplyError(422, 'body must be an object')
        allow = body.get('allowRestart')
        if not isinstance(allow, bool):
            raise ApplyError(422, 'allowRestart must be true or false')
        if not self.lock.acquire(blocking=False):
            raise ApplyError(409, 'an apply is already running: ' + self.busy)
        try:
            if body.get('revision') != revision_of(self.settings):
                raise ApplyError(409, 'revision changed, read the settings again')
            merged, keys = validate_changes(body.get('changes'), self.settings,
                                            allow)
            self.busy = ''
            if any(FIELDS[k][3] == 'restart' for k in keys):
                if self.mission_active():
                    raise ApplyError(409, 'a mission or Nav2 goal is active')
                if not set(keys) <= BRIDGE_ONLY and not self._still():
                    raise ApplyError(409, 'the IMU does not show the robot still')
                self.last_error = ''
                try:
                    self._apply_restart(merged, keys, dict(self.settings))
                finally:
                    self.busy = ''
            else:
                self.last_error = ''
                self._apply_live(merged, keys)
            return self.snapshot()
        except ApplyError as exc:
            self.last_error = exc.message
            raise
        finally:
            self.lock.release()


def handler_for(supervisor, token):
    """Build the HTTP handler around one supervisor."""
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *_args):
            pass

        def _reply(self, status, body):
            data = json.dumps(body).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def _allowed(self):
            if not token:
                self._reply(403, {'message': 'STACK_SUPERVISOR_TOKEN is not set'})
                return False
            if self.headers.get('x-robot-settings-token') != token:
                self._reply(401, {'message': 'bad token'})
                return False
            return True

        def do_GET(self):
            length = int(self.headers.get('Content-Length', 0) or 0)
            if length:
                self.rfile.read(length)
            if self.path not in ('/robot/settings', '/robot/preflight'):
                self._reply(404, {'message': 'not found'})
            elif self._allowed():
                try:
                    if self.path == '/robot/preflight':
                        problems = supervisor.preflight()
                        self._reply(200, {'ok': not problems,
                                          'problems': problems,
                                          'localization':
                                          supervisor.localization()})
                    else:
                        self._reply(200, supervisor.snapshot())
                except ApplyError as exc:
                    self._reply(exc.status, {'message': exc.message})

        def do_POST(self):
            length = int(self.headers.get('Content-Length', 0) or 0)
            raw = self.rfile.read(length)
            if self.path != '/robot/bridge':
                self._reply(404, {'message': 'not found'})
                return
            if not self._allowed():
                return
            try:
                body = json.loads(raw)
                if not isinstance(body, dict):
                    raise ValueError
                if body.get('action') == 'arm':
                    self._reply(200, supervisor.arm(body.get('confirm')))
                elif body.get('action') == 'disarm':
                    self._reply(200, supervisor.disarm())
                else:
                    self._reply(422, {'message': 'action must be arm or disarm'})
            except ValueError:
                self._reply(400, {'message': 'not JSON'})
            except ApplyError as exc:
                self._reply(exc.status, {'message': exc.message})

        def do_PATCH(self):
            length = int(self.headers.get('Content-Length', 0) or 0)
            raw = self.rfile.read(length)
            if self.path != '/robot/settings':
                self._reply(404, {'message': 'not found'})
                return
            if not self._allowed():
                return
            try:
                body = json.loads(raw, parse_constant=lambda _: None)
                self._reply(200, supervisor.patch(body))
            except ValueError:
                self._reply(400, {'message': 'not JSON'})
            except ApplyError as exc:
                self._reply(exc.status, {'message': exc.message})
    return Handler


def main():
    """Serve until killed. The stack and bridge are left as they are on exit."""
    home = os.path.expanduser('~')
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--port', type=int, default=8091)
    parser.add_argument('--map-yaml', default='')
    parser.add_argument('--log-dir', default=os.path.join(home, 'go2_logs'))
    parser.add_argument('--bag-dir', default=os.path.join(home, 'bags'),
                        help='Empty to record nothing.')
    parser.add_argument('--state-dir',
                        default=os.path.join(home, 'stack_settings'))
    parser.add_argument('--env-script', default=os.path.join(
        home, 'go2_ws', 'scripts', 'setup_robot_env.sh'))
    parser.add_argument('--bridge-script',
                        default=os.path.join(home, 'start_bridge.sh'))
    parser.add_argument('--share', default='')
    parser.add_argument('--extra', default='replay:=false',
                        help='More launch arguments.')
    parser.add_argument('--ready-timeout', type=int, default=90)
    parser.add_argument('--nav2-base', default='nav2_livox_go2_obstacle.yaml',
                        help='Nav2 config the settings are written over.')
    parser.add_argument('--sim', action='store_true',
                        help='Beside the simulation: Apply restarts the '
                             'simulation, and the motion bridge cannot be armed.')
    parser.add_argument('--sim-room', default='meeting_room')
    parser.add_argument('--sim-script', default=os.path.join(
        home, 'projects/systonic-2307/Systronic/scripts/start_sim_web.sh'))
    parser.add_argument('--auto-arm', action='store_true',
                        help='Arm the bridge when the web starts a mission, '
                             'after preflight. Starting a mission then moves '
                             'the robot with no further confirmation.')
    args, _ros_args = parser.parse_known_args()
    if not args.share:
        from ament_index_python.packages import get_package_share_directory
        args.share = get_package_share_directory('go2_control')
    os.makedirs(args.log_dir, exist_ok=True)
    observer = Observer(os.environ.get('MQTT_BROKER', ''))
    supervisor = Supervisor(args, observer)
    token = os.environ.get('STACK_SUPERVISOR_TOKEN', '')
    server = ThreadingHTTPServer(('0.0.0.0', args.port),
                                 handler_for(supervisor, token))
    print('stack_supervisor on port %d' % args.port, flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
