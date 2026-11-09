#!/usr/bin/env python3
"""Take missions from the web over MQTT and hand them to Nav2.

The web interface already speaks a protocol, and main.py already implements it.
This node speaks the identical protocol - same topics, same JSON keys - so the
web side needs no change at all. What differs is underneath: main.py steers the
robot itself by publishing /cmd_vel, and this hands each waypoint to Nav2's
navigate_to_pose action instead.

That distinction is the whole reason this file exists. main.py was written
before there was a working Nav2 stack, so it had to drive. There is one now,
proven on the robot on 2026-09-07, and two mission executors fighting over the
same robot is not a design. Rather than rewrite main.py into something that
delegates - which is most of a rewrite, on a file carrying a lot of else -
this is the small piece that translates, and main.py can be retired when the
web has been seen to work against it.

## The protocol, as the web already uses it

    in   /missions/start     {missionId, runId, statusTopic, progressTopic,
                              imageTopic, tagPoses,
                              waypoints: [{sequence, x, y, yaw, isCapture,
                                           captureViews,
                                           name}]}
    in   /missions/control   {action: "pause" | "resume" | "stop"
                                      | "get_state" | "get_params"
                                      | "set_initial_pose", x, y, yaw
                                      | "set_params", params: {...},
                                        restart: bool}
    out  <statusTopic>       {runId, missionId, status, message}
    out  <progressTopic>     {runId, missionId, progress, currentWaypointIndex,
                              currentWaypointName, message,
                              live: {distanceRemaining, navigationTime,
                                     recoveries, yawDeg, yawTargetDeg,
                                     yawErrorDeg}}
    out  <imageTopic>        {runId, missionId, waypointName, takenAt, pose,
                              image}   - image is base64 JPEG
    out  /missions/map       {width, height, resolution, origin:{x,y,yaw},
                              image}   - base64 PNG, occupied pixels dark
    out  /missions/pose      {x, y, yaw, frame, localizationSpread,
                              timestamp}
    out  /missions/params    {values, limits, readonly,
                              clamped_by_bridge, notes, timestamp}
    out  /missions/state     {hasMission, runId, missionId, paused, stopping,
                              waypointCount, hasMap, statusTopic,
                              progressTopic, imageTopic, timestamp}

The reply topics are named by the web inside the mission payload rather than
being fixed here, which is how main.py works and is worth keeping: one robot
can serve several runs without either side agreeing on names in advance.

## What it does not do

It does not publish velocity. It has no publisher on /cmd_vel or
/cmd_vel_safe, so no bug in it can move the robot; the only way it causes
motion is by asking Nav2 for a goal, which goes through the controller, the
watchdog and the bridge exactly as a goal from send_mission does.

    ros2 run go2_control mqtt_mission_bridge --ros-args -p broker:=192.168.68.62
"""

import base64
import collections
import json
import math
import os
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import tf2_ros

from go2_control.nav_ready_check import SPREAD_LIMIT_M


# ------------------------------------------------------------- parameters

Tunable = collections.namedtuple('Tunable', 'node name applies kind')

# The values the web may set: which node owns each, its full parameter name,
# whether the running node picks a change up on its own, and its type.
#
# 'live' means the node subscribes to parameter events and re-reads. On Foxy
# the only one that does is dwb_plugins::KinematicsHandler, which holds the
# speed and acceleration limits, so those four take effect on the next
# control period.
#
# 'restart' means the node read the value once - at configure, or at
# onInitialize - into its own members and will never look again. A set call
# still succeeds and `ros2 param get` returns the new number while the robot
# goes on using the old one. Those are applied by taking the owning node
# round its lifecycle, which is what "restart": true on /missions/control
# asks for.
#
# The AMCL group is the one measured: /amcl went round deactivate, cleanup,
# configure, activate in 18 ms on a replayed stack on 2026-09-11, with
# lifecycle_manager not reacting and the pose coming back by itself through
# save_pose_rate. controller_server has not had the same test.
TUNABLE = {
    'max_vel_x': Tunable(
        '/controller_server', 'FollowPath.max_vel_x', 'live', float),
    'max_vel_theta': Tunable(
        '/controller_server', 'FollowPath.max_vel_theta', 'live', float),
    'acc_lim_x': Tunable(
        '/controller_server', 'FollowPath.acc_lim_x', 'live', float),
    'acc_lim_theta': Tunable(
        '/controller_server', 'FollowPath.acc_lim_theta', 'live', float),

    'xy_goal_tolerance': Tunable(
        '/controller_server', 'goal_checker.xy_goal_tolerance', 'restart',
        float),
    'yaw_goal_tolerance': Tunable(
        '/controller_server', 'goal_checker.yaw_goal_tolerance', 'restart',
        float),
    'inflation_radius': Tunable(
        '/local_costmap/local_costmap', 'inflation_layer.inflation_radius',
        'restart', float),
    'cost_scaling_factor': Tunable(
        '/local_costmap/local_costmap', 'inflation_layer.cost_scaling_factor',
        'restart', float),

    'update_min_d': Tunable('/amcl', 'update_min_d', 'restart', float),
    'update_min_a': Tunable('/amcl', 'update_min_a', 'restart', float),
    'min_particles': Tunable('/amcl', 'min_particles', 'restart', int),
    'max_particles': Tunable('/amcl', 'max_particles', 'restart', int),
    'laser_max_range': Tunable('/amcl', 'laser_max_range', 'restart', float),
    'transform_tolerance': Tunable(
        '/amcl', 'transform_tolerance', 'restart', float),
}

# The node to cycle to make each owner's restart values take effect. The
# local costmap is not cycled on its own: its lifecycle is driven by the
# controller that constructed it, and transitioning it from outside would
# leave the two disagreeing about what state it is in.
CYCLE_OWNER = {
    '/controller_server': '/controller_server',
    '/local_costmap/local_costmap': '/controller_server',
    '/amcl': '/amcl',
}

# Bounds that are not a matter of taste. A request outside them is clamped
# and the reply says so, rather than being refused silently or accepted into
# a robot that then behaves badly.
#
#   xy_goal_tolerance   0.35 is the floor because localisation is 0.258 m
#                       mean and 0.494 m worst over the measured walk. At
#                       0.25 the robot circled a waypoint it had reached for
#                       ten seconds and needed a recovery. This floor drops
#                       when AprilTag lowers the error, not before.
#   max_vel_theta       under 0.2 the wheel base may not turn at all.
#   acc_lim_x           under 0.2 DWB re-plans before the robot has reached
#                       the speed it asked for.
#   update_min_d        0.05 is the measurement setting: /amcl_pose is only
#                       published on update, so a stopped robot holds a pose
#                       up to update_min_d metres stale, which at the shipped
#                       0.20 is the size of the error being measured.
#   max_particles       the ceiling is a CPU limit, not a quality one. The
#                       board has never been measured under this stack.
LIMITS = {
    'max_vel_x': (0.05, 0.60),
    'max_vel_theta': (0.20, 0.80),
    'acc_lim_x': (0.20, 1.00),
    'acc_lim_theta': (0.20, 1.50),
    'xy_goal_tolerance': (0.35, 1.00),
    'yaw_goal_tolerance': (0.10, 1.00),
    'inflation_radius': (0.10, 1.00),
    'cost_scaling_factor': (1.0, 10.0),
    'update_min_d': (0.02, 0.50),
    'update_min_a': (0.02, 0.50),
    'min_particles': (100, 2000),
    'max_particles': (500, 10000),
    'laser_max_range': (5.0, 40.0),
    'transform_tolerance': (0.10, 2.00),
}


def _clamp(name, value):
    """Return (value, note) with value forced inside LIMITS[name]."""
    low, high = LIMITS[name]
    if value < low:
        return low, '%s raised to the floor %s' % (name, low)
    if value > high:
        return high, '%s lowered to the ceiling %s' % (name, high)
    return value, None


def yaw_to_quaternion(yaw):
    import math
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


SCAN_MAX_POINTS = 240
# A page repeats want_scan more often than this while it shows the scan.
SCAN_WANTED_S = 10.0


def turn_needed(yaw, dx, dy):
    """Radians to turn, shortest way, to face a point dx, dy away."""
    bearing = math.atan2(dy, dx)
    return math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))


class GuardHold:
    """How long the collision guard has been refusing commands, without a break.

    Going by the count as well as the flag: the guard's flag stays set after it
    last refused until a command passes, so a mission paused with nothing being
    sent would otherwise look held.
    """

    def __init__(self):
        self.since = None
        self.last_count = 0

    def update(self, stopped, stop_commands, now):
        refusing = bool(stopped) and stop_commands > self.last_count
        self.last_count = stop_commands
        if not refusing:
            self.since = None
        elif self.since is None:
            self.since = now

    def seconds(self, now):
        return 0.0 if self.since is None else now - self.since


def scan_points(scan, x, y, yaw, max_range):
    """A LaserScan's returns within max_range, as (x, y) in the frame of (x, y, yaw).

    inf is "nothing there" and is left out, which is why this reads the scan
    and not the costmap: Nav2 does not clear a cell along a ray that returns
    inf, so a costmap keeps something that has gone for as long as nothing
    stands behind where it was.
    """
    points = []
    for index, distance in enumerate(scan.ranges):
        if not scan.range_min <= distance <= min(scan.range_max, max_range):
            continue
        angle = yaw + scan.angle_min + index * scan.angle_increment
        points.append((x + distance * math.cos(angle),
                       y + distance * math.sin(angle)))
    return points


def new_obstacle_near(points, static_map, x, y, radius, wall_margin=0.30,
                      min_points=3):
    """Say whether something not on the map is within radius of a point.

    points is the latest scan in the frame of the map, an OccupancyGrid. A point
    within wall_margin of a map wall counts as that wall, because the scan lands
    off the drawn wall by the localisation error, 0.3 m on average here, and
    that alone must not end a mission. min_points keeps one stray return from
    doing it either. None means there is no scan or map, so nothing can be said.
    """
    if points is None or static_map is None:
        return None
    info = static_map.info
    res = info.resolution
    margin = int(round(wall_margin / res))

    def on_wall(px, py):
        col = int(math.floor((px - info.origin.position.x) / res))
        row = int(math.floor((py - info.origin.position.y) / res))
        for r in range(max(row - margin, 0), min(row + margin + 1, info.height)):
            for c in range(max(col - margin, 0),
                           min(col + margin + 1, info.width)):
                if static_map.data[r * info.width + c] >= 65:
                    return True
        return False

    count = 0
    for px, py in points:
        if math.hypot(px - x, py - y) <= radius and not on_wall(px, py):
            count += 1
            if count >= min_points:
                return True
    return False


class MqttMissionBridge(Node):

    def __init__(self):
        super().__init__('mqtt_mission_bridge')

        self.broker = self.declare_parameter(
            'broker', os.environ.get('MQTT_BROKER', '192.168.68.62')).value
        self.port = int(self.declare_parameter('port', 1883).value)
        self.frame = self.declare_parameter('frame_id', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        # Seconds to hold at a capture point before asking for the photograph.
        # Nav2 reports success the moment the goal tolerance is met, while the
        # robot is still settling, and a photograph taken then is smeared.
        self.capture_settle = float(
            self.declare_parameter('capture_settle', 3.0).value)
        # Photographs per capture point, turning in place between them.
        # 1 is the original behaviour. Overridden per waypoint by captureViews.
        self.declare_parameter('capture_views', 1)
        self.capture_timeout = float(
            self.declare_parameter('capture_timeout', 20.0).value)
        self.pose_period = float(
            self.declare_parameter('pose_period', 1.0).value)
        self.pose_topic = self.declare_parameter(
            'pose_topic', '/missions/pose').value
        self.map_topic = self.declare_parameter(
            'map_topic', '/missions/map').value
        # The map does not change while the robot runs, so it goes out once
        # when it arrives and then only every map_period seconds, for the
        # benefit of a browser that was opened afterwards. MQTT has no
        # retained-by-default, and a web page that missed the single
        # publication has no map and no way to ask for one.
        self.map_period = float(
            self.declare_parameter('map_period', 30.0).value)

        # What unitree_udp_bridge was started with. It is a separate process
        # on a separate DDS domain and its clamp is a command line argument,
        # not a ROS parameter, so nothing can read it back - this node has to
        # be told. Set both from onsite.env, from the same values the bridge
        # command line uses, or the panel reports a ceiling the robot does
        # not have. On 2026-09-07 Nav2 was set to 0.40 while the bridge cut
        # at 0.25 and no part of the system said so; the robot simply ran
        # slower than every number on screen.
        self.bridge_max_linear = float(self.declare_parameter(
            'bridge_max_linear',
            float(os.environ.get('BRIDGE_MAX_LINEAR', 0.45))).value)
        self.bridge_max_angular = float(self.declare_parameter(
            'bridge_max_angular',
            float(os.environ.get('BRIDGE_MAX_ANGULAR', 0.60))).value)

        self.params_topic = self.declare_parameter(
            'params_topic', '/missions/params').value
        self.state_topic = self.declare_parameter(
            'state_topic', '/missions/state').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Nav2's own spin, so a turn goes through the watchdog and collision
        # guard like any other Nav2 command and this node still publishes no
        # velocity.
        self.spin = ActionClient(self, Spin, 'spin')
        self.turn_before_goal = bool(self.declare_parameter(
            'turn_before_goal', True).value)
        self.turn_min_angle = math.radians(float(self.declare_parameter(
            'turn_min_angle_deg', 60.0).value))
        self.turn_timeout = float(self.declare_parameter(
            'turn_timeout_seconds', 20.0).value)
        self.turn_skip_within = float(self.declare_parameter(
            'turn_skip_within', 0.25).value)
        self._last_live = None

        self.capture_pub = self.create_publisher(
            String, '/mission/capture_request', 10)

        # Setting the initial pose from the web. /initialpose is what RViz's
        # 2D Pose Estimate publishes and what AMCL listens to; this is the
        # same message from the same place, so the robot cannot tell the
        # difference. It is a pose, not a velocity - nothing here moves the
        # robot.
        from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
        from rclpy.qos import QoSProfile as _Qos, ReliabilityPolicy as _Rel
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',
            self._on_initial_pose, 10)
        # The particle cloud's spread is how the web knows whether a pose it
        # sent has taken: AMCL scatters the particles over the covariance
        # given and pulls them in as the scan agrees with the map. Best
        # effort, matching AMCL - a reliable subscription receives nothing.
        self.spread = None
        pqos = _Qos(depth=1)
        pqos.reliability = _Rel.BEST_EFFORT
        self.create_subscription(
            PoseArray, '/particlecloud', self._on_particles, pqos)
        # The same limit and switch as nav_ready_check, which only guards
        # send_mission. A mission from the web came straight here unchecked.
        self.enforce_spread_limit = os.environ.get(
            'GO2_ENFORCE_SPREAD_LIMIT', 'true').lower() not in (
                'false', '0', 'no')
        self.localization_check = None
        self.create_subscription(
            String, '/localization_watch/status',
            self._on_localization_check, 10)
        self.create_subscription(
            String, '/mission/capture_done', self._on_capture_done, 10)
        self.capture_reply = None

        # Mission state. Only the MQTT thread writes the request fields and
        # only the mission thread reads them, with `lock` over the handover.
        self.lock = threading.Lock()
        self.mission = None
        self.paused = False
        self.stopping = False
        self.last_status = None
        self.last_progress = None
        self._last_feedback = 0.0
        self._live_goal_yaw = None
        self.goal_handle = None
        self.worker = None
        self.last_run_id = None

        # Made here rather than on demand: the MQTT thread is the only
        # caller and a client created from it, after rclpy.spin has the
        # executor, is not picked up until something else wakes it.
        from lifecycle_msgs.srv import ChangeState
        from rcl_interfaces.srv import GetParameters, SetParameters
        self._get_cli = {}
        self._set_cli = {}
        for node_name in sorted({t.node for t in TUNABLE.values()}):
            self._get_cli[node_name] = self.create_client(
                GetParameters, node_name + '/get_parameters')
            self._set_cli[node_name] = self.create_client(
                SetParameters, node_name + '/set_parameters')
        # Every restart parameter is owned by controller_server, directly or
        # through the local costmap it constructs, so one node takes them
        # all. The costmap is not cycled on its own: its lifecycle is driven
        # by the controller, and transitioning it from outside would leave
        # the two disagreeing about what state it is in.
        self._cycle_cli = {}
        for owner in sorted(set(CYCLE_OWNER.values())):
            self._cycle_cli[owner] = self.create_client(
                ChangeState, owner + '/change_state')

        self.mqtt = None
        self._connect_mqtt()

        self.create_timer(self.pose_period, self._publish_pose)
        self.scan_topic = str(self.declare_parameter(
            'scan_topic', '/missions/scan').value)
        self.create_timer(0.5, self._publish_scan)

        # Subscribed transient local, because map_server publishes /map once
        # when it activates and latches it - a volatile subscriber connecting
        # afterwards, which this always is, receives nothing at all.
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_payload = None
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map, map_qos)
        self.static_grid = None
        self.create_subscription(
            OccupancyGrid, '/map',
            lambda msg: setattr(self, 'static_grid', msg), map_qos)
        # True once anyone sets a pose (web, RViz, AprilTag). A cloud alone does
        # not count: at startup AMCL has a tight cloud around its default pose.
        self.pose_set = False
        self.scan_until = 0.0
        self.last_scan = None
        self.last_scan_at = None
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.map_wall_margin = float(self.declare_parameter(
            'map_wall_margin', 0.30).value)
        # What the costmap's obstacle layer is set to mark.
        self.obstacle_range = float(self.declare_parameter(
            'obstacle_range', 6.0).value)
        self.obstacle_wait = float(self.declare_parameter(
            'obstacle_wait_seconds', 30.0).value)
        # The body's corners sweep 0.43 m, so it does not fit at the waypoint
        # with anything nearer. The scan sees only the near face of an obstacle,
        # which is further from the waypoint the bigger the obstacle is.
        self.obstacle_radius = float(self.declare_parameter(
            'obstacle_radius', 0.50).value)
        # Close to a goal the robot's own returns land near it too.
        self.obstacle_ignore_within = float(self.declare_parameter(
            'obstacle_ignore_within', 0.6).value)
        self.fail_reason = None
        self.blocked_during_goal = False
        # Nav2's spin and backup recoveries have no timeout, so a robot the
        # collision guard is holding would wait on them for as long as the
        # thing stays there, with the web still saying it was on its way.
        self.guard_hold = GuardHold()
        self.create_subscription(
            String, '/collision_guard/status', self._on_guard_status, 10)
        self.create_timer(self.map_period, self._publish_map)

        self.get_logger().info(
            'mqtt_mission_bridge ready - broker %s:%d, goals go to Nav2, '
            'this node publishes no velocity' % (self.broker, self.port))

    # -------------------------------------------------------------- mqtt

    def _connect_mqtt(self):
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            self.get_logger().error('paho-mqtt is not installed')
            return
        self.mqtt = mqtt.Client()
        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_message = self._on_message
        self.mqtt.reconnect_delay_set(min_delay=1, max_delay=5)
        self.mqtt.on_disconnect = self._on_disconnect
        # connect_async and then loop_start, unconditionally. The previous
        # code called connect() first and loop_start() only if that
        # succeeded, while the log said it would retry - it did not. paho
        # only reconnects from inside a running loop, so one refused or
        # timed-out first attempt left the bridge permanently offline with
        # the node alive and every other check passing. On site on
        # 2026-09-14 the board's Wi-Fi was dropping in and out, the first
        # attempt fell in a gap, and the web had no map, no pose and no way
        # to send a mission for the rest of the run.
        try:
            self.mqtt.connect_async(self.broker, self.port, 60)
        except Exception as exc:                   # noqa: BLE001
            self.get_logger().error('MQTT connect_async failed: %s' % exc)
        self.mqtt.loop_start()

    def _on_disconnect(self, _client, _userdata, rc):
        if rc != 0:
            self.get_logger().warn(
                'MQTT disconnected rc=%s - reconnecting on its own' % rc)

    def _on_connect(self, client, _userdata, _flags, rc):
        self.get_logger().info('MQTT connected rc=%s' % rc)
        client.subscribe('/missions/start')
        client.subscribe('/missions/control')
        # A page opened after the robot has been up would otherwise show an
        # empty map, no robot, and empty sliders until something changed.
        self._publish_state()

    def _publish(self, topic, payload):
        if not topic or self.mqtt is None:
            return
        try:
            self.mqtt.publish(topic, json.dumps(payload))
        except Exception as exc:                   # noqa: BLE001
            self.get_logger().warn('publish to %s failed: %s' % (topic, exc))

    def _on_message(self, _client, _userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            if isinstance(data, str):
                data = json.loads(data)
        except ValueError as exc:
            self.get_logger().error('unreadable MQTT payload: %s' % exc)
            return

        if msg.topic == '/missions/start':
            # A retained or redelivered start must not move the robot again.
            run_id = data.get('runId')
            if msg.retain or (run_id and run_id == self.last_run_id):
                self.get_logger().warn('ignored a repeated mission start')
                return
            self.last_run_id = run_id
            self._start_mission(data)
        elif msg.topic == '/missions/control':
            self._control(data)

    def _control(self, data):
        action = data.get('action', '')
        if action == 'pause':
            self.paused = True
            self._cancel_current()
            self.get_logger().warn('paused by the web')
        elif action == 'resume':
            self.paused = False
            self.get_logger().info('resumed by the web')
        elif action == 'stop':
            self.stopping = True
            self.paused = False
            self._cancel_current()
            self.get_logger().warn('stopped by the web')
        elif action == 'set_initial_pose':
            self._set_initial_pose(data)
        elif action == 'want_scan':
            self.scan_until = time.monotonic() + SCAN_WANTED_S
        elif action == 'get_state':
            self._publish_state()
        elif action == 'get_params':
            self._publish_params()
        elif action == 'set_params':
            self._set_params(data.get('params') or {},
                             restart=bool(data.get('restart')))
        else:
            self.get_logger().warn('unknown control action %r' % action)

    def _publish_state(self):
        """Everything a page that just opened has missed, sent at once.

        MQTT delivers what is published while you are subscribed and nothing
        before it, and the robot's state is published on its own schedule:
        the map every 30 seconds, status only when it changes. A browser
        opened in between had no way to ask, so it showed an empty map and an
        idle robot while a mission was running. This is that way.

        Everything here is a republication of what the robot already sends on
        the normal topics, on the normal topics, so the web needs no second
        code path to handle it - the same subscriber that draws the map
        during a run draws it here.
        """
        self._publish_map()
        self._publish_pose()
        self._publish_params()
        m = self.mission or {}
        if self.last_status:
            # Without its message: this repeat would overwrite the newer text
            # the web already has from progress, such as "Turning toward".
            self._publish(m.get('statusTopic'),
                          {k: v for k, v in self.last_status.items()
                           if k != 'message'})
        if self.last_progress:
            self._publish(m.get('progressTopic'), self.last_progress)
        # Sent even when nothing is running, so the page can tell "no mission"
        # apart from "the robot did not answer". paused is not a status of its
        # own in the protocol, and a page cannot infer it from RUNNING alone.
        self._publish(self.state_topic, {
            'hasMission': bool(self.mission),
            # hasMission outlives the run; this is whether one is going.
            'missionRunning': bool(self.worker is not None
                                   and self.worker.is_alive()),
            'spreadLimit': SPREAD_LIMIT_M,
            'runId': m.get('runId'),
            'missionId': m.get('missionId'),
            'paused': bool(self.paused),
            'stopping': bool(self.stopping),
            'waypointCount': len(m.get('waypoints') or []),
            'hasMap': self.map_payload is not None,
            'statusTopic': m.get('statusTopic'),
            'progressTopic': m.get('progressTopic'),
            'imageTopic': m.get('imageTopic'),
            'timestamp': int(time.time()),
        })

    # ----------------------------------------------------------- parameters

    def _wait(self, future, timeout=3.0):
        """Wait for a service future from the MQTT thread.

        The executor runs in the main thread and resolves the future there,
        so this only watches the clock. It must never spin: two threads
        spinning one node is how the mission thread used to deadlock.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.02)
        return None

    def _read_params(self):
        """Current value of every tunable, by asking the node that owns it.

        Read back from the nodes rather than remembered here, because a
        value this node set is not proof of the value the controller is
        using - the yaml, a relaunch or send_mission can all have moved it.
        """
        from rcl_interfaces.srv import GetParameters
        values = {}
        for node_name, cli in self._get_cli.items():
            names = [t.name for t in TUNABLE.values()
                     if t.node == node_name]
            keys = [k for k, t in TUNABLE.items() if t.node == node_name]
            if not cli.service_is_ready():
                self.get_logger().warn(
                    '%s is not up - its parameters are reported as unknown'
                    % node_name)
                continue
            req = GetParameters.Request()
            req.names = names
            res = self._wait(cli.call_async(req))
            if res is None or len(res.values) != len(keys):
                self.get_logger().warn('no reply from %s' % node_name)
                continue
            for key, value in zip(keys, res.values):
                values[key] = (value.integer_value
                               if TUNABLE[key].kind is int
                               else value.double_value)
        return values

    def _cycle(self, owner):
        """Take one node round its lifecycle so it re-reads its parameters.

        The goal checker's tolerances, the inflation layer's radius and
        every AMCL value here are read once - at configure, or at
        onInitialize - into members that nothing updates afterwards. This is
        the cheap way to make them take effect.

        Measured against /amcl on a replayed stack on 2026-09-11: the four
        transitions took 18 ms, lifecycle_manager did not react at all - no
        bond break, no restart of the managed set - and the pose came back on
        its own, because AMCL writes it into initial_pose.* twice a second
        under save_pose_rate and seeds from there when it configures.

        **controller_server has not had that test.** AMCL is a leaf; the
        controller owns a costmap and an action server, and a client mid-goal
        when this happens is a case nothing here has seen. That is why a
        running mission is refused outright rather than paused around.
        """
        from lifecycle_msgs.msg import Transition
        from lifecycle_msgs.srv import ChangeState

        if self.mission is not None and not self.stopping:
            return ['a mission is running - refusing to cycle %s. Stop the '
                    'mission first.' % owner]
        cli = self._cycle_cli.get(owner)
        if cli is None or not cli.service_is_ready():
            return ['%s is not up - nothing was cycled' % owner]

        notes = []
        for label, transition in (
                ('deactivate', Transition.TRANSITION_DEACTIVATE),
                ('cleanup', Transition.TRANSITION_CLEANUP),
                ('configure', Transition.TRANSITION_CONFIGURE),
                ('activate', Transition.TRANSITION_ACTIVATE)):
            req = ChangeState.Request()
            req.transition = Transition(id=transition)
            res = self._wait(cli.call_async(req), timeout=15.0)
            if res is None or not res.success:
                # Half a cycle is worse than none: the node is left inactive
                # and will not do its job until someone finishes the sequence
                # by hand. Say FAILED rather than reporting a number.
                notes.append(
                    '%s %s FAILED - it is not active and will stay that way '
                    'until someone brings it back' % (owner, label))
                self.get_logger().error(notes[-1])
                return notes
        # Not logged here: _set_params logs every note it collects, and a
        # success line printed in both places reads on site as two cycles.
        notes.append('%s cycled - its restart values are now in use' % owner)
        return notes

    def _set_params(self, requested, restart=False):
        """Apply what the web asked for, clamped, and report what happened."""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters

        notes = []
        by_node = {}
        for key, raw in requested.items():
            if key not in TUNABLE:
                notes.append('%s is not adjustable and was ignored' % key)
                continue
            try:
                value = float(raw)
            except (TypeError, ValueError):
                notes.append('%s was not a number and was ignored' % key)
                continue
            value, note = _clamp(key, value)
            if note:
                notes.append(note)
            spec = TUNABLE[key]
            param = Parameter()
            param.name = spec.name
            param.value = ParameterValue()
            # The type has to match what the node declared. A double sent to
            # an integer parameter is refused with a type error, which is how
            # min_particles would have failed had it gone out as a double.
            if spec.kind is int:
                param.value.type = ParameterType.PARAMETER_INTEGER
                param.value.integer_value = int(round(value))
            else:
                param.value.type = ParameterType.PARAMETER_DOUBLE
                param.value.double_value = value
            by_node.setdefault(spec.node, []).append(param)

        for node_name, params in by_node.items():
            cli = self._set_cli[node_name]
            if not cli.service_is_ready():
                notes.append('%s is not up - nothing was changed there'
                             % node_name)
                continue
            req = SetParameters.Request()
            req.parameters = params
            res = self._wait(cli.call_async(req))
            if res is None:
                notes.append('%s did not answer within 3s' % node_name)
                continue
            for param, result in zip(params, res.results):
                if not result.successful:
                    notes.append('%s refused: %s' % (
                        param.name, result.reason or 'no reason given'))

        # The particle counts are the one pair where each value can be
        # inside its own bounds and the pair still nonsense. AMCL accepts
        # min > max and then behaves unpredictably, so it is caught here.
        lo = requested.get('min_particles')
        hi = requested.get('max_particles')
        if lo is not None and hi is not None:
            try:
                if float(lo) > float(hi):
                    notes.append('min_particles above max_particles - both '
                                 'ignored')
                    by_node = {n: [p for p in ps
                                   if p.name not in ('min_particles',
                                                     'max_particles')]
                               for n, ps in by_node.items()}
            except (TypeError, ValueError):
                pass

        stale = sorted({k for k in requested
                        if k in TUNABLE and TUNABLE[k].applies == 'restart'})
        if stale and not restart:
            owners = sorted({CYCLE_OWNER[TUNABLE[k].node] for k in stale})
            notes.append(
                'stored but NOT in use until %s cycled: %s'
                % (' and '.join(owners) + (' are' if len(owners) > 1
                                           else ' is'), ', '.join(stale)))
        elif stale and restart:
            for owner in sorted({CYCLE_OWNER[TUNABLE[k].node] for k in stale}):
                notes.extend(self._cycle(owner))
        elif restart and not stale:
            notes.append('nothing asked for needed a restart - not cycling')

        for note in notes:
            self.get_logger().warn(note)
        # Always answer with what the nodes hold now, not with what was
        # asked for. The web must draw the robot's values, never its own.
        self._publish_params(notes)

    def _publish_params(self, notes=None):
        values = self._read_params()
        effective = min(values.get('max_vel_x', self.bridge_max_linear),
                        self.bridge_max_linear)
        effective_ang = min(
            values.get('max_vel_theta', self.bridge_max_angular),
            self.bridge_max_angular)
        payload = {
            'values': values,
            'limits': {k: {'min': lo, 'max': hi}
                       for k, (lo, hi) in LIMITS.items()},
            # 'live' takes effect on the next control period. 'restart' is
            # stored, is visible to ros2 param get, and is NOT used by the
            # running robot until the stack is relaunched. The panel has to
            # say so; a slider that moves and changes nothing is worse than
            # no slider.
            'applies': {k: t.applies for k, t in TUNABLE.items()},
            # Which node has to be cycled for each restart value,
            # so the panel can group them: one Apply per node
            # rather than one per slider.
            'cycles': {k: CYCLE_OWNER[t.node]
                       for k, t in TUNABLE.items()
                       if t.applies == 'restart'},
            'readonly': {
                'bridge_max_linear': self.bridge_max_linear,
                'bridge_max_angular': self.bridge_max_angular,
                'effective_max_linear': effective,
                'effective_max_angular': effective_ang,
            },
            # True when Nav2 is set above what the bridge will pass. The
            # robot then runs at the bridge's number and every Nav2 reading
            # on the panel is wrong. The web is expected to show this.
            'clamped_by_bridge': (
                values.get('max_vel_x', 0.0) > self.bridge_max_linear
                or values.get('max_vel_theta', 0.0) > self.bridge_max_angular),
            'notes': notes or [],
            'timestamp': int(time.time()),
        }
        self._publish(self.params_topic, payload)

    def _cancel_current(self):
        handle = self.goal_handle
        if handle is not None:
            handle.cancel_goal_async()

    # ----------------------------------------------------------- mission

    def _start_mission(self, data):
        # Stop whatever is running before starting the next one, rather than
        # refusing. A mission stopped from the web leaves this thread finishing
        # its cancel, and a bare is_alive() check then rejected the very next
        # mission the operator sent - so the web appeared to stop responding
        # until someone waited long enough, with nothing saying why.
        if self.worker is not None and self.worker.is_alive():
            self.get_logger().warn('a mission is running - stopping it first')
            self.stopping = True
            self.paused = False
            self._cancel_current()
            self.worker.join(timeout=10.0)
            if self.worker.is_alive():
                self.get_logger().error(
                    'the previous mission did not stop within 10s - refusing '
                    'the new one rather than running two at once')
                return

        waypoints = sorted(data.get('waypoints', []),
                           key=lambda w: w.get('sequence', 0))
        if not waypoints:
            self.get_logger().error('mission has no waypoints')
            return

        with self.lock:
            self.mission = {
                'missionId': data.get('missionId'),
                'runId': data.get('runId'),
                'statusTopic': data.get('statusTopic'),
                'progressTopic': data.get('progressTopic'),
                'imageTopic': data.get('imageTopic'),
                'waypoints': waypoints,
            }
        # Spread is None until the robot has moved about 0.2 m after a pose is
        # set, because AMCL publishes no cloud before that. Refusing then would
        # refuse every first mission, so only a measured wide cloud stops it.
        if (self.enforce_spread_limit and self.spread is not None
                and self.spread > SPREAD_LIMIT_M):
            note = ('localization spread %.2f m is above %.2f m - set the pose '
                    'again or move the robot until it settles'
                    % (self.spread, SPREAD_LIMIT_M))
            self.get_logger().warn('mission refused: ' + note)
            self._status('FAILED', note)
            # Refused is not started: the same run may be sent again.
            self.last_run_id = None
            with self.lock:
                self.mission = None
            return
        if self.spread is None:
            self.get_logger().warn(
                'starting with no spread measured yet - AMCL has not updated '
                'since the pose was set')
        self.paused = False
        self.stopping = False

        self.worker = threading.Thread(target=self._run_mission, daemon=True)
        self.worker.start()

    def _status(self, status, message=''):
        m = self.mission or {}
        payload = {'runId': m.get('runId'), 'missionId': m.get('missionId'),
                   'status': status, 'message': message}
        # Kept so a page that opened mid-mission can be told where things
        # stand. Status is otherwise only sent when it changes, which for a
        # long leg is minutes apart, and a browser refreshed in that gap sees
        # a running robot as an idle one.
        self.last_status = payload
        self._publish(m.get('statusTopic'), payload)

    def _progress(self, index, name, percent, message=''):
        m = self.mission or {}
        payload = {'runId': m.get('runId'), 'missionId': m.get('missionId'),
                   'progress': percent, 'currentWaypointIndex': index,
                   'currentWaypointName': name, 'message': message}
        self.last_progress = payload
        self._publish(m.get('progressTopic'), payload)

    def _run_mission(self):
        mission = self.mission
        waypoints = mission['waypoints']
        total = len(waypoints)

        self._status('PENDING', 'Mission accepted')
        self._progress(0, '', 0, 'Mission queued')

        if not self.nav.wait_for_server(timeout_sec=15.0):
            self._status('FAILED', 'navigate_to_pose action server not available')
            return

        self._status('RUNNING', 'Mission started')

        for index, wp in enumerate(waypoints):
            if self.stopping:
                self._status('CANCELLED', 'Stopped by operator')
                return
            while self.paused and not self.stopping:
                time.sleep(0.2)

            name = wp.get('name', 'WP-%d' % index)
            self._progress(index, name, int(index * 100 / total),
                           'Going to %s' % name)

            self.fail_reason = None
            ok = self._go_when_clear(wp, index, name, int(index * 100 / total))
            if self.stopping:
                self._status('CANCELLED', 'Stopped by operator')
                return
            if not ok:
                self._status('FAILED',
                             self.fail_reason or 'Could not reach %s' % name)
                return

            if wp.get('isCapture'):
                # One photograph facing the waypoint's yaw, or several turning
                # round in place: captureViews on the waypoint, else the
                # capture_views parameter, which is read here each time and so
                # can be changed with ros2 param set between missions. 4 is a
                # photograph every 90 degrees.
                #
                # Turned in steps rather than one sweep because each step is an
                # ordinary Nav2 goal at the same x, y - it finishes, settles and
                # photographs before the next - and because a turn that loses
                # localisation part-way loses one step, not the whole circle.
                try:
                    views = int(wp.get('captureViews')
                                or self.get_parameter('capture_views').value
                                or 1)
                except (TypeError, ValueError):
                    views = 1
                views = max(1, min(views, 12))
                base_yaw = float(wp.get('yaw', 0.0))

                for k in range(views):
                    if self.stopping:
                        self._status('CANCELLED', 'Stopped by operator')
                        return
                    while self.paused and not self.stopping:
                        time.sleep(0.2)
                    label = name if views == 1 else '%s_%d' % (name, k + 1)

                    if k > 0:
                        turned = dict(wp)
                        yaw = base_yaw + k * 2.0 * math.pi / views
                        turned['yaw'] = math.atan2(math.sin(yaw), math.cos(yaw))
                        self._progress(index, name,
                                       int((index + 1) * 100 / total),
                                       'Turning to view %d of %d at %s'
                                       % (k + 1, views, name))
                        if not self._go_to(turned):
                            if self.stopping:
                                self._status('CANCELLED', 'Stopped by operator')
                                return
                            # A view that cannot be reached is skipped rather
                            # than failing the mission: the other photographs
                            # and the rest of the route are still worth having.
                            self._progress(index, name,
                                           int((index + 1) * 100 / total),
                                           'Could not turn to view %d at %s - '
                                           'skipped' % (k + 1, name))
                            continue

                    self._progress(index, name, int((index + 1) * 100 / total),
                                   'Photographing %s' % label)
                    time.sleep(self.capture_settle)
                    self._capture(label)

        self._progress(total - 1, waypoints[-1].get('name', ''), 100,
                       'Finish: progress = 100')
        self._status('COMPLETED', 'Mission complete')

    def _on_scan(self, msg):
        self.last_scan = msg
        self.last_scan_at = time.monotonic()

    def _goal_blocked(self, wp):
        """Say whether an obstacle not on the map is sitting on this waypoint."""
        x, y = float(wp.get('x', 0.0)), float(wp.get('y', 0.0))
        scan = self.last_scan
        if scan is None or time.monotonic() - self.last_scan_at > 2.0:
            return False
        try:
            robot = self.tf_buffer.lookup_transform(
                self.frame, self.base_frame, rclpy.time.Time())
            sensor = self.tf_buffer.lookup_transform(
                self.frame, scan.header.frame_id, rclpy.time.Time())
        except Exception:                          # noqa: BLE001
            return False
        # Close to a goal the robot's own returns land near it too.
        if math.hypot(robot.transform.translation.x - x,
                      robot.transform.translation.y - y) \
                < self.obstacle_ignore_within:
            return False
        q = sensor.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        points = scan_points(scan, sensor.transform.translation.x,
                             sensor.transform.translation.y, yaw,
                             self.obstacle_range)
        return bool(new_obstacle_near(points, self.static_grid, x, y,
                                      self.obstacle_radius,
                                      self.map_wall_margin))

    def _go_when_clear(self, wp, index, name, percent):
        """Drive to a waypoint, waiting while an obstacle sits on it.

        Nav2 alone would try to get as close as it can, spin, back up and try
        again. Waiting ends the mission if the obstacle stays.
        """
        deadline = None
        while not self.stopping:
            if self._goal_blocked(wp):
                now = time.monotonic()
                if deadline is None:
                    deadline = now + self.obstacle_wait
                    note = 'Waiting: obstacle at %s' % name
                    self.get_logger().warn(note)
                    self._progress(index, name, percent, note)
                self._publish_wait(name, now, deadline)
                if now >= deadline:
                    self.fail_reason = (
                        'Obstacle at %s did not clear in %d s'
                        % (name, self.obstacle_wait))
                    self.get_logger().warn(self.fail_reason)
                    return False
                time.sleep(0.5)
                continue
            if deadline is not None:
                self._progress(index, name, percent, 'Going to %s' % name)
            self.blocked_during_goal = False
            self._turn_toward(wp, index, name, percent)
            if self.stopping:
                return False
            arrive_yaw = self._arrival_yaw(wp)
            started = time.monotonic()
            ok = self._go_to(wp, arrive_yaw)
            # Foxy's BT navigator can drop a fresh goal at once with "send_goal
            # failed" when a server is slow to answer. That is not the goal
            # being unreachable, so send it again.
            for _ in range(2):
                if ok or self.stopping or self.blocked_during_goal \
                        or time.monotonic() - started >= 4.0:
                    break
                self.get_logger().warn('Nav2 dropped %s at once, retrying' % name)
                self.fail_reason = None
                time.sleep(1.0)
                started = time.monotonic()
                ok = self._go_to(wp, arrive_yaw)
            if ok and arrive_yaw is not None and not self.stopping:
                self._progress(index, name, percent, 'Turning at %s' % name)
                self._spin_to_yaw(float(wp.get('yaw', 0.0)))
            if ok or self.stopping or not self.blocked_during_goal:
                return ok
            # Cancelled for an obstacle that appeared on the way: the wait
            # runs from when it was seen.
            deadline = None
        return False

    def _turn_toward(self, wp, index, name, percent):
        """Face a waypoint that is behind the robot before driving to it.

        DWB scores short forward arcs and this robot may not reverse, so a goal
        close behind it can leave no arc worth taking: on 2026-11-09 a robot
        0.59 m from a goal behind it stood still until Nav2 gave up. Foxy has
        no rotation shim to do this. Best effort: if the turn is refused or
        held up, Nav2 gets the goal as it always did.
        """
        if not self.turn_before_goal:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame, self.base_frame, rclpy.time.Time())
        except Exception:                          # noqa: BLE001
            return
        dx = float(wp.get('x', 0.0)) - tf.transform.translation.x
        dy = float(wp.get('y', 0.0)) - tf.transform.translation.y
        yaw = self._current_yaw()
        if yaw is None or math.hypot(dx, dy) < self.turn_skip_within:
            return
        angle = turn_needed(yaw, dx, dy)
        if abs(angle) < self.turn_min_angle:
            return
        note = 'Turning toward %s' % name
        self.get_logger().info('%s: %.0f deg' % (note, math.degrees(angle)))
        self._progress(index, name, percent, note)
        self._spin(angle)
        self._progress(index, name, percent, 'Going to %s' % name)

    def _spin(self, angle):
        """Turn on the spot through Nav2; gives up quietly if refused or held."""
        if not self.spin.wait_for_server(timeout_sec=2.0):
            return
        goal = Spin.Goal()
        goal.target_yaw = float(angle)
        send = self.spin.send_goal_async(goal)
        while not send.done() and rclpy.ok():
            time.sleep(0.05)
        handle = send.result()
        if handle is None or not handle.accepted:
            return
        result = handle.get_result_async()
        deadline = time.monotonic() + self.turn_timeout
        # Spin has no timeout of its own, and the collision guard can hold it.
        while not result.done() and rclpy.ok():
            if self.stopping or self.paused \
                    or time.monotonic() >= deadline \
                    or self.guard_hold.seconds(time.monotonic()) >= 2.0:
                handle.cancel_goal_async()
                break
            time.sleep(0.1)
        while self.paused and not self.stopping:
            time.sleep(0.2)

    def _publish_wait(self, name, now, deadline):
        """Tell the web how long an obstacle on a waypoint has been waited for."""
        if now - self._last_feedback < 1.0:
            return
        self._last_feedback = now
        m = self.mission or {}
        payload = dict(self.last_progress or {})
        payload.update({'runId': m.get('runId'),
                        'missionId': m.get('missionId'),
                        'live': {
                            'blockedAt': name,
                            'blockedSeconds': int(
                                self.obstacle_wait - (deadline - now)),
                            'obstacleWaitSeconds': int(self.obstacle_wait),
                        }})
        self.last_progress = payload
        self._publish(m.get('progressTopic'), payload)

    def _arrival_yaw(self, wp):
        """The heading to arrive with, when the waypoint's own is the wrong way.

        A goal whose heading points back the way the robot came fails the same
        way a goal behind it does, about 0.3 m out. Arrive facing forward and
        turn on the spot instead. None when the waypoint's heading is fine.
        """
        if not self.turn_before_goal:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame, self.base_frame, rclpy.time.Time())
        except Exception:                          # noqa: BLE001
            return None
        dx = float(wp.get('x', 0.0)) - tf.transform.translation.x
        dy = float(wp.get('y', 0.0)) - tf.transform.translation.y
        if math.hypot(dx, dy) < self.turn_skip_within:
            return None
        bearing = math.atan2(dy, dx)
        if abs(turn_needed(float(wp.get('yaw', 0.0)), dx, dy)) <= math.pi / 2:
            return None
        return bearing

    def _spin_to_yaw(self, target):
        yaw = self._current_yaw()
        if yaw is None:
            return
        angle = math.atan2(math.sin(target - yaw), math.cos(target - yaw))
        if abs(angle) >= 0.05:
            self._spin(angle)

    def _go_to(self, wp, arrive_yaw=None):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wp.get('x', 0.0))
        pose.pose.position.y = float(wp.get('y', 0.0))
        run_id = (self.mission or {}).get('runId')
        self._last_live = None
        goal_yaw = float(wp.get('yaw', 0.0)) if arrive_yaw is None else arrive_yaw
        z, w = yaw_to_quaternion(goal_yaw)
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        goal.pose = pose

        # Nav2's feedback is the only place the distance remaining and the
        # recovery count exist, and the web had neither - a mission that was
        # circling a waypoint it could not finish looked, on the web, exactly
        # like one walking normally. The heading is added here because Nav2
        # does not report it at all, and on 2026-09-14 every failure to
        # finish a goal turned out to be the heading rather than the
        # position.
        self._live_goal_yaw = goal_yaw
        send = self.nav.send_goal_async(
            goal, feedback_callback=self._on_nav_feedback)
        while not send.done() and rclpy.ok():
            time.sleep(0.05)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False
        self.goal_handle = handle

        result_future = handle.get_result_async()
        next_check = time.monotonic() + 0.5
        while not result_future.done() and rclpy.ok():
            time.sleep(0.1)
            if not self.blocked_during_goal and time.monotonic() >= next_check:
                next_check = time.monotonic() + 0.5
                if self._goal_blocked(wp):
                    self.blocked_during_goal = True
                    handle.cancel_goal_async()
                elif self.guard_hold.seconds(time.monotonic()) \
                        >= self.obstacle_wait:
                    self.fail_reason = (
                        'Stopped by an obstacle for %d s'
                        % self.obstacle_wait)
                    self.get_logger().warn(self.fail_reason)
                    handle.cancel_goal_async()
                    # Do not cancel it again every half second.
                    next_check = math.inf
        self.goal_handle = None
        result = result_future.result()
        ok = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
        replaced = self.mission is None or self.mission.get('runId') != run_id
        if not ok and not self.fail_reason and not self.stopping and not replaced \
                and not self.blocked_during_goal and self._last_live:
            self.fail_reason = 'Nav2 gave up on %s with %.2f m left' % (
                wp.get('name', 'the waypoint'),
                self._last_live['distanceRemaining'])
        return ok

    def _on_guard_status(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        self.guard_hold.update(status.get('stopped'),
                               status.get('stopCommands', 0), time.monotonic())

    def _on_nav_feedback(self, msg):
        """Forward Nav2's per-goal numbers to the web, once a second."""
        now = time.monotonic()
        if now - self._last_feedback < 1.0:
            return
        self._last_feedback = now
        fb = msg.feedback

        live = {
            'distanceRemaining': round(float(fb.distance_remaining), 3),
            'navigationTime': int(fb.navigation_time.sec),
            'recoveries': int(fb.number_of_recoveries),
            # The guard can hold the robot for obstacle_wait seconds before the
            # mission fails; without this the web only hears about it at the end.
            'guardHeldSeconds': int(self.guard_hold.seconds(now)),
            'obstacleWaitSeconds': int(self.obstacle_wait),
        }

        yaw = self._current_yaw()
        if yaw is not None and self._live_goal_yaw is not None:
            err = math.atan2(math.sin(self._live_goal_yaw - yaw),
                             math.cos(self._live_goal_yaw - yaw))
            live.update({
                'yawDeg': round(math.degrees(yaw), 2),
                'yawTargetDeg': round(math.degrees(self._live_goal_yaw), 2),
                'yawErrorDeg': round(math.degrees(err), 2),
            })

        m = self.mission or {}
        payload = dict(self.last_progress or {})
        payload.update({'runId': m.get('runId'),
                        'missionId': m.get('missionId'),
                        'live': live})
        self.last_progress = payload
        self._last_live = live
        self._publish(m.get('progressTopic'), payload)

    def _current_yaw(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame, self.base_frame, rclpy.time.Time())
        except Exception:                          # noqa: BLE001
            return None
        q = tf.transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # ----------------------------------------------------------- capture

    def _on_capture_done(self, msg):
        try:
            self.capture_reply = json.loads(msg.data)
        except ValueError:
            self.capture_reply = {'ok': False, 'error': 'unreadable reply'}

    def _capture(self, name):
        """Ask mission_capture for a photograph and forward it to the web.

        A failed photograph does not end the mission. Walking the rest of the
        route and coming back with the other pictures beats standing in a
        corridor over one of them - but the web is told, in the progress
        message, rather than being left to assume it arrived.
        """
        self.capture_reply = None
        request = String()
        request.data = json.dumps(
            {'label': name, 'mission': str(self.mission.get('runId') or '')})
        self.capture_pub.publish(request)

        deadline = time.monotonic() + self.capture_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.capture_reply is not None:
                break
            time.sleep(0.1)

        reply = self.capture_reply
        if reply is None or not reply.get('ok'):
            reason = 'no reply' if reply is None else reply.get('error', '')
            self.get_logger().error('capture at %s failed: %s' % (name, reason))
            return

        path = reply.get('path')
        try:
            with open(path, 'rb') as handle:
                encoded = base64.b64encode(handle.read()).decode('ascii')
        except OSError as exc:
            self.get_logger().error('cannot read %s: %s' % (path, exc))
            return

        m = self.mission
        self._publish(m.get('imageTopic'),
                      {'runId': m.get('runId'), 'missionId': m.get('missionId'),
                       'waypointName': name,
                       'takenAt': time.strftime('%Y-%m-%dT%H:%M:%S'),
                       'pose': reply.get('pose'),
                       'image': encoded})
        self.get_logger().info(
            'sent %s to the web (%d KB)' % (name, len(encoded) // 1024))

    # --------------------------------------------------------------- map

    def _on_map(self, msg):
        """Turn the occupancy grid into a PNG the browser can draw.

        The metadata travels with it and is not optional: resolution and
        origin are what turn the x/y this node publishes into a pixel on that
        image. A map without them is a picture, and the robot marker lands
        somewhere arbitrary on it.

        Row order is flipped. An OccupancyGrid's first row is the bottom of
        the map in world terms, and every image format and every canvas draws
        the first row at the top.
        """
        try:
            import numpy as np
            import cv2
        except ImportError as exc:
            self.get_logger().error('cannot encode the map: %s' % exc)
            return

        grid = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        image = np.full(grid.shape, 127, dtype=np.uint8)   # unknown
        image[grid == 0] = 255                             # free
        image[grid >= 50] = 0                              # occupied
        image = np.flipud(image)

        ok, buf = cv2.imencode('.png', image)
        if not ok:
            self.get_logger().error('PNG encoding failed')
            return

        origin = msg.info.origin
        import math
        q = origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.map_payload = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin': {'x': origin.position.x, 'y': origin.position.y,
                       'yaw': yaw},
            'frame': msg.header.frame_id or self.frame,
            'timestamp': int(time.time()),
            'image': base64.b64encode(buf.tobytes()).decode('ascii'),
        }
        self.get_logger().info(
            'map ready for the web: %dx%d at %.3f m/cell, %d KB PNG'
            % (msg.info.width, msg.info.height, msg.info.resolution,
               len(self.map_payload['image']) // 1024))
        self._publish_map()

    def _publish_map(self):
        if self.map_payload is not None:
            self._publish(self.map_topic, self.map_payload)

    # -------------------------------------------------------------- pose

    def _on_particles(self, msg):
        n = len(msg.poses)
        if n < 2:
            self.spread = None
            return
        xs = [p.position.x for p in msg.poses]
        ys = [p.position.y for p in msg.poses]
        mx, my = sum(xs) / n, sum(ys) / n
        self.spread = math.sqrt(
            sum((x - mx) ** 2 + (y - my) ** 2 for x, y in zip(xs, ys)) / n)

    def _on_localization_check(self, msg):
        try:
            self.localization_check = json.loads(msg.data)
        except ValueError:
            self.localization_check = None
            return
        # Sent for as long as AMCL stays out, so resuming from the web while it
        # is still wrong pauses again rather than driving on a stale position.
        if (self.localization_check.get('pause') and not self.paused
                and self.worker is not None and self.worker.is_alive()):
            self.paused = True
            self._cancel_current()
            self.get_logger().error(
                'paused: AMCL is %.2f m from FAST-LIO - set the pose again'
                % self.localization_check.get('distance', 0.0))

    def _on_initial_pose(self, msg):
        # init_pose seeds AMCL at the origin at startup. That is a default, not
        # somebody saying where the robot is, and no real pose is exactly zero.
        p = msg.pose.pose.position
        if p.x != 0.0 or p.y != 0.0:
            self.pose_set = True

    def _set_initial_pose(self, data):
        """Tell AMCL where the robot is, from a click on the web map.

        Refused while a mission runs. Moving the localiser's idea of the
        robot under a goal that is being driven is how a robot sets off
        towards a wall at full speed.

        The covariance is deliberately loose - 0.5 m and about 15 degrees -
        because a click on a 5 cm map is not more accurate than that, and a
        tight covariance around a slightly wrong click leaves AMCL no room to
        correct it from the scan. The spread reported afterwards is what
        says whether it converged.
        """
        from geometry_msgs.msg import PoseWithCovarianceStamped
        note = None
        # self.mission outlives the run so a page opened later can still read
        # it, so only a live worker means a mission is running.
        running = self.worker is not None and self.worker.is_alive()
        if running and not self.stopping:
            note = 'a mission is running - initial pose refused'
            self.get_logger().warn(note)
        else:
            try:
                x = float(data['x'])
                y = float(data['y'])
                yaw = float(data.get('yaw', 0.0))
            except (KeyError, TypeError, ValueError):
                note = 'set_initial_pose needs numeric x, y and yaw (radians)'
        if note is None:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self.frame
            # Zero stamp: "latest transform". A current-time stamp runs a few
            # milliseconds ahead of odometry and AMCL refuses it, which was
            # measured on site on 2026-09-11.
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            z, w = yaw_to_quaternion(yaw)
            msg.pose.pose.orientation.z = z
            msg.pose.pose.orientation.w = w
            cov = [0.0] * 36
            cov[0] = 0.25          # x variance, (0.5 m)^2
            cov[7] = 0.25          # y variance
            cov[35] = 0.068        # yaw variance, (15 deg)^2
            msg.pose.covariance = cov
            self.initialpose_pub.publish(msg)
            self.spread = None     # the old cloud no longer means anything
            self.pose_set = True
            self.get_logger().info(
                'initial pose from the web: x=%.3f y=%.3f yaw=%.3f'
                % (x, y, yaw))
        self._publish(self.state_topic, {
            'initialPose': {'accepted': note is None, 'note': note,
                            'x': data.get('x'), 'y': data.get('y'),
                            'yaw': data.get('yaw')},
            'timestamp': int(time.time())})

    def _publish_scan(self):
        """The scan in map coordinates, so the web can lay it over the map.

        Only while a page is showing it (each page repeats want_scan), and
        thinned to a few hundred points to stay light on Wi-Fi.
        """
        scan = self.last_scan
        now = time.monotonic()
        if scan is None or now >= self.scan_until \
                or now - self.last_scan_at > 2.0:
            return
        try:
            sensor = self.tf_buffer.lookup_transform(
                self.frame, scan.header.frame_id, rclpy.time.Time())
        except Exception:                          # noqa: BLE001
            return
        q = sensor.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        points = scan_points(scan, sensor.transform.translation.x,
                             sensor.transform.translation.y, yaw,
                             self.obstacle_range * 2)
        step = max(1, math.ceil(len(points) / SCAN_MAX_POINTS))
        self._publish(self.scan_topic,
                      {'frame': self.frame,
                       'points': [[round(x, 2), round(y, 2)]
                                  for x, y in points[::step]],
                       'timestamp': int(time.time())})

    def _publish_pose(self):
        """Where the robot is, once a second, whether or not a mission runs.

        The web needs to draw the robot on the map between missions as well as
        during them, and a pose that only appears while navigating makes the
        robot look absent whenever it is idle.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame, self.base_frame, rclpy.time.Time())
        except Exception:                          # noqa: BLE001
            return
        import math
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # Wall time, not the ROS clock: the web draws a marker and needs to
        # know how old it is, and a stale pose that looks current is worse
        # than a gap. The board's clock was 56 years out until today, which is
        # exactly the failure this field makes visible.
        # localizationSpread: metres of particle scatter, or None when AMCL
        # has not updated since the last pose was set - it only updates after
        # the robot moves about 0.2 m. Under 0.35 is localised.
        self._publish(self.pose_topic,
                      {'x': t.x, 'y': t.y, 'yaw': yaw, 'frame': self.frame,
                       'localizationSpread': (None if self.spread is None
                                              else round(self.spread, 3)),
                       'poseSet': self.pose_set,
                       'spreadLimit': SPREAD_LIMIT_M,
                       'localizationCheck': self.localization_check,
                       'timestamp': int(time.time())})


def main(argv=None):
    rclpy.init(args=argv)
    node = MqttMissionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
