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
                                           name}]}
    in   /missions/control   {action: "pause" | "resume" | "stop"
                                      | "get_state" | "get_params"
                                      | "set_params", params: {...},
                                        restart: bool}
    out  <statusTopic>       {runId, missionId, status, message}
    out  <progressTopic>     {runId, missionId, progress, currentWaypointIndex,
                              currentWaypointName, message}
    out  <imageTopic>        {runId, missionId, waypointName, takenAt, pose,
                              image}   - image is base64 JPEG
    out  /missions/map       {width, height, resolution, origin:{x,y,yaw},
                              image}   - base64 PNG, occupied pixels dark
    out  /missions/pose      {x, y, yaw, frame, timestamp}
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
import json
import os
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

import tf2_ros


# ------------------------------------------------------------- parameters

# The eight values the web may set, each mapped to the node that owns it, its
# full parameter name, and whether the running node actually picks the change
# up.
#
# 'live' means the node subscribes to parameter events and re-reads: on Foxy
# that is dwb_plugins::KinematicsHandler, which has an
# on_parameter_event_callback and is what holds the speed and acceleration
# limits. Those four take effect on the next control period.
#
# 'restart' means the node read the value once, at configure or initialise
# time, into its own members and will never look again. Foxy's
# SimpleGoalChecker and InflationLayer both do this - neither library carries
# a parameter callback of any kind. A set call still succeeds and the new
# value is visible to `ros2 param get`, and the robot goes on using the old
# one. That gap is reported rather than hidden; it is the same failure as the
# bridge clamp, and it is why the reply carries an `applies` field.
#
# AMCL's update_min_d, min_particles and the rest are absent for the same
# reason, one step worse: they need the lifecycle taken down and back up,
# which also throws away the pose estimate.
TUNABLE = {
    'max_vel_x': ('/controller_server', 'FollowPath.max_vel_x', 'live'),
    'max_vel_theta': ('/controller_server', 'FollowPath.max_vel_theta',
                      'live'),
    'acc_lim_x': ('/controller_server', 'FollowPath.acc_lim_x', 'live'),
    'acc_lim_theta': ('/controller_server', 'FollowPath.acc_lim_theta',
                      'live'),
    'xy_goal_tolerance': ('/controller_server',
                          'goal_checker.xy_goal_tolerance', 'restart'),
    'yaw_goal_tolerance': ('/controller_server',
                           'goal_checker.yaw_goal_tolerance', 'restart'),
    'inflation_radius': ('/local_costmap/local_costmap',
                         'inflation_layer.inflation_radius', 'restart'),
    'cost_scaling_factor': ('/local_costmap/local_costmap',
                            'inflation_layer.cost_scaling_factor', 'restart'),
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
LIMITS = {
    'max_vel_x': (0.05, 0.60),
    'max_vel_theta': (0.20, 0.80),
    'acc_lim_x': (0.20, 1.00),
    'acc_lim_theta': (0.20, 1.50),
    'xy_goal_tolerance': (0.35, 1.00),
    'yaw_goal_tolerance': (0.10, 1.00),
    'inflation_radius': (0.10, 1.00),
    'cost_scaling_factor': (1.0, 10.0),
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

        self.capture_pub = self.create_publisher(
            String, '/mission/capture_request', 10)
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
        self.goal_handle = None
        self.worker = None

        # Made here rather than on demand: the MQTT thread is the only
        # caller and a client created from it, after rclpy.spin has the
        # executor, is not picked up until something else wakes it.
        from lifecycle_msgs.srv import ChangeState
        from rcl_interfaces.srv import GetParameters, SetParameters
        self._get_cli = {}
        self._set_cli = {}
        for node_name in sorted({t[0] for t in TUNABLE.values()}):
            self._get_cli[node_name] = self.create_client(
                GetParameters, node_name + '/get_parameters')
            self._set_cli[node_name] = self.create_client(
                SetParameters, node_name + '/set_parameters')
        # Every restart parameter is owned by controller_server, directly or
        # through the local costmap it constructs, so one node takes them
        # all. The costmap is not cycled on its own: its lifecycle is driven
        # by the controller, and transitioning it from outside would leave
        # the two disagreeing about what state it is in.
        self._cycle_cli = self.create_client(
            ChangeState, '/controller_server/change_state')

        self.mqtt = None
        self._connect_mqtt()

        self.create_timer(self.pose_period, self._publish_pose)

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
        try:
            self.mqtt.connect(self.broker, self.port, 60)
            self.mqtt.loop_start()
        except Exception as exc:                   # noqa: BLE001
            self.get_logger().error(
                'MQTT connect failed: %s - the node stays up and retries'
                % exc)

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
            self._publish(m.get('statusTopic'), self.last_status)
        if self.last_progress:
            self._publish(m.get('progressTopic'), self.last_progress)
        # Sent even when nothing is running, so the page can tell "no mission"
        # apart from "the robot did not answer". paused is not a status of its
        # own in the protocol, and a page cannot infer it from RUNNING alone.
        self._publish(self.state_topic, {
            'hasMission': bool(self.mission),
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
            names = [t[1] for t in TUNABLE.values() if t[0] == node_name]
            keys = [k for k, t in TUNABLE.items() if t[0] == node_name]
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
                values[key] = value.double_value
        return values

    def _cycle_controller(self):
        """Take controller_server round its lifecycle so it re-reads.

        The goal checker's tolerances and the inflation layer's radius are
        read once, at configure and at onInitialize, into members that
        nothing updates afterwards. This is the cheap way to make them take
        effect: verified against /amcl on a replayed stack on 2026-09-11,
        where the four transitions took 18 ms and lifecycle_manager did not
        react at all - no bond break, no restart of the managed set.

        **The same check has not been run against controller_server.** AMCL
        is a leaf; the controller owns a costmap and an action server, and a
        client mid-goal when this happens is a case nothing here has seen.
        That is why a running mission is refused rather than paused.
        """
        from lifecycle_msgs.msg import Transition
        from lifecycle_msgs.srv import ChangeState

        if self.mission is not None and not self.stopping:
            return ['a mission is running - refusing to cycle '
                    'controller_server. Stop the mission first.']
        if not self._cycle_cli.service_is_ready():
            return ['/controller_server is not up - nothing was cycled']

        notes = []
        for label, transition in (
                ('deactivate', Transition.TRANSITION_DEACTIVATE),
                ('cleanup', Transition.TRANSITION_CLEANUP),
                ('configure', Transition.TRANSITION_CONFIGURE),
                ('activate', Transition.TRANSITION_ACTIVATE)):
            req = ChangeState.Request()
            req.transition = Transition(id=transition)
            res = self._wait(self._cycle_cli.call_async(req), timeout=15.0)
            if res is None or not res.success:
                # Half a cycle is worse than none: the controller is left
                # inactive and will not drive until someone finishes the
                # sequence by hand. Say so rather than reporting a number.
                notes.append(
                    'controller_server %s FAILED - it is not active and the '
                    'robot cannot be driven until it is brought back' % label)
                self.get_logger().error(notes[-1])
                return notes
        notes.append('controller_server cycled - the restart values are now '
                     'in use')
        self.get_logger().warn(notes[-1])
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
            node_name, full, _applies = TUNABLE[key]
            param = Parameter()
            param.name = full
            param.value = ParameterValue()
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = value
            by_node.setdefault(node_name, []).append(param)

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

        stale = sorted({k for k in requested
                        if k in TUNABLE and TUNABLE[k][2] == 'restart'})
        if stale and not restart:
            notes.append(
                'stored but NOT in use until controller_server is cycled: %s'
                % ', '.join(stale))
        elif stale and restart:
            notes.extend(self._cycle_controller())
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
            'applies': {k: t[2] for k, t in TUNABLE.items()},
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

            ok = self._go_to(wp)
            if self.stopping:
                self._status('CANCELLED', 'Stopped by operator')
                return
            if not ok:
                self._status('FAILED', 'Could not reach %s' % name)
                return

            if wp.get('isCapture'):
                self._progress(index, name, int((index + 1) * 100 / total),
                               'Photographing %s' % name)
                time.sleep(self.capture_settle)
                self._capture(name)

        self._progress(total - 1, waypoints[-1].get('name', ''), 100,
                       'Finish: progress = 100')
        self._status('COMPLETED', 'Mission complete')

    def _go_to(self, wp):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wp.get('x', 0.0))
        pose.pose.position.y = float(wp.get('y', 0.0))
        z, w = yaw_to_quaternion(float(wp.get('yaw', 0.0)))
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        goal.pose = pose

        send = self.nav.send_goal_async(goal)
        while not send.done() and rclpy.ok():
            time.sleep(0.05)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False
        self.goal_handle = handle

        result_future = handle.get_result_async()
        while not result_future.done() and rclpy.ok():
            time.sleep(0.1)
        self.goal_handle = None
        result = result_future.result()
        return result is not None and result.status == GoalStatus.STATUS_SUCCEEDED

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
        self._publish(self.pose_topic,
                      {'x': t.x, 'y': t.y, 'yaw': yaw, 'frame': self.frame,
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
