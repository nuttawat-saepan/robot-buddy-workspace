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
    in   /missions/control   {action: "pause" | "resume" | "stop"}
    out  <statusTopic>       {runId, missionId, status, message}
    out  <progressTopic>     {runId, missionId, progress, currentWaypointIndex,
                              currentWaypointName, message}
    out  <imageTopic>        {runId, missionId, waypointName, takenAt, pose,
                              image}   - image is base64 JPEG

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
        self.goal_handle = None
        self.worker = None

        self.mqtt = None
        self._connect_mqtt()

        self.create_timer(self.pose_period, self._publish_pose)

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
            self._control(data.get('action', ''))

    def _control(self, action):
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

    def _cancel_current(self):
        handle = self.goal_handle
        if handle is not None:
            handle.cancel_goal_async()

    # ----------------------------------------------------------- mission

    def _start_mission(self, data):
        if self.worker is not None and self.worker.is_alive():
            self.get_logger().warn(
                'a mission is already running - ignoring the new one')
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
        self._publish(m.get('statusTopic'),
                      {'runId': m.get('runId'), 'missionId': m.get('missionId'),
                       'status': status, 'message': message})

    def _progress(self, index, name, percent, message=''):
        m = self.mission or {}
        self._publish(m.get('progressTopic'),
                      {'runId': m.get('runId'), 'missionId': m.get('missionId'),
                       'progress': percent, 'currentWaypointIndex': index,
                       'currentWaypointName': name, 'message': message})

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
        self._publish(self.pose_topic,
                      {'x': t.x, 'y': t.y, 'yaw': yaw, 'frame': self.frame})


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
