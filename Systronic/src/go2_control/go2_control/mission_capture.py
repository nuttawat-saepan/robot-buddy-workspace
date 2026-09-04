#!/usr/bin/env python3
"""Photograph a capture point and record where the robot was standing.

The deliverable is a robot that walks a mission, photographs designated
points, and sends the pictures back. `send_mission` already walks the list and
holds at capture points; until now it held for three seconds and took nothing,
because nothing existed to take it.

This node is that missing half, and it is deliberately separate from
`camera.py`. That one streams frames continuously for a human watching RViz.
A mission photograph is a different job: one frame, on request, at a known
pose, written to disk with enough context to be useful three weeks later on
someone else's screen. A stream cannot answer "where was the robot standing
when this was taken", and that is the question the web interface has to answer.

## The frame source is chosen, not assumed

    unitree   the robot's own camera through the SDK        (site)
    topic     any sensor_msgs/Image publisher              (Gazebo, a webcam)
    test      a generated frame with the pose drawn on it  (this desk)
    auto      unitree, then topic, then test               (default)

`test` is not a stub to be replaced later. Everything downstream of the frame -
the pose lookup, the JSON sidecar, the file naming, the done message, the
mission's wait - is identical whichever source produced the pixels, so `test`
exercises the whole path on a machine with no robot and no camera. The one
thing it cannot tell you is whether the robot's camera works.

## Talking to it

    /mission/capture_request   std_msgs/String   a name, or a JSON object
    /mission/capture_done      std_msgs/String   JSON: ok, path, pose, error

Plain topics rather than a service, because go2_interfaces carries no srv and
adding one means rebuilding the interfaces package on the board before
anything here can run. A request and a reply on two topics needs neither.

The reply is always published, including on failure. A mission that waits for
a reply that never comes looks exactly like a mission that has hung, and the
robot is standing still in a corridor while someone works out which it is.

## Running it

    ros2 run go2_control mission_capture --ros-args -p source:=test
    ros2 topic pub --once /mission/capture_request std_msgs/msg/String \\
        "{data: 'north_door'}"

Read-only with respect to the robot: it takes pictures and writes files. It
publishes no velocity and cannot move anything.
"""

import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

import tf2_ros


class MissionCapture(Node):

    def __init__(self):
        super().__init__('mission_capture')

        self.source = self.declare_parameter('source', 'auto').value
        self.image_topic = self.declare_parameter(
            'image_topic', '/camera/stream').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter(
            'base_frame',
            os.environ.get('GO2_BASE_FRAME', 'base_link')).value
        self.out_dir = os.path.expanduser(self.declare_parameter(
            'out_dir', '~/go2_captures').value)
        # A photograph taken from a pose nobody trusts is worse than no
        # photograph, because it will be believed. The capture still happens -
        # throwing away the only picture of a place is not this node's call -
        # but the sidecar says so and the reply carries the warning up.
        self.pose_timeout = self.declare_parameter('pose_timeout', 1.0).value
        self.jpeg_quality = self.declare_parameter('jpeg_quality', 90).value

        os.makedirs(self.out_dir, exist_ok=True)

        self._cv2 = None
        self._bridge = None
        self._latest_frame = None
        self._latest_frame_time = 0.0
        self._unitree_client = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.done_pub = self.create_publisher(String, '/mission/capture_done', 10)
        self.create_subscription(
            String, '/mission/capture_request', self.on_request, 10)

        self.source = self._resolve_source()

        self.get_logger().info(
            'mission_capture ready  source=%s  out_dir=%s  frames=%s -> %s'
            % (self.source, self.out_dir, self.map_frame, self.base_frame))

    # ------------------------------------------------------------ sources

    def _resolve_source(self):
        """Settle on a frame source now, so the first capture is not the
        moment anyone discovers there isn't one."""
        try:
            import cv2
            self._cv2 = cv2
        except ImportError:
            self.get_logger().error(
                'cv2 is missing - no source can encode a frame')
            return 'none'

        wanted = self.source
        if wanted in ('auto', 'unitree') and self._try_unitree():
            return 'unitree'
        if wanted == 'unitree':
            self.get_logger().error(
                'source:=unitree was asked for and the SDK camera did not '
                'open. Refusing to quietly fall back - a mission that '
                'silently photographs a test pattern is worse than one that '
                'fails here.')
            return 'none'

        if wanted in ('auto', 'topic'):
            self._subscribe_image()
            if wanted == 'topic':
                return 'topic'
            # In auto, a topic that nobody publishes is indistinguishable from
            # one that has not started yet, so keep the subscription and let
            # the first capture decide.
            return 'topic-or-test'

        return 'test'

    def _try_unitree(self):
        try:
            from unitree_sdk2py.go2.video.video_client import VideoClient
        except ImportError:
            return False
        try:
            client = VideoClient()
            client.SetTimeout(3.0)
            client.Init()
            code, _ = client.GetImageSample()
            if code != 0:
                self.get_logger().warn(
                    'Unitree camera answered code %s' % code)
                return False
            self._unitree_client = client
            return True
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn('Unitree camera unavailable: %s' % exc)
            return False

    def _subscribe_image(self):
        try:
            from cv_bridge import CvBridge
            from sensor_msgs.msg import Image
        except ImportError:
            self.get_logger().warn(
                'cv_bridge is missing - the topic source is unavailable')
            return
        self._bridge = CvBridge()
        # Camera topics are best effort almost without exception, and a
        # reliable subscriber on a best-effort publisher receives nothing at
        # all while reporting no error - the failure this project has hit on
        # /scan, /particlecloud and /livox/lidar in turn.
        self.create_subscription(
            Image, self.image_topic, self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg):
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._latest_frame_time = time.time()
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn('could not convert image: %s' % exc)

    def grab_frame(self, label, pose):
        """Return (frame, source_used) or (None, reason)."""
        if self.source == 'none':
            return None, 'no frame source is available'

        if self._unitree_client is not None:
            code, data = self._unitree_client.GetImageSample()
            if code != 0:
                return None, 'Unitree camera returned code %s' % code
            import numpy as np
            frame = self._cv2.imdecode(
                np.frombuffer(bytes(data), dtype=np.uint8),
                self._cv2.IMREAD_COLOR)
            if frame is None:
                return None, 'Unitree camera returned an undecodable frame'
            return frame, 'unitree'

        if self._latest_frame is not None:
            age = time.time() - self._latest_frame_time
            if age < 2.0:
                return self._latest_frame.copy(), 'topic'
            if self.source == 'topic':
                return None, '%s is stale by %.1f s' % (self.image_topic, age)

        if self.source == 'topic':
            return None, 'nothing has been received on %s' % self.image_topic

        return self._test_frame(label, pose), 'test'

    def _test_frame(self, label, pose):
        """A frame that says what it is, so it can never be mistaken for a
        real photograph of anywhere."""
        import numpy as np
        cv2 = self._cv2
        frame = np.full((480, 640, 3), 40, dtype=np.uint8)
        lines = [
            'TEST FRAME - NOT A PHOTOGRAPH',
            'label  %s' % label,
            'time   %s' % time.strftime('%Y-%m-%d %H:%M:%S'),
        ]
        if pose is not None:
            lines.append('pose   x=%.2f y=%.2f yaw=%.2f'
                         % (pose['x'], pose['y'], pose['yaw']))
        else:
            lines.append('pose   unknown')
        for i, text in enumerate(lines):
            cv2.putText(frame, text, (20, 60 + i * 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1,
                        cv2.LINE_AA)
        return frame

    # --------------------------------------------------------------- pose

    def current_pose(self):
        """map -> base_link as a plain dict, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.pose_timeout))
        except Exception:                             # noqa: BLE001
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        # yaw alone, because that is what a mission file carries and what
        # anyone reading the sidecar later actually wants.
        import math
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return {'x': t.x, 'y': t.y, 'z': t.z, 'yaw': yaw,
                'frame': self.map_frame}

    # ------------------------------------------------------------ request

    def on_request(self, msg):
        label, mission = self._parse_request(msg.data)
        pose = self.current_pose()
        frame, used = self.grab_frame(label, pose)

        if frame is None:
            self._reply(ok=False, label=label, error=used)
            self.get_logger().error('capture "%s" failed: %s' % (label, used))
            return

        directory = os.path.join(self.out_dir, mission) if mission else self.out_dir
        os.makedirs(directory, exist_ok=True)
        stem = '%s_%s' % (self._safe(label), time.strftime('%Y%m%d_%H%M%S'))
        image_path = os.path.join(directory, stem + '.jpg')
        side_path = os.path.join(directory, stem + '.json')

        try:
            self._cv2.imwrite(
                image_path, frame,
                [int(self._cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)])
        except Exception as exc:                      # noqa: BLE001
            self._reply(ok=False, label=label, error='could not write: %s' % exc)
            return

        sidecar = {
            'label': label,
            'mission': mission,
            'image': os.path.basename(image_path),
            'taken': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'source': used,
            'pose': pose,
        }
        if pose is None:
            sidecar['warning'] = (
                'no %s -> %s transform: this photograph has no position'
                % (self.map_frame, self.base_frame))
        with open(side_path, 'w') as handle:
            json.dump(sidecar, handle, indent=2)

        self.get_logger().info(
            'captured "%s" from %s -> %s' % (label, used, image_path))
        self._reply(ok=True, label=label, path=image_path, pose=pose,
                    source=used,
                    warning=sidecar.get('warning'))

    def _parse_request(self, data):
        """A bare name is the common case; JSON carries a mission folder."""
        data = (data or '').strip()
        if data.startswith('{'):
            try:
                payload = json.loads(data)
                return (str(payload.get('label') or 'capture'),
                        str(payload.get('mission') or ''))
            except ValueError:
                pass
        return (data or 'capture'), ''

    @staticmethod
    def _safe(name):
        keep = [c if (c.isalnum() or c in '-_') else '_' for c in name]
        return ''.join(keep)[:60] or 'capture'

    def _reply(self, **fields):
        fields = {k: v for k, v in fields.items() if v is not None}
        msg = String()
        msg.data = json.dumps(fields)
        self.done_pub.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = MissionCapture()
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
