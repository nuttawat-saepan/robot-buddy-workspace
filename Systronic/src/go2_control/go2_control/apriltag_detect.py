"""Detect AprilTags in the camera stream and publish them for april_localizer.

Read-only with respect to the robot: it subscribes to images and publishes
detections. It cannot move anything.

## Why this exists rather than apriltag_ros

`april_localizer.py` consumes `apriltag_msgs/AprilTagDetectionArray`, and Foxy
packages neither `apriltag_ros` nor `apriltag_msgs`. Only `ros-foxy-apriltag`,
the C detector library with no ROS node around it, is available. Building
`apriltag_ros` from source would work, but it also has to be built for the
Unitree board's aarch64, and it pulls in the C library and its wrapper.

OpenCV is already a dependency of this package and already knows how to find
AprilTags: `cv2.aruco` has carried the `DICT_APRILTAG_*` dictionaries since
3.4, and OpenCV 4.2 is what Focal ships. So the whole detector is a dictionary
lookup and a call to `detectMarkers`, which leaves `apriltag_msgs` - three
message definitions - as the only thing that has to be built from source.

## What it has to get right

`AprilTagDetection` carries no pose. `april_localizer` recovers the pose itself
with `cv2.solvePnP` from the four corners, the `CameraInfo` intrinsics and the
configured `tag_size`, so this node only has to fill `id` and `corners`.

Corner order is the part that is easy to get wrong and impossible to notice.
`april_localizer` builds its object points as

    (-h, +h)  (+h, +h)  (+h, -h)  (-h, -h)      # top-left, clockwise, y up

which is exactly the order `cv2.aruco.detectMarkers` returns. Publishing the
same four points in a different order produces a pose that looks reasonable and
places the robot somewhere it is not - the failure mode that is worse than no
AprilTag at all.

## Tag family

The family must match the tags physically on the wall. 36h11 is the AprilTag
default and what this node defaults to; a 36h11 detector will not see a 25h9
tag at all, and the symptom is simply that nothing is ever detected.

    ros2 run go2_control apriltag_detect
    ros2 run go2_control apriltag_detect --ros-args -p family:=36h10
"""
import sys

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray, Point
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


FAMILIES = {
    '16h5': cv2.aruco.DICT_APRILTAG_16H5,
    '25h9': cv2.aruco.DICT_APRILTAG_25H9,
    '36h10': cv2.aruco.DICT_APRILTAG_36H10,
    '36h11': cv2.aruco.DICT_APRILTAG_36H11,
}


class AprilTagDetect(Node):

    def __init__(self):
        super().__init__('apriltag_detect')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('compressed', False)
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('family', '36h11')
        # Detecting every frame is wasted work: april_localizer only needs
        # min_detections (3 by default) agreeing samples, and the camera frame
        # rate is far higher than the robot's pose changes. On a 4-core board
        # shared with FAST-LIO and the leg controller this is the difference
        # between a detector that fits and one that does not.
        self.declare_parameter('detect_every_n', 3)

        family = self.get_parameter('family').value
        if family not in FAMILIES:
            raise RuntimeError(
                f'unknown family {family!r}; known: {sorted(FAMILIES)}')
        self.dictionary = cv2.aruco.Dictionary_get(FAMILIES[family])
        self.params = cv2.aruco.DetectorParameters_create()
        # Sub-pixel corner refinement. The pose comes from solvePnP on these
        # four points, so corner accuracy is pose accuracy - the default
        # CORNER_REFINE_NONE leaves the corners at integer pixels, which at a
        # few metres is worth centimetres of position error.
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG

        self.family = family
        self.every_n = max(1, int(self.get_parameter('detect_every_n').value))
        self.bridge = CvBridge()
        self.frames = 0
        self.detections = 0
        self.last_ids = []

        self.pub = self.create_publisher(
            AprilTagDetectionArray,
            self.get_parameter('detections_topic').value, 10)

        topic = self.get_parameter('image_topic').value
        if self.get_parameter('compressed').value:
            self.create_subscription(
                CompressedImage, topic, self.on_compressed,
                qos_profile_sensor_data)
        else:
            self.create_subscription(
                Image, topic, self.on_image, qos_profile_sensor_data)

        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f'detecting AprilTag {family} on {topic}, every {self.every_n} '
            f'frame(s)')

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as exc:
            self.get_logger().warn(f'cannot convert image: {exc}')
            return
        self.process(frame, msg.header)

    def on_compressed(self, msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        if frame is None:
            self.get_logger().warn('cannot decode compressed image')
            return
        self.process(frame, msg.header)

    def process(self, gray, header):
        self.frames += 1
        if self.frames % self.every_n:
            return

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.params)

        out = AprilTagDetectionArray()
        # The header is carried through unchanged, and its frame_id is what
        # april_localizer looks the camera transform up by. If the camera node
        # stamps a frame nothing publishes a transform for, every detection is
        # discarded with a TF warning - which is the state this workspace was
        # in before base_link -> camera_link was added.
        out.header = header

        if ids is not None:
            for tag_id, quad in zip(ids.flatten(), corners):
                det = AprilTagDetection()
                det.family = self.family
                det.id = int(tag_id)
                # quad is (1, 4, 2), in the order detectMarkers guarantees:
                # top-left, top-right, bottom-right, bottom-left. Passed
                # straight through because that is the order
                # april_localizer's object points assume.
                pts = quad.reshape(4, 2)
                det.corners = [Point(x=float(x), y=float(y)) for x, y in pts]
                centre = pts.mean(axis=0)
                det.centre = Point(x=float(centre[0]), y=float(centre[1]))
                out.detections.append(det)
            self.detections += len(out.detections)
            self.last_ids = [int(i) for i in ids.flatten()]

        # Published even when empty, so a consumer can tell "looking and seeing
        # nothing" apart from "detector not running".
        self.pub.publish(out)

    def report(self):
        if self.frames == 0:
            self.get_logger().warn(
                'no images received - is the camera node running, and does '
                'image_topic match?')
        else:
            seen = f', last saw ids {self.last_ids}' if self.last_ids else ''
            self.get_logger().info(
                f'{self.frames} frames, {self.detections} detections{seen}')


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
