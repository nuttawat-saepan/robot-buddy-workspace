"""Live 2D occupancy grid from FAST-LIO2's registered point cloud.

Subscribes to the world-frame cloud FAST-LIO publishes each scan, drops every
point outside a horizontal band, and counts what is left into a grid published
as nav_msgs/OccupancyGrid on /map. RViz then shows a 2D map filling in while
the run is going, with no SLAM pass and no map_saver step.

Why not pointcloud_to_laserscan plus slam_toolbox: a Mid-360 samples different
directions every frame, so a single-frame slice leaves half the beams empty and
the holes move around, which is a poor input for scan matching. FAST-LIO has
already solved the pose using the IMU, so the pose is not in question here and
all this has to do is accumulate.

This node is read-only with respect to the robot. It subscribes to a cloud and
publishes a map; it never publishes /cmd_vel and starts no motion.

Heights are in FAST-LIO's world frame, whose origin is wherever the sensor
started, not the floor. With the sensor on a standing Go2 the floor sits a few
tens of centimetres below zero, which is what the defaults assume.

That world frame also inherits the sensor's orientation at startup, so a
tilted mount tilts the whole map. The Mid-360 sits 13 degrees nose-down on the
Go2, which is enough that a flat slice cuts diagonally through the floor and
paints it as wall right across the room. pitch_deg and roll_deg level the cloud
before slicing; set pitch_deg to the mount tilt.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from tf2_ros import StaticTransformBroadcaster


def cloud_xyz(msg):
    """Pull x/y/z out of a PointCloud2 without pulling in a PCL dependency."""
    offsets = {f.name: f.offset for f in msg.fields}
    for axis in ('x', 'y', 'z'):
        if axis not in offsets:
            return None
    count = msg.width * msg.height
    if count == 0:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    raw = raw[:count * msg.point_step].reshape(count, msg.point_step)
    out = np.empty((count, 3), dtype=np.float32)
    for i, axis in enumerate(('x', 'y', 'z')):
        start = offsets[axis]
        out[:, i] = raw[:, start:start + 4].copy().view(np.float32).ravel()
    return out


class LivoxGridMap(Node):

    def __init__(self):
        super().__init__('livox_grid_map')

        self.declare_parameter('cloud_topic', '/cloud_registered')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'camera_init')
        self.declare_parameter('grid_frame', 'map_level')
        # When the LIO launch runs with enable_tf_bridge, it already undoes the
        # mount tilt on odom->camera_init, and odom is then exactly this node's
        # levelled frame. Publishing our own transform as well would give
        # camera_init two parents. In that case set grid_frame to odom and turn
        # this off.
        self.declare_parameter('publish_level_tf', True)
        # Mount tilt of the Mid-360 on the Go2: 13 degrees nose-down. Positive
        # means nose-down. Set to 0 for a handheld run held level.
        self.declare_parameter('pitch_deg', 13.0)
        self.declare_parameter('roll_deg', 0.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('z_min', -0.30)
        self.declare_parameter('z_max', 0.50)
        # One stray return should not become a wall. Three is enough to reject
        # noise while still marking a thin object seen over a couple of frames.
        self.declare_parameter('hit_threshold', 3)
        self.declare_parameter('publish_period', 1.0)
        # The grid is preallocated so cells never have to be reindexed. 60 m
        # square at 5 cm is 1.44M cells, a few MB, and larger than any run this
        # is meant for.
        self.declare_parameter('extent', 60.0)

        get = self.get_parameter
        self.resolution = float(get('resolution').value)
        self.z_min = float(get('z_min').value)
        self.z_max = float(get('z_max').value)
        self.hit_threshold = int(get('hit_threshold').value)
        self.map_frame = str(get('map_frame').value)
        self.grid_frame = str(get('grid_frame').value)
        extent = float(get('extent').value)

        pitch = math.radians(float(get('pitch_deg').value))
        roll = math.radians(float(get('roll_deg').value))
        # Columns of R_level_sensor are the sensor axes written in a level
        # frame, so p_level = R * p_sensor.
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        r_pitch = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        r_roll = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        self.level = (r_pitch @ r_roll).astype(np.float32)
        self.levelling = bool(pitch or roll)

        self.side = int(extent / self.resolution)
        self.origin = -0.5 * extent
        self.hits = np.zeros((self.side, self.side), dtype=np.int32)
        self.scans = 0

        # Transient local so RViz still gets the map if it connects late, which
        # it usually does when the operator is already walking.
        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(
            OccupancyGrid, str(get('map_topic').value), map_qos)
        self.create_subscription(
            PointCloud2, str(get('cloud_topic').value), self.cloud_cb, 20)
        self.create_timer(float(get('publish_period').value), self.publish_map)

        # The grid lives in the levelled frame, so it needs its own frame id and
        # a transform back to the one FAST-LIO publishes, or RViz would draw it
        # flat while the cloud stays tilted.
        if bool(get('publish_level_tf').value):
            self.static_tf = StaticTransformBroadcaster(self)
            self.static_tf.sendTransform(self.level_transform(pitch, roll))

        self.get_logger().info(
            f'slice z {self.z_min:.2f}..{self.z_max:.2f} m, '
            f'{self.resolution} m cells, {self.side}x{self.side} grid, '
            f'levelling pitch {math.degrees(pitch):.1f} deg '
            f'roll {math.degrees(roll):.1f} deg')

    def level_transform(self, pitch, roll):
        """map_frame -> grid_frame, the inverse of the levelling rotation."""
        cy, sy = math.cos(-roll / 2.0), math.sin(-roll / 2.0)
        cq, sq = math.cos(-pitch / 2.0), math.sin(-pitch / 2.0)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.grid_frame
        # Inverse of R = Ry(pitch) * Rx(roll) is Rx(-roll) * Ry(-pitch).
        tf.transform.rotation.x = sy * cq
        tf.transform.rotation.y = cy * sq
        tf.transform.rotation.z = -sy * sq
        tf.transform.rotation.w = cy * cq
        return tf

    def cloud_cb(self, msg):
        points = cloud_xyz(msg)
        if points is None:
            return
        if self.levelling:
            points = points @ self.level.T
        band = points[(points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)]
        if band.size == 0:
            return

        idx = ((band[:, :2] - self.origin) / self.resolution).astype(np.int64)
        inside = np.all((idx >= 0) & (idx < self.side), axis=1)
        idx = idx[inside]
        if idx.size == 0:
            return

        flat = idx[:, 1] * self.side + idx[:, 0]
        counts = np.bincount(flat, minlength=self.side * self.side)
        self.hits += counts.reshape(self.side, self.side).astype(np.int32)
        self.scans += 1

    def publish_map(self):
        touched = self.hits > 0
        if not touched.any():
            self.get_logger().warn(
                'no points in the height band yet - check z_min/z_max against '
                'where the sensor started', throttle_duration_sec=10.0)
            return

        # Publish only the observed region. The preallocated grid is mostly
        # empty and sending all of it would cost several MB per message.
        rows = np.flatnonzero(touched.any(axis=1))
        cols = np.flatnonzero(touched.any(axis=0))
        r0, r1 = int(rows[0]), int(rows[-1]) + 1
        c0, c1 = int(cols[0]), int(cols[-1]) + 1
        window = self.hits[r0:r1, c0:c1]

        data = np.full(window.shape, -1, dtype=np.int8)   # unknown
        data[window > 0] = 0                              # seen through
        data[window >= self.hit_threshold] = 100          # occupied

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.grid_frame
        grid.info.resolution = self.resolution
        grid.info.width = int(window.shape[1])
        grid.info.height = int(window.shape[0])
        grid.info.origin.position.x = self.origin + c0 * self.resolution
        grid.info.origin.position.y = self.origin + r0 * self.resolution
        grid.info.origin.orientation.w = 1.0
        grid.data = data.ravel().tolist()
        self.pub.publish(grid)

        occupied = int((data == 100).sum())
        self.get_logger().info(
            f'scans {self.scans}  map {grid.info.width}x{grid.info.height}  '
            f'occupied {occupied}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxGridMap()
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
