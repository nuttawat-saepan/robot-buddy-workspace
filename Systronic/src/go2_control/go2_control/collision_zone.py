"""Which velocity commands would take the body into something nearby.

The arithmetic of collision_guard, kept apart from ROS so that it can be run
over a bag and tested without a graph.

All distances are in base_link, from the centre of the robot:

    body          +-0.35 long, +-0.25 wide
    stop line     body + 0.20: +-0.55 long, +-0.45 wide

Which part of that ring matters depends on the motion. Driving straight only
looks ahead or behind, across the body's width plus a little. Checking the
sides as well would make a box 0.90 m wide, and the robot would stop at every
0.9 m doorway it could in fact pass through. A turn swings the corners, so it
looks at the whole ring, but only at points the rotation would bring within
turn_clearance of the body.
"""
from dataclasses import dataclass

import numpy as np

from go2_control.frames import level_rotation

FRONT = 'front'
REAR = 'rear'
TURN_LEFT = 'turn_left'
TURN_RIGHT = 'turn_right'
ZONES = (FRONT, REAR, TURN_LEFT, TURN_RIGHT)

# How far the rotation test looks ahead: about a second at DWB's 0.35 rad/s,
# which also covers stopping from that speed. Faster turns, such as a recovery
# spin, are limited by the bridge rather than allowed for here.
_TURN_ANGLES_RAD = (0.09, 0.18, 0.27, 0.35)


@dataclass(frozen=True)
class Geometry:
    body_x: float = 0.35
    body_y: float = 0.25
    stop_x: float = 0.55
    stop_y: float = 0.45
    strip_y: float = 0.30
    # A turn is refused only if it would bring the body this close to a point.
    turn_clearance: float = 0.05
    # Something on the robot returns points just past the front of the body
    # rectangle, at x 0.36-0.39 and |y| 0.05-0.15, measured standing still on
    # 2026-11-04. Without this the robot would always see itself in the way.
    self_x: float = 0.40
    self_y: float = 0.15
    min_z: float = 0.25
    max_z: float = 1.05
    # Below this a command component counts as not moving that way.
    linear_deadband: float = 0.01
    # DWB trims its heading continuously while driving straight. Treating that
    # as a turn would make a doorway jamb stop the robot, so only a real turn
    # looks at the ring.
    angular_deadband: float = 0.10


def to_base(xyz, pitch_deg, roll_deg, sensor_x, sensor_height):
    """Sensor-frame points to base_link, using the fixed mount.

    The robot's own pitch and roll are ignored, which is acceptable because
    only points close to the robot are used: 5 degrees at 0.55 m is 5 cm.
    """
    rotation = np.array(level_rotation(np.radians(pitch_deg),
                                       np.radians(roll_deg)))
    base = xyz @ rotation.T
    return base + np.array([sensor_x, 0.0, sensor_height])


def obstacle_points(base, geometry=Geometry()):
    """x, y of points in the height band that are not the robot itself."""
    x, y, z = base[:, 0], base[:, 1], base[:, 2]
    in_band = (z > geometry.min_z) & (z < geometry.max_z)
    in_body = (np.abs(x) <= geometry.body_x) & (np.abs(y) <= geometry.body_y)
    on_self = ((x >= 0.0) & (x <= geometry.self_x)
               & (np.abs(y) <= geometry.self_y))
    keep = in_band & ~in_body & ~on_self
    return np.stack([x[keep], y[keep]], axis=1)


def _gap(x, y, geometry):
    dx = np.maximum(np.abs(x) - geometry.body_x, 0.0)
    dy = np.maximum(np.abs(y) - geometry.body_y, 0.0)
    return np.hypot(dx, dy)


def zone_counts(xy, geometry=Geometry()):
    """How many points are in each zone. A dict keyed by ZONES."""
    x, y = xy[:, 0], xy[:, 1]
    in_strip = np.abs(y) <= geometry.strip_y
    in_ring = ((np.abs(x) <= geometry.stop_x)
               & (np.abs(y) <= geometry.stop_y))

    counts = {
        FRONT: int(np.count_nonzero(
            in_strip & (x > 0.0) & (x <= geometry.stop_x))),
        REAR: int(np.count_nonzero(
            in_strip & (x < 0.0) & (x >= -geometry.stop_x))),
    }

    # Turning the body left moves a fixed point right, in the body's frame, by
    # the same angle. A point only counts if the turn would actually bring the
    # corner within turn_clearance of it. Refusing every turn that merely
    # reduced the gap left a robot beside a box unable to turn either way, and
    # Nav2's spin recovery has no timeout, so it waited on the guard for 102 s
    # in the simulation.
    gap = _gap(x, y, geometry)
    for zone, sign in ((TURN_LEFT, -1.0), (TURN_RIGHT, 1.0)):
        endangered = np.zeros_like(in_ring)
        for angle in _TURN_ANGLES_RAD:
            cos_a, sin_a = np.cos(angle), np.sin(angle)
            moved_x = cos_a * x - sign * sin_a * y
            moved_y = sign * sin_a * x + cos_a * y
            moved_gap = _gap(moved_x, moved_y, geometry)
            endangered |= ((moved_gap < gap - 1e-6)
                           & (moved_gap < geometry.turn_clearance))
        counts[zone] = int(np.count_nonzero(in_ring & endangered))
    return counts


def zones_for_command(vx, wz, geometry=Geometry()):
    """The zones that have to be clear for this command to be safe."""
    needed = []
    if vx > geometry.linear_deadband:
        needed.append(FRONT)
    elif vx < -geometry.linear_deadband:
        needed.append(REAR)
    if wz > geometry.angular_deadband:
        needed.append(TURN_LEFT)
    elif wz < -geometry.angular_deadband:
        needed.append(TURN_RIGHT)
    return needed


class GuardState:
    """Decides from the zones seen and the clock. Holds no ROS objects.

    A zone blocks from the first frame with enough points in it until it has
    been clear for `release_seconds`, so a surface flickering across the line
    does not make the robot jerk. Zones are held separately: a robot stopped
    by something ahead may still back away from it.

    With no recent frame everything is blocked. Not seeing is not clear.
    """

    def __init__(self, geometry=Geometry(), min_points=5,
                 release_seconds=0.5, frame_timeout=0.30):
        self.geometry = geometry
        self.min_points = min_points
        self.release_seconds = release_seconds
        self.frame_timeout = frame_timeout
        self.last_frame = None
        self.last_hit = {zone: None for zone in ZONES}
        self.counts = {zone: 0 for zone in ZONES}

    def update(self, xy, now):
        self.last_frame = now
        self.counts = zone_counts(xy, self.geometry)
        for zone, count in self.counts.items():
            if count >= self.min_points:
                self.last_hit[zone] = now

    def stale(self, now):
        return (self.last_frame is None
                or now - self.last_frame > self.frame_timeout)

    def blocked_zones(self, now):
        return [zone for zone, hit in self.last_hit.items()
                if hit is not None and now - hit < self.release_seconds]

    def refusal(self, vx, wz, now):
        """Why this command may not pass, or None if it may."""
        needed = zones_for_command(vx, wz, self.geometry)
        if not needed:
            return None
        if self.stale(now):
            return 'no recent LiDAR frame'
        blocked = self.blocked_zones(now)
        hit = [zone for zone in needed if zone in blocked]
        return ', '.join(hit) if hit else None
