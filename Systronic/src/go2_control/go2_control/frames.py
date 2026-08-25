"""Mount-pose arithmetic shared by the launch files that tie FAST-LIO in.

FAST-LIO's world frame `camera_init` inherits the sensor's orientation at
startup, so the whole world frame is tilted by the mount angle. Two transforms
undo that, and they have to agree with each other and with the levelling
`pcd_to_map` applied when the map was built:

    odom -> camera_init     the levelling rotation itself
    body -> base_link       its inverse, plus the drop to the floor

Both livox_amcl.launch.py and livox_slam.launch.py publish that pair, which is
why the arithmetic lives here rather than being copied into each. Getting the
two copies out of step would put the live scan and the map in different frames,
and the symptom - a localiser that almost works - is a poor way to find out.
"""
import math


def level_rotation(pitch_rad, roll_rad):
    """Ry(pitch) @ Rx(roll), matching pcd_to_map.level() exactly."""
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    r_pitch = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    r_roll = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    return tuple(
        tuple(sum(r_pitch[i][k] * r_roll[k][j] for k in range(3))
              for j in range(3))
        for i in range(3)
    )


def transpose(m):
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def to_ypr(m):
    """ZYX Euler angles, the order static_transform_publisher expects.

    The inverse of Ry(p) @ Rx(r) is Rx(-r) @ Ry(-p), which is not of the form
    Rz @ Ry @ Rx, so negating the two angles is wrong. At the tilt measured on
    02_loop it is wrong by 0.31 degrees - small, but there is no reason to
    accept it when the exact decomposition is four lines.
    """
    yaw = math.atan2(m[1][0], m[0][0])
    pitch = math.atan2(-m[2][0], math.hypot(m[2][1], m[2][2]))
    roll = math.atan2(m[2][1], m[2][2])
    return yaw, pitch, roll


def lio_bridges(pitch_deg, roll_deg, sensor_height):
    """The two static transforms, ready for static_transform_publisher.

    Returns (odom_to_camera_init, body_to_base_link), each as
    (x, y, z, yaw, pitch, roll) - the argument order the publisher takes.

    camera_init sits where the sensor started, which is what odom means, so the
    first transform is a pure rotation. The second undoes the tilt and then
    drops to the floor; the drop is a vector in the levelled frame, so it has
    to be rotated into `body` before it can be used as a translation there.
    """
    rot = level_rotation(math.radians(pitch_deg), math.radians(roll_deg))
    inv = transpose(rot)

    odom_ypr = to_ypr(rot)
    body_ypr = to_ypr(inv)
    body_xyz = tuple(inv[i][2] * (-sensor_height) for i in range(3))

    return ((0.0, 0.0, 0.0) + odom_ypr, body_xyz + body_ypr)
