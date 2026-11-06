"""The obstacle test room and its scenarios, kept apart from ROS.

The room runs x 0 to 16, y -3 to 3, with a wall across it at x 11 that has a
0.9 m doorway on the centre line. The robot starts at (2, 0) facing +x.

    python3 -m go2_control.sim_scenarios worlds map     writes the world and map

A rectangle is (x_min, y_min, x_max, y_max, height).
"""
from dataclasses import dataclass, field
import os
import sys

import numpy as np

START = (2.0, 0.0)
THICK = 0.10
WALL_HEIGHT = 2.0
DOOR_X = 11.0
DOOR_WIDTH = 0.90
BODY_X = 0.35
BODY_Y = 0.25

WALLS = (
    (0.0, 3.0, 16.0, 3.0 + THICK, WALL_HEIGHT),
    (0.0, -3.0 - THICK, 16.0, -3.0, WALL_HEIGHT),
    (-THICK, -3.0, 0.0, 3.0, WALL_HEIGHT),
    (16.0, -3.0, 16.0 + THICK, 3.0, WALL_HEIGHT),
    (DOOR_X, DOOR_WIDTH / 2, DOOR_X + THICK, 3.0, WALL_HEIGHT),
    (DOOR_X, -3.0, DOOR_X + THICK, -DOOR_WIDTH / 2, WALL_HEIGHT),
)


def box_at(x, y, size=0.5, height=0.8):
    half = size / 2
    return (x - half, y - half, x + half, y + half, height)


@dataclass(frozen=True)
class Scenario:
    name: str
    goal: tuple
    # What is there from the start.
    boxes: tuple = ()
    # Appears when the robot's x passes trigger_x: (trigger_x, rectangle).
    popup: tuple = None
    start: tuple = START
    arrives: bool = True
    # Allowed to touch: only for showing a limit that is already known.
    known_contact: bool = False
    note: str = field(default='')


SCENARIOS = {s.name: s for s in (
    Scenario('clear', goal=(8.0, 0.0),
             note='nothing in the way: must not stop at all'),
    Scenario('static_ahead', goal=(8.0, 0.0), boxes=(box_at(5.0, 0.0),),
             note='on the map path from the start: plan around it'),
    Scenario('popup_far', goal=(8.0, 0.0), popup=(3.5, box_at(5.5, 0.0)),
             note='appears 2 m ahead: Nav2 has time to replan'),
    Scenario('popup_close', goal=(8.0, 0.0), popup=(3.5, box_at(4.5, 0.0)),
             note='appears about 0.4 m past the stop line'),
    Scenario('door', goal=(13.0, 0.0),
             note='0.9 m doorway: the guard must not stop it'),
    Scenario('blocked_goal', goal=(8.0, 0.0), boxes=(box_at(8.0, 0.0),),
             arrives=False, note='box on the goal: fail without touching it'),
    Scenario('sealed', goal=(8.0, 0.0),
             boxes=((5.0, -3.0, 5.2, 3.0, 1.0),), arrives=False,
             note='no way through: fail without touching it'),
    Scenario('turn_near_box', goal=(0.8, 0.0), start=(3.0, 0.0),
             boxes=(box_at(3.30, 0.72, size=0.4),),
             note='must turn round with a box beside the nose'),
    Scenario('low_box', goal=(8.0, 0.0),
             boxes=(box_at(5.0, 0.0, height=0.15),), known_contact=True,
             note='15 cm high: below the band of /scan and the guard'),
)}


def _perimeter(half_x, half_y, step=0.02):
    xs = np.arange(-half_x, half_x + step, step)
    ys = np.arange(-half_y, half_y + step, step)
    return np.vstack([
        np.column_stack([xs, np.full_like(xs, half_y)]),
        np.column_stack([xs, np.full_like(xs, -half_y)]),
        np.column_stack([np.full_like(ys, half_x), ys]),
        np.column_stack([np.full_like(ys, -half_x), ys]),
    ])


_BODY_OUTLINE = _perimeter(BODY_X, BODY_Y)


def body_gap(x, y, yaw, rectangles):
    """Shortest distance from the body at this pose to any rectangle, metres."""
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    outline = _BODY_OUTLINE @ np.array([[cos_y, sin_y], [-sin_y, cos_y]])
    outline = outline + np.array([x, y])
    best = np.inf
    for x_min, y_min, x_max, y_max, _ in rectangles:
        dx = np.maximum(np.maximum(x_min - outline[:, 0],
                                   outline[:, 0] - x_max), 0.0)
        dy = np.maximum(np.maximum(y_min - outline[:, 1],
                                   outline[:, 1] - y_max), 0.0)
        best = min(best, float(np.hypot(dx, dy).min()))
    return best


def box_sdf(name, rectangle):
    x_min, y_min, x_max, y_max, height = rectangle
    size = f'{x_max - x_min} {y_max - y_min} {height}'
    return (
        f'<?xml version="1.0"?><sdf version="1.6"><model name="{name}">'
        '<static>true</static>'
        f'<pose>{(x_min + x_max) / 2} {(y_min + y_max) / 2} {height / 2} '
        '0 0 0</pose><link name="link">'
        f'<collision name="c"><geometry><box><size>{size}</size></box>'
        '</geometry></collision>'
        f'<visual name="v"><geometry><box><size>{size}</size></box>'
        '</geometry></visual></link></model></sdf>')


def world_sdf():
    walls = []
    for index, wall in enumerate(WALLS):
        inner = box_sdf(f'wall_{index}', wall)
        walls.append(inner.split('<sdf version="1.6">')[1].split('</sdf>')[0])
    return (
        '<?xml version="1.0"?>\n<sdf version="1.6"><world name="obstacle_room">'
        '<include><uri>model://ground_plane</uri></include>'
        '<include><uri>model://sun</uri></include>'
        '<plugin name="ros_init" filename="libgazebo_ros_init.so"/>'
        '<plugin name="ros_factory" filename="libgazebo_ros_factory.so"/>'
        + ''.join(walls) + '</world></sdf>\n')


def map_image(resolution=0.05, margin=0.5):
    """Trinary occupancy image: 254 free, 0 occupied."""
    width = int(round((16.0 + 2 * margin) / resolution))
    height = int(round((6.0 + 2 * margin) / resolution))
    origin = (-margin, -3.0 - margin)
    image = np.full((height, width), 254, dtype=np.uint8)
    for x_min, y_min, x_max, y_max, _ in WALLS:
        col_a = int((x_min - origin[0]) / resolution)
        col_b = int(np.ceil((x_max - origin[0]) / resolution))
        row_a = int((y_min - origin[1]) / resolution)
        row_b = int(np.ceil((y_max - origin[1]) / resolution))
        image[height - row_b:height - row_a, col_a:col_b] = 0
    return image, origin, resolution


def write_files(world_dir, map_dir):
    with open(os.path.join(world_dir, 'obstacle_room.world'), 'w') as out:
        out.write(world_sdf())
    image, origin, resolution = map_image()
    with open(os.path.join(map_dir, 'obstacle_room.pgm'), 'wb') as out:
        out.write(b'P5\n%d %d\n255\n' % (image.shape[1], image.shape[0]))
        out.write(image.tobytes())
    with open(os.path.join(map_dir, 'obstacle_room.yaml'), 'w') as out:
        out.write(
            'image: obstacle_room.pgm\nmode: trinary\n'
            f'resolution: {resolution}\n'
            f'origin: [{origin[0]}, {origin[1]}, 0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


if __name__ == '__main__':
    write_files(sys.argv[1], sys.argv[2])
