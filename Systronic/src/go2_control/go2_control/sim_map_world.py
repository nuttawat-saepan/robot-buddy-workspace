"""Turn a saved map into a Gazebo world, so a real site can be simulated.

    ros2 run go2_control sim_map_world <map.yaml> <worlds_dir> <models_dir> <name>

The walls go into <models_dir>/<name>_walls.stl, found through GAZEBO_MODEL_PATH.

Every occupied cell becomes part of a 2 m wall. Only what the map holds is
there: no furniture the scan never saw, nothing low, no glass, and any error in
the map is a real wall in the world, so the simulated scan always agrees with
the map better than the real one does.
"""
import re
import sys

import numpy as np

WALL_HEIGHT = 2.0
OCCUPIED_BELOW = 89     # occupied_thresh 0.65 of a 255 grey image


def read_pgm(path):
    """A binary PGM as a uint8 array, top row first."""
    with open(path, 'rb') as source:
        raw = source.read()
    fields, at = [], 0
    while len(fields) < 4:
        while raw[at:at + 1].isspace():
            at += 1
        if raw[at:at + 1] == b'#':
            at = raw.index(b'\n', at)
            continue
        end = at
        while not raw[end:end + 1].isspace():
            end += 1
        fields.append(raw[at:end])
        at = end
    if fields[0] != b'P5':
        raise ValueError(f'{path} is not a binary PGM')
    width, height = int(fields[1]), int(fields[2])
    return np.frombuffer(raw, np.uint8, width * height, at + 1).reshape(
        height, width)


def read_map(yaml_path):
    """(image file named by the yaml, resolution, origin x, origin y)."""
    with open(yaml_path) as source:
        text = source.read()
    origin = re.search(r'origin:\s*\[([^\]]+)\]', text).group(1).split(',')
    return (re.search(r'image:\s*(\S+)', text).group(1),
            float(re.search(r'resolution:\s*([\d.]+)', text).group(1)),
            float(origin[0]), float(origin[1]))


def cell_blocks(mask):
    """Cover the True cells with rectangles (col, row, cols, rows), exactly.

    Each row's runs are extended downwards while the next row has the same run,
    which keeps a map's long walls as one model each.
    """
    open_runs, blocks = {}, []
    for row in range(mask.shape[0] + 1):
        runs = set()
        if row < mask.shape[0]:
            edges = np.flatnonzero(np.diff(np.r_[0, mask[row].astype(int), 0]))
            runs = {(int(a), int(b)) for a, b in zip(edges[::2], edges[1::2])}
        for run in set(open_runs) - runs:
            first = open_runs.pop(run)
            blocks.append((run[0], first, run[1] - run[0], row - first))
        for run in runs - set(open_runs):
            open_runs[run] = row
    return blocks


def rectangles(image, resolution, origin_x, origin_y):
    """Occupied cells as sim_scenarios rectangles, in map metres."""
    height = image.shape[0]
    result = []
    for col, row, cols, rows in cell_blocks(image < OCCUPIED_BELOW):
        y_max = origin_y + (height - row) * resolution
        result.append((origin_x + col * resolution,
                       y_max - rows * resolution,
                       origin_x + (col + cols) * resolution,
                       y_max, WALL_HEIGHT))
    return result


def box_triangles(wall):
    """The 12 triangles of one wall block, outward facing."""
    x0, y0, x1, y1, height = wall
    v = [(x0, y0, 0), (x1, y0, 0), (x1, y1, 0), (x0, y1, 0),
         (x0, y0, height), (x1, y0, height), (x1, y1, height), (x0, y1, height)]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
             (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return [(v[a], v[b], v[c]) for a, b, c in faces]


def stl_bytes(walls):
    """Every wall block in one binary STL."""
    import struct
    triangles = [t for wall in walls for t in box_triangles(wall)]
    out = [b'\0' * 80, struct.pack('<I', len(triangles))]
    for a, b, c in triangles:
        out.append(struct.pack('<12fH', 0, 0, 0, *a, *b, *c, 0))
    return b''.join(out)


def world_sdf(name, mesh_uri):
    """The walls as one static mesh.

    As boxes, the meeting room's 386 blocks ran Gazebo at 0.06 of real time,
    one model or many, because every box is tested against every ray. A mesh
    is searched through a tree instead.
    """
    geometry = f'<geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry>'
    return (
        f'<?xml version="1.0"?>\n<sdf version="1.6"><world name="{name}">'
        '<include><uri>model://ground_plane</uri></include>'
        '<include><uri>model://sun</uri></include>'
        '<plugin name="ros_init" filename="libgazebo_ros_init.so"/>'
        '<plugin name="ros_factory" filename="libgazebo_ros_factory.so"/>'
        '<model name="walls"><static>true</static><link name="link">'
        f'<collision name="c">{geometry}</collision>'
        f'<visual name="v">{geometry}</visual>'
        '</link></model></world></sdf>\n')


def main():
    yaml_path, worlds_dir, models_dir, name = sys.argv[1:5]
    image_name, resolution, origin_x, origin_y = read_map(yaml_path)
    image = read_pgm(f'{yaml_path.rsplit("/", 1)[0]}/{image_name}')
    walls = rectangles(image, resolution, origin_x, origin_y)
    with open(f'{models_dir}/{name}_walls.stl', 'wb') as out:
        out.write(stl_bytes(walls))
    with open(f'{worlds_dir}/{name}.world', 'w') as out:
        out.write(world_sdf(name, f'model://{name}_walls.stl'))
    print(f'{name}: {len(walls)} wall blocks from {image.shape} cells')


if __name__ == '__main__':
    main()
