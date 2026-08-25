"""Turn a FAST-LIO2 point cloud into a Nav2 2D occupancy map.

FAST-LIO writes the accumulated run to a single .pcd. This slices a horizontal
band out of it and rasterises that band into the .pgm/.yaml pair Nav2's
map_server expects, so no SLAM pass is needed on top of the LIO result.

Why slice the accumulated cloud rather than the live scan: a Mid-360 samples
different directions every frame, so pointcloud_to_laserscan on a single frame
leaves half the beams empty and the holes move around. After LIO has registered
a few thousand frames into one cloud those holes are filled in.

This is offline post-processing. It reads a file and writes files; it starts no
ROS node and publishes nothing.

    ros2 run go2_control pcd_to_map \
        ~/ws_fastlio_livox/src/FAST_LIO/PCD/scans.pcd \
        -o src/go2_control/map/livox_handheld

Height band defaults are given in the LiDAR's own frame, whose origin is the
sensor, not the floor. On a handheld run the sensor is around chest height, so
the band that corresponds to a robot-height obstacle sits well below zero.
Check --z-min/--z-max against the reported cloud extent before trusting a map.
"""
import argparse
import math
import os
import struct
import sys

import numpy as np


class PCDError(Exception):
    pass


def read_pcd(path):
    """Read x/y/z from a binary or ASCII PCD. Returns an (N, 3) float array.

    Deliberately not using python-pcl or open3d: neither is a dependency of
    this workspace, and the subset of the format FAST-LIO writes is small.
    """
    with open(path, 'rb') as handle:
        raw = handle.read()

    end = raw.find(b'DATA ')
    if end < 0:
        raise PCDError(f'{path}: no DATA line, not a PCD file')
    line_end = raw.find(b'\n', end)
    header_text = raw[:line_end].decode('ascii', 'replace')
    body = raw[line_end + 1:]

    header = {}
    for line in header_text.splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        key, _, rest = line.partition(' ')
        header[key.upper()] = rest.split()

    data_kind = header.get('DATA', ['ascii'])[0].lower()
    fields = header.get('FIELDS', [])
    sizes = [int(v) for v in header.get('SIZE', [])]
    types = [v.upper() for v in header.get('TYPE', [])]
    counts = [int(v) for v in header.get('COUNT', ['1'] * len(fields))]
    n_points = int(header.get('POINTS', [0])[0])

    for axis in ('x', 'y', 'z'):
        if axis not in fields:
            raise PCDError(f'{path}: missing field {axis!r}, got {fields}')

    if data_kind == 'ascii':
        idx = [fields.index(a) for a in ('x', 'y', 'z')]
        rows = []
        for line in body.decode('ascii', 'replace').splitlines():
            parts = line.split()
            if len(parts) <= max(idx):
                continue
            rows.append([float(parts[i]) for i in idx])
        return np.asarray(rows, dtype=np.float64)

    if data_kind != 'binary':
        raise PCDError(
            f'{path}: DATA {data_kind} is not supported (binary or ascii only). '
            'binary_compressed comes from other tools, not from FAST-LIO.')

    np_kind = {'F': 'f', 'U': 'u', 'I': 'i'}
    names, formats, offsets = [], [], []
    offset = 0
    for name, size, kind, count in zip(fields, sizes, types, counts):
        width = size * count
        if kind in np_kind and count == 1:
            names.append(name)
            formats.append(f'<{np_kind[kind]}{size}')
            offsets.append(offset)
        offset += width

    dtype = np.dtype({
        'names': names, 'formats': formats,
        'offsets': offsets, 'itemsize': offset,
    })
    usable = len(body) // offset
    if n_points and usable < n_points:
        print(f'warning: header claims {n_points} points, file holds {usable}',
              file=sys.stderr)
    cloud = np.frombuffer(body[:usable * offset], dtype=dtype)
    return np.stack([cloud['x'], cloud['y'], cloud['z']], axis=1).astype(np.float64)


def level(points, pitch_deg, roll_deg):
    """Rotate a cloud out of the sensor's tilted world frame into a level one.

    FAST-LIO's world frame inherits the sensor's orientation at startup, so a
    mounted Mid-360 tilts the entire cloud by its mount angle. Slicing a tilted
    cloud at constant z cuts diagonally through the floor and paints it as wall
    across the room, which is what this undoes.
    """
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    if not pitch and not roll:
        return points
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    r_pitch = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    r_roll = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return points @ (r_pitch @ r_roll).T


def rasterise(points, resolution, z_min, z_max, hit_threshold, padding):
    """Slice a height band and count points per cell.

    Returns (grid, origin_x, origin_y) where grid is uint8:
        0 = free, 100 = occupied, 255 = unknown.
    """
    band = points[(points[:, 2] >= z_min) & (points[:, 2] <= z_max)]
    if band.size == 0:
        raise PCDError(
            f'no points between z={z_min} and z={z_max}. '
            'The band is in the sensor frame, so it is probably in the wrong '
            'place; widen it or shift it using the cloud extent printed above.')

    min_xy = band[:, :2].min(axis=0) - padding
    max_xy = band[:, :2].max(axis=0) + padding
    span = np.ceil((max_xy - min_xy) / resolution).astype(int) + 1
    width, height = int(span[0]), int(span[1])

    col = ((band[:, 0] - min_xy[0]) / resolution).astype(np.int64)
    row = ((band[:, 1] - min_xy[1]) / resolution).astype(np.int64)
    np.clip(col, 0, width - 1, out=col)
    np.clip(row, 0, height - 1, out=row)

    hits = np.bincount(row * width + col, minlength=width * height)
    hits = hits.reshape(height, width)

    # Everything is unknown until a cell is either hit enough times to be an
    # obstacle, or the sensor clearly saw through it. A single stray return
    # should not become a wall, hence the threshold.
    grid = np.full((height, width), 255, dtype=np.uint8)
    grid[hits > 0] = 0
    grid[hits >= hit_threshold] = 100
    return grid, float(min_xy[0]), float(min_xy[1])


def flood_free(grid, seed_rc, seal):
    """Mark everything reachable from the sensor's start as free space.

    Rasterising hits alone leaves most of the map unknown: a cell only becomes
    free if a beam happened to stop in it, while every cell a beam merely
    passed through stays unknown. Nav2 will not plan through unknown, so a map
    built that way has almost nowhere to drive even though the room is empty.

    Proper ray tracing needs the sensor pose for every scan, which the
    accumulated .pcd does not carry. Flood filling from where the sensor
    started is the next best thing and is exact for the case that matters: an
    enclosed space whose walls the walk actually closed. Anything the flood
    cannot reach - behind a wall, outside the building - correctly stays
    unknown.
    """
    height, width = grid.shape
    free = np.zeros(grid.shape, dtype=bool)
    blocked = grid == 100

    # Walls come out of a real scan with holes in them - a doorway the walk
    # never entered, a window, a stretch the beam grazed. The fill escapes
    # through any one of them and floods the outdoors, which is far worse than
    # leaving the map unknown: Nav2 would happily plan a route through a wall.
    # Thickening the obstacles seals gaps up to 2*seal cells wide, and the
    # thickening is discarded afterwards so the walls keep their real position.
    if seal > 0:
        sealed = blocked.copy()
        for _ in range(seal):
            grown = sealed.copy()
            grown[1:, :] |= sealed[:-1, :]
            grown[:-1, :] |= sealed[1:, :]
            grown[:, 1:] |= sealed[:, :-1]
            grown[:, :-1] |= sealed[:, 1:]
            sealed = grown
        blocked = sealed

    row, col = seed_rc
    row = int(np.clip(row, 0, height - 1))
    col = int(np.clip(col, 0, width - 1))
    if blocked[row, col]:
        return grid, 0

    # Iterative row-scan fill: fast enough on a map this size and immune to the
    # recursion limit a naive flood fill would hit.
    stack = [(row, col)]
    free[row, col] = True
    while stack:
        r, c = stack.pop()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < height and 0 <= nc < width \
                    and not free[nr, nc] and not blocked[nr, nc]:
                free[nr, nc] = True
                stack.append((nr, nc))

    out = grid.copy()
    out[free] = 0
    return out, int(free.sum())


def write_map(grid, origin_x, origin_y, resolution, stem):
    """Write the .pgm/.yaml pair map_server loads."""
    height, width = grid.shape

    # map_server reads PGM with white = free and black = occupied, and reads
    # rows top-down while the map frame counts y upwards, so the raster is
    # flipped on write.
    shade = np.full(grid.shape, 205, dtype=np.uint8)   # unknown
    shade[grid == 0] = 254                             # free
    shade[grid == 100] = 0                             # occupied
    shade = np.flipud(shade)

    pgm_path = f'{stem}.pgm'
    with open(pgm_path, 'wb') as handle:
        handle.write(b'P5\n')
        handle.write(b'# generated by go2_control pcd_to_map\n')
        handle.write(f'{width} {height}\n255\n'.encode('ascii'))
        handle.write(shade.tobytes())

    yaml_path = f'{stem}.yaml'
    with open(yaml_path, 'w') as handle:
        handle.write(
            f'image: {os.path.basename(pgm_path)}\n'
            f'resolution: {resolution}\n'
            f'origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.196\n'
        )
    return pgm_path, yaml_path


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Slice a FAST-LIO2 .pcd into a Nav2 2D occupancy map.')
    parser.add_argument('pcd', help='input .pcd, e.g. .../PCD/scans.pcd')
    parser.add_argument(
        '-o', '--out', required=True,
        help='output path without extension; .pgm and .yaml are written')
    parser.add_argument('--resolution', type=float, default=0.05,
                        help='metres per cell (default 0.05, Nav2 convention)')
    parser.add_argument('--z-min', type=float, default=-1.2,
                        help='bottom of the slice, sensor frame (default -1.2)')
    parser.add_argument('--z-max', type=float, default=-0.2,
                        help='top of the slice, sensor frame (default -0.2)')
    parser.add_argument(
        '--hit-threshold', type=int, default=3,
        help='points in a cell before it counts as occupied (default 3)')
    parser.add_argument('--padding', type=float, default=1.0,
                        help='metres of margin around the cloud (default 1.0)')
    parser.add_argument(
        '--pitch-deg', type=float, default=13.0,
        help='mount tilt to undo before slicing, positive nose-down '
             '(default 13.0, the Mid-360 mount on the Go2; use 0 for a '
             'handheld run held level)')
    parser.add_argument('--roll-deg', type=float, default=0.0,
                        help='mount roll to undo before slicing (default 0)')
    parser.add_argument(
        '--seal', type=int, default=5,
        help='cells to thicken obstacles by before the flood fill, to close '
             'gaps in walls so the fill cannot escape outdoors (default 5, '
             'sealing holes up to about 50 cm at 5 cm resolution). Measured on '
             'a real room: 3 was not enough and the fill escaped through a '
             'doorway, flooding the outdoors as free space. Raise it if free '
             'space still spills outside the building; lower it if narrow but '
             'genuine passages get sealed off')
    parser.add_argument(
        '--no-fill-free', action='store_true',
        help='skip the flood fill that marks reachable cells as free space. '
             'Without the fill almost the whole map stays unknown and Nav2 has '
             'nowhere to plan, so only use this to inspect raw sensor coverage')
    args = parser.parse_args(argv)

    try:
        points = read_pcd(args.pcd)
    except (OSError, PCDError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1

    if points.size == 0:
        print('error: cloud is empty', file=sys.stderr)
        return 1

    points = level(points, args.pitch_deg, args.roll_deg)

    lo = points.min(axis=0)
    hi = points.max(axis=0)
    print(f'points     {len(points)}')
    print(f'extent x   {lo[0]:7.2f} .. {hi[0]:7.2f} m')
    print(f'extent y   {lo[1]:7.2f} .. {hi[1]:7.2f} m')
    print(f'extent z   {lo[2]:7.2f} .. {hi[2]:7.2f} m')
    print(f'slice z    {args.z_min:7.2f} .. {args.z_max:7.2f} m')
    print(f'levelled   pitch {args.pitch_deg:.1f} deg  roll {args.roll_deg:.1f} deg')

    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    try:
        grid, origin_x, origin_y = rasterise(
            points, args.resolution, args.z_min, args.z_max,
            args.hit_threshold, args.padding)
    except PCDError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1

    if not args.no_fill_free:
        # The sensor starts at the origin of FAST-LIO's world frame, so that is
        # a cell the robot demonstrably occupied and therefore free.
        seed = (int(round(-origin_y / args.resolution)),
                int(round(-origin_x / args.resolution)))
        grid, filled = flood_free(grid, seed, args.seal)
        print(f'flood fill  seed cell {seed}  ->  {filled} free cells')

    pgm_path, yaml_path = write_map(
        grid, origin_x, origin_y, args.resolution, args.out)

    total = grid.size
    occupied = int((grid == 100).sum())
    free = int((grid == 0).sum())
    print(f'grid       {grid.shape[1]} x {grid.shape[0]} cells '
          f'@ {args.resolution} m')
    print(f'occupied   {occupied} cells ({100.0 * occupied / total:.1f}%)')
    print(f'free       {free} cells ({100.0 * free / total:.1f}%)')
    print(f'wrote      {pgm_path}')
    print(f'wrote      {yaml_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
