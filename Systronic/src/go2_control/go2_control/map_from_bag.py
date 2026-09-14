#!/usr/bin/env python3
"""Recover a map_server map from an OccupancyGrid recorded in a bag.

A map lost off the board is not necessarily gone. Anything that recorded
/map caught the whole grid in one message - the cells, the resolution and
the origin are all in it - so a run bagged while the map was loaded can be
turned back into the .pgm and .yaml that map_server wants.

That is how the 2026-09-14 site map came back after a deploy deleted it: a
twelve-second test recording had exactly one /map message in it.

    ros2 run go2_control map_from_bag --bag ~/bags/run --out ~/maps/site

Reads the sqlite directly rather than through rosbag2_py, which Foxy does
not ship with Python bindings for.

The image is written in the same convention map_server reads: row 0 of the
PGM is the top of the picture and the last row of the grid, occupied cells
dark, unknown mid-grey. The yaml's image: line is a bare filename, which is
the form map_server wants and the form that has now failed twice when it was
written as a path.
"""
import argparse
import os
import sqlite3
import sys


def load_grid(bag_path, topic='/map'):
    """Return the last OccupancyGrid published on `topic` in the bag."""
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import OccupancyGrid

    if os.path.isdir(bag_path):
        dbs = sorted(f for f in os.listdir(bag_path) if f.endswith('.db3'))
        if not dbs:
            raise SystemExit('no .db3 in %s' % bag_path)
        bag_path = os.path.join(bag_path, dbs[-1])

    con = sqlite3.connect(bag_path)
    cur = con.cursor()
    rows = cur.execute(
        'SELECT id, name, type FROM topics WHERE name = ?', (topic,)).fetchall()
    if not rows:
        names = [r[0] for r in cur.execute('SELECT name FROM topics')]
        raise SystemExit('%s is not in this bag. It has:\n  %s'
                         % (topic, '\n  '.join(names)))
    topic_id = rows[0][0]

    # The last one, not the first: a map republished after an edit or a
    # relaunch is the one worth keeping.
    blob = cur.execute(
        'SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp DESC'
        ' LIMIT 1', (topic_id,)).fetchone()
    if blob is None:
        raise SystemExit('%s exists in the bag but carries no messages'
                         % topic)
    return deserialize_message(bytes(blob[0]), OccupancyGrid)


def write_map(grid, out_base):
    info = grid.info
    w, h = info.width, info.height
    pgm_path = out_base + '.pgm'
    yaml_path = out_base + '.yaml'

    # OccupancyGrid is row-major from the bottom-left; PGM is written from the
    # top row down. Reversing the rows is what keeps north pointing the same
    # way in both, and getting it wrong produces a mirrored map that looks
    # plausible until the robot drives through a wall.
    rows = []
    for y in range(h - 1, -1, -1):
        line = bytearray(w)
        base = y * w
        for x in range(w):
            v = grid.data[base + x]
            if v < 0:
                line[x] = 205          # unknown
            elif v >= 65:
                line[x] = 0            # occupied
            elif v <= 25:
                line[x] = 254          # free
            else:
                line[x] = 205
        rows.append(bytes(line))

    with open(pgm_path, 'wb') as fh:
        fh.write(b'P5\n%d %d\n255\n' % (w, h))
        for line in rows:
            fh.write(line)

    q = info.origin.orientation
    import math
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    with open(yaml_path, 'w') as fh:
        fh.write('image: %s\n' % os.path.basename(pgm_path))
        fh.write('mode: trinary\n')
        fh.write('resolution: %.6f\n' % info.resolution)
        fh.write('origin: [%.6f, %.6f, %.6f]\n'
                 % (info.origin.position.x, info.origin.position.y, yaw))
        fh.write('negate: 0\n')
        fh.write('occupied_thresh: 0.65\n')
        fh.write('free_thresh: 0.25\n')

    return pgm_path, yaml_path, w, h, info.resolution


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--bag', required=True,
                        help='Bag directory or .db3 file.')
    parser.add_argument('--out', required=True,
                        help='Output path without extension.')
    parser.add_argument('--topic', default='/map')
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    grid = load_grid(args.bag, args.topic)
    pgm, yaml_path, w, h, res = write_map(grid, os.path.expanduser(args.out))

    print('%s  %dx%d at %.3f m/cell = %.1f x %.1f m'
          % (yaml_path, w, h, res, w * res, h * res))
    print(pgm)
    print('\npoint SITE_MAP at the yaml, then relaunch - map_server reads it '
          'once at configure')


if __name__ == '__main__':
    main()
