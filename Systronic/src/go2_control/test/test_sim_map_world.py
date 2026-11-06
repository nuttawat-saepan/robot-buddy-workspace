"""A saved map becomes walls in the right place."""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import struct

from go2_control.sim_map_world import (
    cell_blocks, rectangles, stl_bytes, world_sdf)


class Blocks(unittest.TestCase):

    def test_covers_exactly_the_occupied_cells(self):
        np.random.seed(1)
        mask = np.random.random((40, 50)) < 0.3
        painted = np.zeros(mask.shape, bool)
        for col, row, cols, rows in cell_blocks(mask):
            self.assertFalse(painted[row:row + rows, col:col + cols].any())
            painted[row:row + rows, col:col + cols] = True
        self.assertTrue((painted == mask).all())

    def test_a_long_wall_is_one_block(self):
        mask = np.zeros((20, 20), bool)
        mask[2:18, 5] = True
        self.assertEqual(cell_blocks(mask), [(5, 2, 1, 16)])


class Placement(unittest.TestCase):

    def test_top_row_of_the_image_is_the_top_of_the_map(self):
        image = np.full((10, 10), 254, np.uint8)
        image[0, 2] = 0
        x_min, y_min, x_max, y_max, _ = rectangles(image, 0.5, -1.0, -2.0)[0]
        self.assertAlmostEqual(x_min, 0.0)
        self.assertAlmostEqual(x_max, 0.5)
        self.assertAlmostEqual(y_min, 2.5)
        self.assertAlmostEqual(y_max, 3.0)

    def test_unknown_grey_is_not_a_wall(self):
        self.assertEqual(
            rectangles(np.full((5, 5), 205, np.uint8), 0.05, 0.0, 0.0), [])

    def test_the_mesh_holds_every_wall(self):
        stl = stl_bytes([(0, 0, 1, 1, 2.0), (2, 2, 3, 3, 2.0)])
        self.assertEqual(struct.unpack('<I', stl[80:84])[0], 24)
        self.assertEqual(len(stl), 84 + 24 * 50)

    def test_the_world_uses_the_mesh(self):
        self.assertIn('model://m_walls.stl', world_sdf('m', 'model://m_walls.stl'))


if __name__ == '__main__':
    unittest.main()
