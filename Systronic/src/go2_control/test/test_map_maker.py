"""The map page: names, the yaml fix, the preview image."""

from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_control.map_maker import (
    check_name, default_name, fix_image_line, grid_rgb, known_area, png)


class Names(unittest.TestCase):

    def test_default_is_a_site_name(self):
        self.assertRegex(default_name(0), r'^site_\d{8}_\d{4}$')

    def test_rejects_paths_and_odd_characters(self):
        with tempfile.TemporaryDirectory() as d:
            for bad in ('', '../x', 'a b', 'map.yaml', 'x' * 61):
                self.assertIsNotNone(check_name(bad, d))
            self.assertIsNone(check_name('site_20261106_1400', d))

    def test_never_overwrites(self):
        with tempfile.TemporaryDirectory() as d:
            Path(d, 'room.yaml').write_text('image: room.pgm\n')
            self.assertIsNotNone(check_name('room', d))


class YamlFix(unittest.TestCase):

    def test_full_path_becomes_a_bare_name(self):
        text = 'image: /home/unitree/maps/site_1.pgm\nresolution: 0.05\n'
        self.assertEqual(fix_image_line(text),
                         'image: site_1.pgm\nresolution: 0.05\n')

    def test_bare_name_is_left_alone(self):
        self.assertEqual(fix_image_line('image: a.pgm\n'), 'image: a.pgm\n')


class Image(unittest.TestCase):

    def test_colours_and_the_top_row_is_the_far_side(self):
        # 2x2 grid, row 0 is the bottom of the map.
        rows = grid_rgb([0, 100, -1, 0], 2, 2)
        self.assertEqual(bytes(rows[0]), b'\xa0\xa0\xa0\xff\xff\xff')
        self.assertEqual(bytes(rows[1]), b'\xff\xff\xff\x10\x10\x10')

    def test_png_size(self):
        data = png(grid_rgb([0] * 6, 3, 2), scale=2)
        self.assertEqual(data[:8], b'\x89PNG\r\n\x1a\n')
        self.assertEqual(struct.unpack('>II', data[16:24]), (6, 4))

    def test_known_area(self):
        self.assertAlmostEqual(known_area([0, 100, -1, -1], 0.5), 0.5)


if __name__ == '__main__':
    unittest.main()
