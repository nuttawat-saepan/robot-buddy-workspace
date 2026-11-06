"""The arithmetic the Gazebo obstacle results depend on."""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_control.sim_actuator import step_towards
from go2_control.sim_scenarios import (
    DOOR_WIDTH, SCENARIOS, START, WALLS, body_gap, box_at, map_image)


class BodyGap(unittest.TestCase):

    def test_box_ahead_is_measured_from_the_nose(self):
        box = box_at(2.0, 0.0, size=0.5)
        self.assertAlmostEqual(body_gap(0, 0, 0, [box]), 2.0 - 0.25 - 0.35, 2)

    def test_turning_brings_the_corner_closer(self):
        box = box_at(0.9, 0.0, size=0.2)
        straight = body_gap(0, 0, 0, [box])
        turned = body_gap(0, 0, np.pi / 4, [box])
        self.assertAlmostEqual(straight, 0.45, 2)
        self.assertLess(turned, straight)

    def test_overlap_is_zero(self):
        self.assertEqual(body_gap(0, 0, 0, [box_at(0.3, 0.0)]), 0.0)


class Room(unittest.TestCase):

    def test_start_and_goals_are_clear_of_the_walls(self):
        self.assertGreater(body_gap(*START, 0.0, WALLS), 1.0)
        for scenario in SCENARIOS.values():
            self.assertGreater(
                body_gap(*scenario.goal, 0.0, WALLS), 0.3, scenario.name)

    def test_the_doorway_fits_the_body_with_0_2_m_each_side(self):
        gap = body_gap(11.05, 0.0, 0.0, WALLS)
        self.assertAlmostEqual(gap, (DOOR_WIDTH - 0.50) / 2, 2)

    def test_map_shows_the_doorway_open_and_the_wall_closed(self):
        image, origin, resolution = map_image()

        def cell(x, y):
            col = int((x - origin[0]) / resolution)
            row = image.shape[0] - 1 - int((y - origin[1]) / resolution)
            return image[row, col]

        self.assertEqual(cell(11.05, 0.0), 254)
        self.assertEqual(cell(11.05, 1.0), 0)
        self.assertEqual(cell(5.0, 0.0), 254)


class Actuator(unittest.TestCase):

    def test_velocity_changes_only_as_fast_as_the_limit(self):
        self.assertAlmostEqual(step_towards(0.4, 0.0, 0.02), 0.38)
        self.assertAlmostEqual(step_towards(0.0, 0.4, 0.02), 0.02)
        self.assertEqual(step_towards(0.01, 0.0, 0.02), 0.0)


if __name__ == '__main__':
    unittest.main()
