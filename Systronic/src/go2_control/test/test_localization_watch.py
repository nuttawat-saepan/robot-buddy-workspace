"""Check the disagreement between FAST-LIO and AMCL used by localization_watch."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_control.localization_watch import disagreement


class DisagreementTest(unittest.TestCase):

    def test_same_correction_is_zero(self):
        distance, yaw = disagreement((1, 2, 0.5), (1, 2, 0.5), (4, -3, 1.0))
        self.assertAlmostEqual(distance, 0.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_translation_shows_directly(self):
        distance, _ = disagreement((0, 0, 0), (0.6, 0.8, 0), (5, 5, 0))
        self.assertAlmostEqual(distance, 1.0)

    def test_small_yaw_moves_a_far_robot(self):
        # map->odom translation is unchanged, but 10 m from the odom origin a
        # small rotation of the correction moves the robot well over a metre.
        distance, yaw = disagreement((0, 0, 0), (0, 0, 0.2), (10, 0, 0))
        self.assertGreater(distance, 1.0)
        self.assertAlmostEqual(yaw, 0.2)

    def test_yaw_wraps(self):
        _, yaw = disagreement((0, 0, math.pi - 0.1), (0, 0, -math.pi + 0.1), (0, 0, 0))
        self.assertAlmostEqual(yaw, 0.2)


if __name__ == '__main__':
    unittest.main()
