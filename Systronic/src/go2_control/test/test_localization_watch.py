"""Check how localization_watch compares FAST-LIO with AMCL and gates the result."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_control.localization_watch import (
    Gate, compose, count_clusters, disagreement, inverse)


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

    def test_inverse_undoes_compose(self):
        a = (1.5, -2.0, 0.7)
        x, y, yaw = compose(a, inverse(a))
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(yaw, 0.0)


class GateTest(unittest.TestCase):

    ODOM = (0.0, 0.0, 0.0)

    def setUp(self):
        self.gate = Gate(limit=1.0, hold_seconds=3.0)
        self.gate.update((0.0, 0.0, 0.0), self.ODOM, now=0.0)

    def test_first_correction_is_the_anchor(self):
        gate = Gate(limit=1.0, hold_seconds=3.0)
        gate.update((50.0, 50.0, 1.0), self.ODOM, now=0.0)
        self.assertEqual(gate.accepted, (50.0, 50.0, 1.0))
        self.assertFalse(gate.warning)

    def test_within_limit_is_used(self):
        self.gate.update((0.9, 0.0, 0.0), self.ODOM, now=1.0)
        self.assertEqual(self.gate.accepted, (0.9, 0.0, 0.0))
        self.assertFalse(self.gate.rejecting)

    def test_beyond_limit_keeps_the_last_good(self):
        self.gate.update((0.5, 0.0, 0.0), self.ODOM, now=1.0)
        self.gate.update((3.0, 0.0, 0.0), self.ODOM, now=2.0)
        self.assertEqual(self.gate.accepted, (0.5, 0.0, 0.0))
        self.assertTrue(self.gate.warning)
        self.assertTrue(self.gate.rejecting)

    def test_creep_is_measured_from_the_anchor(self):
        for i in range(1, 16):
            self.gate.update((0.1 * i, 0.0, 0.0), self.ODOM, now=float(i))
        self.assertAlmostEqual(self.gate.accepted[0], 1.0)
        self.assertTrue(self.gate.rejecting)

    def test_lost_only_after_hold(self):
        self.gate.update((3.0, 0.0, 0.0), self.ODOM, now=10.0)
        self.assertFalse(self.gate.lost(12.9))
        self.assertTrue(self.gate.lost(13.0))

    def test_return_within_limit_ends_rejection(self):
        self.gate.update((3.0, 0.0, 0.0), self.ODOM, now=10.0)
        self.gate.update((0.5, 0.0, 0.0), self.ODOM, now=20.0)
        self.assertEqual(self.gate.accepted, (0.5, 0.0, 0.0))
        self.assertFalse(self.gate.lost(30.0))
        self.assertFalse(self.gate.warning)

    def test_warning_clears_below_margin_only(self):
        self.gate.update((1.5, 0.0, 0.0), self.ODOM, now=1.0)
        self.gate.update((0.9, 0.0, 0.0), self.ODOM, now=2.0)
        self.assertTrue(self.gate.warning)
        self.gate.update((0.7, 0.0, 0.0), self.ODOM, now=3.0)
        self.assertFalse(self.gate.warning)


class ClusterTest(unittest.TestCase):

    def test_empty(self):
        self.assertEqual(count_clusters([]), 0)

    def test_one_group(self):
        self.assertEqual(
            count_clusters([(0.1 * i, 0.0) for i in range(20)]), 1)

    def test_two_groups(self):
        points = [(0.0, 0.01 * i) for i in range(10)]
        points += [(5.0, 0.01 * i) for i in range(10)]
        self.assertEqual(count_clusters(points), 2)

    def test_stragglers_are_ignored(self):
        points = [(0.0, 0.0)] * 99 + [(9.0, 9.0)]
        self.assertEqual(count_clusters(points), 1)


if __name__ == '__main__':
    unittest.main()
