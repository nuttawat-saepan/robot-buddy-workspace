"""The stop line: 0.20 m from the body, in the direction of travel."""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_control.collision_zone import (
    FRONT, GuardState, REAR, TURN_LEFT, TURN_RIGHT,
    obstacle_points, to_base, zone_counts, zones_for_command)


def cluster(x, y, count=10):
    return np.tile([x, y], (count, 1))


def refused(x, y, vx, wz):
    state = GuardState()
    state.update(cluster(x, y), 100.0)
    return state.refusal(vx, wz, 100.01)


class StraightLine(unittest.TestCase):

    def test_stops_inside_the_line_ahead(self):
        self.assertEqual(refused(0.54, 0.0, 0.2, 0.0), FRONT)

    def test_goes_on_outside_the_line_ahead(self):
        self.assertIsNone(refused(0.56, 0.0, 0.2, 0.0))

    def test_may_back_away_from_something_ahead(self):
        self.assertIsNone(refused(0.40, 0.0, -0.1, 0.0))

    def test_stops_backing_into_something(self):
        self.assertEqual(refused(-0.50, 0.0, -0.1, 0.0), REAR)

    def test_a_jamb_of_a_0_9_m_doorway_does_not_stop_it(self):
        self.assertIsNone(refused(0.50, 0.45, 0.2, 0.0))
        self.assertIsNone(refused(0.50, -0.45, 0.2, 0.0))

    def test_steering_trim_does_not_count_as_a_turn(self):
        self.assertIsNone(refused(0.50, 0.45, 0.2, 0.05))


class Turning(unittest.TestCase):

    def test_front_corner_swinging_towards_a_wall_stops(self):
        # Turning left swings the front left corner into something on its left.
        self.assertEqual(refused(0.30, 0.32, 0.0, 0.3), TURN_LEFT)

    def test_turning_the_other_way_is_free(self):
        self.assertIsNone(refused(0.30, 0.32, 0.0, -0.3))

    def test_rear_corner_swings_the_opposite_way(self):
        self.assertEqual(refused(-0.30, 0.32, 0.0, -0.3), TURN_RIGHT)

    def test_nothing_outside_the_ring_stops_a_turn(self):
        self.assertIsNone(refused(0.60, 0.50, 0.0, 0.3))

    def test_a_box_beside_the_body_clear_of_the_corners_allows_a_turn(self):
        # The corner sweeps a 0.43 m radius, so 0.52 m to the side is never
        # reached. This is the case that left the simulated robot stuck.
        for x in (0.10, 0.30):
            self.assertIsNone(refused(x, 0.52, 0.0, 0.3))
            self.assertIsNone(refused(x, 0.52, 0.0, -0.3))

    def test_stopped_at_the_line_it_can_still_turn_away(self):
        self.assertIsNone(refused(0.50, 0.0, 0.0, 0.3))
        self.assertIsNone(refused(0.50, 0.0, 0.0, -0.3))

    def test_a_point_inside_the_corner_sweep_stops_that_turn(self):
        self.assertEqual(refused(0.25, 0.30, 0.0, 0.3), TURN_LEFT)
        self.assertIsNone(refused(0.25, 0.30, 0.0, -0.3))


class WhatCounts(unittest.TestCase):

    def test_a_stray_point_does_not_stop_it(self):
        state = GuardState()
        state.update(cluster(0.50, 0.0, count=4), 100.0)
        self.assertIsNone(state.refusal(0.2, 0.0, 100.01))

    def test_stays_stopped_until_clear_for_the_release_time(self):
        state = GuardState()
        state.update(cluster(0.50, 0.0), 100.0)
        for now in (100.1, 100.2, 100.3, 100.4):
            state.update(np.empty((0, 2)), now)
        self.assertEqual(state.refusal(0.2, 0.0, 100.45), FRONT)
        state.update(np.empty((0, 2)), 100.5)
        self.assertIsNone(state.refusal(0.2, 0.0, 100.55))

    def test_no_frame_yet_is_not_clear(self):
        self.assertEqual(GuardState().refusal(0.2, 0.0, 100.0),
                         'no recent LiDAR frame')

    def test_old_frame_is_not_clear(self):
        state = GuardState()
        state.update(np.empty((0, 2)), 100.0)
        self.assertIsNone(state.refusal(0.2, 0.0, 100.2))
        self.assertEqual(state.refusal(0.2, 0.0, 100.4),
                         'no recent LiDAR frame')

    def test_zero_always_passes_even_with_no_frame(self):
        self.assertIsNone(GuardState().refusal(0.0, 0.0, 100.0))
        self.assertEqual(zones_for_command(0.0, 0.0), [])


class Points(unittest.TestCase):

    def test_body_and_the_thing_on_its_nose_are_ignored(self):
        base = np.array([
            [0.0, 0.20, 0.5],    # inside the body
            [0.37, 0.10, 0.5],   # the part seen standing still
            [0.37, 0.20, 0.5],   # beside that part, so not the robot
            [0.50, 0.0, 0.1],    # below the band
            [0.50, 0.0, 1.2],    # above the band
        ])
        self.assertEqual(obstacle_points(base).tolist(), [[0.37, 0.20]])

    def test_the_mount_puts_a_point_where_a_tape_measure_would(self):
        # One metre along the sensor's nose, tilted down 13 degrees from 0.35 m
        # up, and 0.30 m ahead of the middle of the robot.
        base = to_base(np.array([[1.0, 0.0, 0.0]]), 13.0, 0.0, 0.30, 0.35)
        self.assertAlmostEqual(base[0, 0], 0.30 + np.cos(np.radians(13)), 5)
        self.assertAlmostEqual(base[0, 2], 0.35 - np.sin(np.radians(13)), 5)

    def test_counts_cover_every_zone(self):
        self.assertEqual(set(zone_counts(np.empty((0, 2)))),
                         {FRONT, REAR, TURN_LEFT, TURN_RIGHT})


if __name__ == '__main__':
    unittest.main()
