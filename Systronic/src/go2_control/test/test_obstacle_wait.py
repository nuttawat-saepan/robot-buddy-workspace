"""A waypoint with something new on it waits, and ends the mission if it stays."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_control.mqtt_mission_bridge import (
    MqttMissionBridge, new_obstacle_near, scan_points)


def grid(cells=()):
    data = [0] * 400
    for col, row, value in cells:
        data[row * 20 + col] = value
    return SimpleNamespace(data=data, info=SimpleNamespace(
        width=20, height=20, resolution=0.1,
        origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0))))


def at(x, y, count=3):
    return [(x, y)] * count


class NearTest(unittest.TestCase):

    def test_unknown_without_a_scan_or_a_map(self):
        self.assertIsNone(new_obstacle_near(None, grid(), 1.0, 1.0, 0.3))
        self.assertIsNone(new_obstacle_near([], None, 1.0, 1.0, 0.3))

    def test_new_obstacle_counts(self):
        self.assertTrue(new_obstacle_near(
            at(1.05, 1.05), grid(), 1.05, 1.05, 0.3))

    def test_one_stray_return_does_not(self):
        self.assertFalse(new_obstacle_near(
            at(1.05, 1.05, count=2), grid(), 1.05, 1.05, 0.3))

    def test_map_wall_does_not_count(self):
        self.assertFalse(new_obstacle_near(
            at(1.05, 1.05), grid([(10, 10, 100)]), 1.05, 1.05, 0.3))

    def test_scan_off_a_wall_by_the_localisation_error_is_that_wall(self):
        self.assertFalse(new_obstacle_near(
            at(1.05, 1.05), grid([(13, 10, 100)]), 1.05, 1.05, 0.3))

    def test_something_clear_of_the_walls_still_counts(self):
        self.assertTrue(new_obstacle_near(
            at(1.05, 1.05), grid([(15, 10, 100)]), 1.05, 1.05, 0.3))

    def test_far_obstacle_does_not_count(self):
        self.assertFalse(new_obstacle_near(
            at(1.95, 1.95), grid(), 0.5, 0.5, 0.3))


class ScanPointsTest(unittest.TestCase):

    def scan(self, ranges):
        return SimpleNamespace(ranges=ranges, range_min=0.1, range_max=40.0,
                               angle_min=0.0, angle_increment=math.pi / 2)

    def test_a_return_lands_where_the_sensor_is_looking(self):
        (px, py), = scan_points(self.scan([2.0]), 1.0, 1.0, math.pi / 2, 6.0)
        self.assertAlmostEqual(px, 1.0)
        self.assertAlmostEqual(py, 3.0)

    def test_inf_and_out_of_range_are_left_out(self):
        scan = self.scan([float('inf'), 7.0, 0.05, float('nan')])
        self.assertEqual(scan_points(scan, 0.0, 0.0, 0.0, 6.0), [])

    def test_gone_means_gone(self):
        # The case that kept a waypoint blocked after the box was taken away.
        points = scan_points(self.scan([float('inf')]), 0.0, 0.0, 0.0, 6.0)
        self.assertFalse(new_obstacle_near(points, grid(), 1.0, 0.0, 0.3))


def node(blocked, go=True):
    state = SimpleNamespace(
        stopping=False, obstacle_wait=0.2, blocked_during_goal=False,
        fail_reason=None, get_logger=lambda: mock.Mock(), _progress=mock.Mock())
    state._goal_blocked = mock.Mock(side_effect=blocked)
    state._go_to = mock.Mock(return_value=go)
    return state


class WaitTest(unittest.TestCase):

    def run_it(self, state):
        with mock.patch('time.sleep'):
            return MqttMissionBridge._go_when_clear(
                state, {'x': 1, 'y': 1}, 0, 'WP', 0)

    def test_clear_goes_straight(self):
        state = node(lambda _wp: False)
        self.assertTrue(self.run_it(state))
        state._go_to.assert_called_once()

    def test_goes_once_the_obstacle_clears(self):
        answers = iter([True, True, False])
        state = node(lambda _wp: next(answers))
        self.assertTrue(self.run_it(state))
        state._go_to.assert_called_once()
        self.assertIn('Waiting', state._progress.call_args_list[0][0][3])

    def test_fails_when_it_stays(self):
        state = node(lambda _wp: True)
        clock = iter([0.0, 0.1, 0.3])
        with mock.patch('time.monotonic', lambda: next(clock)):
            self.assertFalse(self.run_it(state))
        state._go_to.assert_not_called()
        self.assertIn('did not clear', state.fail_reason)

    def test_stop_ends_the_wait(self):
        state = node(lambda _wp: True)
        state.stopping = True
        self.assertFalse(self.run_it(state))

    def test_ordinary_failure_is_not_retried(self):
        state = node(lambda _wp: False, go=False)
        self.assertFalse(self.run_it(state))
        state._go_to.assert_called_once()


if __name__ == '__main__':
    unittest.main()
