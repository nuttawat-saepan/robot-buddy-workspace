"""The manual drive page for the simulation: limits, the dead man, refusals."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_control.sim_manual_drive import (
    Deadman, joystick_command, limit, status_text)


class Limit(unittest.TestCase):

    def test_clamps_both_ways(self):
        self.assertEqual(limit(5.0, 0.45), 0.45)
        self.assertEqual(limit(-5.0, 0.45), -0.45)
        self.assertEqual(limit(0.2, 0.45), 0.2)

    def test_anything_not_a_number_is_a_stop(self):
        for bad in (None, 'fast', [], float('nan')):
            self.assertEqual(limit(bad, 0.45), 0.0)


class DeadmanTimer(unittest.TestCase):

    def test_nothing_before_the_first_command(self):
        self.assertIsNone(Deadman(0.4).current(100.0))

    def test_holds_while_commands_arrive_then_lets_go(self):
        deadman = Deadman(0.4)
        deadman.set(0.2, 0.1, 100.0)
        self.assertEqual(deadman.current(100.3), (0.2, 0.1))
        self.assertIsNone(deadman.current(100.5))

    def test_clear_drops_it_at_once(self):
        deadman = Deadman(0.4)
        deadman.set(0.2, 0.1, 100.0)
        deadman.clear()
        self.assertIsNone(deadman.current(100.1))


class Joystick(unittest.TestCase):

    def test_reads_what_the_backend_forwards(self):
        self.assertEqual(joystick_command(
            '{"linear": {"x": 0.4, "y": 0, "z": 0},'
            ' "angular": {"x": 0, "y": 0, "z": -0.4}}'), (0.4, -0.4))

    def test_anything_else_is_ignored(self):
        for bad in ('', 'stop', '{"linear": 1}', '[]'):
            self.assertIsNone(joystick_command(bad))


class Status(unittest.TestCase):

    def test_ready(self):
        self.assertFalse(status_text(False, None)[1])

    def test_a_running_goal_refuses(self):
        self.assertTrue(status_text(True, None)[1])

    def test_the_guard_refusal_reaches_the_page(self):
        message, blocked = status_text(False, 'front')
        self.assertTrue(blocked)
        self.assertIn('front', message)


if __name__ == '__main__':
    unittest.main()
