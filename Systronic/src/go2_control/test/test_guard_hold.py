"""A mission must not wait forever on a robot the collision guard is holding."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_control.mqtt_mission_bridge import GuardHold


class Hold(unittest.TestCase):

    def test_time_runs_while_commands_are_being_refused(self):
        hold = GuardHold()
        hold.update('front', 10, 100.0)
        hold.update('front', 20, 100.5)
        self.assertEqual(hold.seconds(130.0), 30.0)

    def test_a_command_getting_through_starts_it_again(self):
        hold = GuardHold()
        hold.update('front', 10, 100.0)
        hold.update(None, 10, 100.5)
        self.assertEqual(hold.seconds(130.0), 0.0)
        hold.update('front', 11, 131.0)
        self.assertEqual(hold.seconds(132.0), 1.0)

    def test_a_flag_left_set_with_nothing_being_refused_is_not_a_hold(self):
        # Paused: nothing is being sent, and the flag has not been cleared.
        hold = GuardHold()
        hold.update('front', 10, 100.0)
        hold.update('front', 10, 100.5)
        self.assertEqual(hold.seconds(130.0), 0.0)

    def test_nothing_heard_is_no_hold(self):
        self.assertEqual(GuardHold().seconds(100.0), 0.0)


if __name__ == '__main__':
    unittest.main()
