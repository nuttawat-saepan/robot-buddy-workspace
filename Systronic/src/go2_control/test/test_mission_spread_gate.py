"""A web mission is refused on a measured wide cloud, and only then."""

from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_control.mqtt_mission_bridge import MqttMissionBridge


def bridge(spread, enforce=True):
    return SimpleNamespace(
        worker=None, lock=threading.Lock(), mission=None, spread=spread,
        enforce_spread_limit=enforce, paused=True, stopping=True,
        get_logger=lambda: mock.Mock(), _status=mock.Mock(),
        _run_mission=mock.Mock())


MISSION = {'missionId': 'm', 'runId': 'r', 'waypoints': [{'sequence': 0}]}


class SpreadGateTest(unittest.TestCase):

    def start(self, node):
        with mock.patch('threading.Thread') as thread:
            MqttMissionBridge._start_mission(node, dict(MISSION))
        return thread

    def test_wide_cloud_is_refused(self):
        node = bridge(0.8)
        thread = self.start(node)
        thread.assert_not_called()
        self.assertEqual(node._status.call_args[0][0], 'FAILED')
        self.assertIsNone(node.mission)

    def test_tight_cloud_starts(self):
        self.start_ok(bridge(0.2))

    def test_unmeasured_cloud_starts(self):
        self.start_ok(bridge(None))

    def test_advisory_mode_starts_on_wide_cloud(self):
        self.start_ok(bridge(0.8, enforce=False))

    def start_ok(self, node):
        thread = self.start(node)
        thread.assert_called_once()
        node._status.assert_not_called()


if __name__ == '__main__':
    unittest.main()
