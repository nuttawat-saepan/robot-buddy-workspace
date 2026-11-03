"""The bridge pauses a running mission when the guard asks, and stays paused."""

import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_control.mqtt_mission_bridge import MqttMissionBridge


def bridge(running=True, paused=False):
    return SimpleNamespace(
        localization_check=None, paused=paused,
        worker=SimpleNamespace(is_alive=lambda: running),
        _cancel_current=mock.Mock(), get_logger=lambda: mock.Mock())


def status(**fields):
    return SimpleNamespace(data=json.dumps(fields))


class GuardPauseTest(unittest.TestCase):

    def test_pause_request_stops_a_running_mission(self):
        node = bridge()
        MqttMissionBridge._on_localization_check(node, status(pause=True, distance=2.0))
        self.assertTrue(node.paused)
        node._cancel_current.assert_called_once()

    def test_resuming_while_still_out_pauses_again(self):
        node = bridge()
        MqttMissionBridge._on_localization_check(node, status(pause=True, distance=2.0))
        node.paused = False
        MqttMissionBridge._on_localization_check(node, status(pause=True, distance=2.0))
        self.assertTrue(node.paused)

    def test_no_mission_nothing_to_pause(self):
        node = bridge(running=False)
        MqttMissionBridge._on_localization_check(node, status(pause=True))
        self.assertFalse(node.paused)
        node._cancel_current.assert_not_called()

    def test_watch_status_never_pauses(self):
        node = bridge()
        MqttMissionBridge._on_localization_check(node, status(ok=False, pause=False))
        self.assertFalse(node.paused)

    def test_bad_message_is_ignored(self):
        node = bridge()
        MqttMissionBridge._on_localization_check(node, SimpleNamespace(data='not json'))
        self.assertIsNone(node.localization_check)
        self.assertFalse(node.paused)


if __name__ == '__main__':
    unittest.main()
