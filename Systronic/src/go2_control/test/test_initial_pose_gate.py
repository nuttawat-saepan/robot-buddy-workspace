"""The web pose is refused only while a mission is actually running."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_control.mqtt_mission_bridge import MqttMissionBridge


def bridge(worker_alive, stopping=False):
    worker = None if worker_alive is None else mock.Mock(
        is_alive=lambda: worker_alive)
    return SimpleNamespace(
        worker=worker, mission={'runId': 'r'}, stopping=stopping, frame='map',
        spread=0.5, state_topic='/missions/state',
        initialpose_pub=mock.Mock(), get_logger=lambda: mock.Mock(),
        _publish=mock.Mock())


class InitialPoseGateTest(unittest.TestCase):

    def send(self, node):
        MqttMissionBridge._set_initial_pose(node, {'x': 1, 'y': 2, 'yaw': 0})
        return node._publish.call_args[0][1]['initialPose']['accepted']

    def test_finished_mission_does_not_block(self):
        node = bridge(worker_alive=False)
        self.assertTrue(self.send(node))
        node.initialpose_pub.publish.assert_called_once()

    def test_no_worker_does_not_block(self):
        self.assertTrue(self.send(bridge(worker_alive=None)))

    def test_running_mission_blocks(self):
        node = bridge(worker_alive=True)
        self.assertFalse(self.send(node))
        node.initialpose_pub.publish.assert_not_called()

    def test_stopping_mission_does_not_block(self):
        self.assertTrue(self.send(bridge(worker_alive=True, stopping=True)))


if __name__ == '__main__':
    unittest.main()
