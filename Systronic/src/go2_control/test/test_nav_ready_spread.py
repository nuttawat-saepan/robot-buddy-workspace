"""Verify that operator-selected advisory spread preserves other checks."""

import sys
from pathlib import Path
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_control.nav_ready_check import NavReadyCheck


class SpreadPolicyTest(unittest.TestCase):
    def check(self, enforce=True, missing_tf=False, missing_scan=False):
        node = SimpleNamespace(
            spread=0.8, particles=500, enforce_spread_limit=enforce,
            map_frame='map', odom_frame='odom', base_frame='base_link',
            occupied=10, costmap_cells=100, scan_topic='/scan',
            cmd_topic='/cmd_vel_nav_preview',
            has_tf=lambda *_: (not missing_tf, 'missing transform'),
            count_publishers=lambda topic: (
                0 if missing_scan and topic == '/scan' else 1))
        return NavReadyCheck.readiness(node)

    def test_enforced_spread_fails(self):
        ok, lines = self.check()
        self.assertFalse(ok)
        self.assertEqual(lines[0][1], 'FAIL')

    def test_advisory_spread_warns(self):
        ok, lines = self.check(enforce=False)
        self.assertTrue(ok)
        self.assertEqual(lines[0][1], 'WARN')

    def test_advisory_does_not_bypass_missing_tf(self):
        self.assertFalse(self.check(enforce=False, missing_tf=True)[0])

    def test_advisory_does_not_bypass_missing_scan(self):
        self.assertFalse(self.check(enforce=False, missing_scan=True)[0])


if __name__ == '__main__':
    unittest.main()
