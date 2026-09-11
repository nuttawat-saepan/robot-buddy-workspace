#!/usr/bin/env python3
"""Apply parameters that a running node only reads at configure time.

Some of the values worth changing on site are read once and kept. Foxy's
nav2_amcl reads its alphas, particle counts and update thresholds in
on_configure; InflationLayer reads inflation_radius in onInitialize; the
goal checker reads its tolerances when the controller configures. None of
them carry a parameter callback, so `ros2 param set` succeeds, `ros2 param
get` returns the new number, and the robot goes on using the old one.

Relaunching the stack picks them up and costs about five minutes with a
fresh initial pose at the end of it. This does the same thing by taking the
one node through its lifecycle instead:

    set the parameter  ->  deactivate  ->  cleanup  ->  configure  ->  activate

Measured against a replayed stack on 2026-09-11 the four transitions took
18 ms. The two seconds this then waits before putting the pose back is the
whole cost, and lifecycle_manager did not react to any of it - no bond
break, no restart of the managed set.

The set has to come first. on_configure reads through get_parameter, not by
re-reading the yaml, so the value set on the node is what the reconfigure
picks up - which is also why this works at all without touching a file.

## What it costs

AMCL forgets where the robot is. The particle filter is destroyed by
cleanup and comes back seeded at the launch default, so the pose has to be
put back. This captures /amcl_pose before the cycle and republishes it to
/initialpose afterwards, which is the same thing an operator does from RViz
but without the aiming error.

For the length of the cycle there is no map->odom transform at all. Nav2
loses the map frame, the controller fails, and the behaviour tree will start
reaching for recoveries. **Only do this while the robot is stopped.** The
node refuses if it can see a mission running.

## The risk that has to be tested, not assumed

nav2's lifecycle_manager holds a bond with each node it manages and treats a
bond going away as a crash. Whether it tolerates a node cycling underneath
it, or tears the whole managed set down and brings it back, is a property of
the version in use rather than something to reason about. Run --dry-run
against a replay once before trusting it on site.

    ros2 run go2_control reconfigure --node /amcl \\
        --set alpha1=0.03 --set alpha2=0.10
    ros2 run go2_control reconfigure --node /local_costmap/local_costmap \\
        --set inflation_layer.inflation_radius=0.55
    ros2 run go2_control reconfigure --node /amcl --cycle-only
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

# The four transitions, in the order that takes an active node back to
# active having re-read its parameters. cleanup is the one that matters:
# configure is only called again because cleanup put the node back into the
# unconfigured state.
CYCLE = [
    ('deactivate', Transition.TRANSITION_DEACTIVATE),
    ('cleanup', Transition.TRANSITION_CLEANUP),
    ('configure', Transition.TRANSITION_CONFIGURE),
    ('activate', Transition.TRANSITION_ACTIVATE),
]


class Reconfigure(Node):

    def __init__(self, target):
        super().__init__('reconfigure')
        self.target = target
        self.change = self.create_client(
            ChangeState, target + '/change_state')
        self.state = self.create_client(GetState, target + '/get_state')
        self.pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10)
        self.initial = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

    def _on_pose(self, msg):
        self.pose = msg

    def _call(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('%s is not there' % client.srv_name)
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def current_state(self):
        res = self._call(self.state, GetState.Request())
        return res.current_state.label if res else 'unknown'

    def set_params(self, pairs):
        """Set doubles on the target node through its parameter service."""
        from rcl_interfaces.msg import (Parameter, ParameterType,
                                        ParameterValue)
        from rcl_interfaces.srv import SetParameters
        cli = self.create_client(SetParameters, self.target + '/set_parameters')
        params = []
        for name, raw in pairs:
            p = Parameter()
            p.name = name
            p.value = ParameterValue()
            p.value.type = ParameterType.PARAMETER_DOUBLE
            p.value.double_value = float(raw)
            params.append(p)
        req = SetParameters.Request()
        req.parameters = params
        res = self._call(cli, req)
        if res is None:
            return False
        ok = True
        for p, r in zip(params, res.results):
            if not r.successful:
                self.get_logger().error(
                    '%s refused: %s' % (p.name, r.reason or 'no reason'))
                ok = False
            else:
                self.get_logger().info(
                    'set %s = %.4f (stored, not yet in use)'
                    % (p.name, p.value.double_value))
        return ok

    def cycle(self, restore_pose=True):
        held = self.pose
        if restore_pose and held is None:
            self.get_logger().warn(
                'no /amcl_pose seen - nothing to put back afterwards, the '
                'robot will come back at the launch default pose')

        for label, transition in CYCLE:
            res = self._call(
                self.change, ChangeState.Request(
                    transition=Transition(id=transition)))
            if res is None or not res.success:
                self.get_logger().error(
                    '%s failed - the node is now in state %r and is not '
                    'coming back on its own' % (label, self.current_state()))
                return False
            self.get_logger().info('%s ok' % label)

        if restore_pose and held is not None:
            # Two seconds for the node to finish activating and subscribe.
            # Sent to /initialpose rather than set on the node because that
            # is the path AMCL actually acts on.
            time.sleep(2.0)
            # The stamp is left exactly as captured, having tried the
            # alternatives. AMCL logs "Failed to transform initial pose in
            # time" either way - /amcl_pose is stamped with the scan and
            # runs about 20 ms ahead of the newest odom->base_link, and a
            # zero stamp did not avoid it because AMCL substitutes a time of
            # its own. The warning is cosmetic: the line after it is AMCL
            # setting the pose to exactly the value handed over, measured
            # across three runs.
            #
            # This republish is insurance rather than the mechanism. AMCL is
            # configured with save_pose_rate 0.5, so it writes its own pose
            # into initial_pose.* twice a second and seeds from there when it
            # configures - measured here it came back at -1.767 -3.305 on its
            # own, the same place this then sets it to. What this adds is the
            # pose as of the moment the cycle started rather than up to two
            # seconds before it. If save_pose_rate is ever set to zero, this
            # stops being insurance and becomes the only thing putting the
            # robot back.
            self.initial.publish(held)
            self.get_logger().info(
                'pose put back at x=%.3f y=%.3f'
                % (held.pose.pose.position.x, held.pose.pose.position.y))
        return True


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--node', required=True,
                        help='Fully qualified node name, e.g. /amcl')
    parser.add_argument('--set', action='append', default=[],
                        metavar='NAME=VALUE',
                        help='A double parameter to set before cycling. '
                             'Repeatable.')
    parser.add_argument('--cycle-only', action='store_true',
                        help='Cycle without setting anything, to find out '
                             'whether lifecycle_manager tolerates it.')
    parser.add_argument('--no-restore-pose', action='store_true',
                        help='Do not put /amcl_pose back afterwards. Only '
                             'sensible for a node that is not AMCL.')
    parser.add_argument('--dry-run', action='store_true',
                        help='Print what would happen and stop.')
    args, ros_args = parser.parse_known_args(
        argv if argv is not None else sys.argv[1:])

    pairs = []
    for item in args.set:
        if '=' not in item:
            parser.error('--set wants NAME=VALUE, got %r' % item)
        name, _, value = item.partition('=')
        pairs.append((name, value))

    if not pairs and not args.cycle_only:
        parser.error('nothing to do: pass --set or --cycle-only')

    if args.dry_run:
        print('would set on %s:' % args.node)
        for name, value in pairs:
            print('   %s = %s' % (name, value))
        print('then: ' + ' -> '.join(label for label, _ in CYCLE))
        print('then republish the held /amcl_pose to /initialpose'
              if not args.no_restore_pose else 'then nothing')
        return

    rclpy.init(args=ros_args)
    node = Reconfigure(args.node)
    try:
        # Let a pose arrive before anything is torn down.
        deadline = time.time() + 3.0
        while time.time() < deadline and node.pose is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        before = node.current_state()
        node.get_logger().info('%s is %s' % (args.node, before))
        if before != 'active':
            node.get_logger().error(
                'expected active - refusing to cycle a node that is not '
                'running normally')
            return

        if pairs and not node.set_params(pairs):
            node.get_logger().error('parameters refused - not cycling')
            return

        ok = node.cycle(restore_pose=not args.no_restore_pose)
        node.get_logger().info(
            'finished, %s is %s' % (args.node, node.current_state()))
        if not ok:
            sys.exit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
