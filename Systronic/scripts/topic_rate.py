#!/usr/bin/env python3
"""Measure a topic's rate without printing a word of its contents.

Every other way of asking this question on Foxy gives a wrong answer here.

`ros2 topic hz` takes no QoS options and subscribes reliable, so it matches
nothing on /livox/lidar, /scan, /Odometry or /particlecloud - all of which
publish best effort - and reports silence for a topic running at 10 Hz.

`ros2 topic echo --qos-profile sensor_data` matches, but it deserializes and
prints every message. A Mid-360 cloud is tens of thousands of points, so
counting the "---" separators finds none within any sane timeout, and reports
silence for the same live topic. That was measured against a map visibly
growing in RViz.

Both failures say the same word a dead sensor says, which is precisely the
distinction a diagnostic exists to draw. This subscribes with sensor QoS and
counts arrivals, touching nothing in the message.

    ./topic_rate.py /livox/lidar 4.0
    -> /livox/lidar  9.8 Hz  (39 msgs in 4.0s)

Exit status is 0 when at least one message arrived, 1 when none did, so a
shell can branch on it.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def resolve_type(topic, node, deadline):
    """Wait for the topic to appear and return its message class."""
    while time.monotonic() < deadline:
        for name, types in node.get_topic_names_and_types():
            if name == topic and types:
                package, kind, message = types[0].split('/')
                module = __import__('%s.%s' % (package, kind),
                                    fromlist=[message])
                return getattr(module, message)
        # Spin while waiting. Sleeping instead leaves the participant's
        # discovery unserviced, so the graph stays empty and every topic in a
        # busy stack reports "absent" while `ros2 topic list` lists all 49 of
        # them - a false negative from the tool that exists to prevent them.
        rclpy.spin_once(node, timeout_sec=0.2)
    return None


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print('usage: topic_rate.py <topic> [seconds]')
        return 2
    topic = argv[0]
    window = float(argv[1]) if len(argv) > 1 else 4.0

    rclpy.init()
    node = rclpy.create_node('topic_rate_%d' % (int(time.time() * 1000) % 100000))
    try:
        deadline = time.monotonic() + 12.0
        message_type = resolve_type(topic, node, deadline)
        if message_type is None:
            print('%-24s absent' % topic)
            return 1

        count = [0]
        node.create_subscription(
            message_type, topic, lambda _m: count.__setitem__(0, count[0] + 1),
            qos_profile_sensor_data)

        end = time.monotonic() + window
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

        if count[0] == 0:
            print('%-24s silent   (0 msgs in %.1fs)' % (topic, window))
            return 1
        print('%-24s %5.1f Hz  (%d msgs in %.1fs)'
              % (topic, count[0] / window, count[0], window))
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
