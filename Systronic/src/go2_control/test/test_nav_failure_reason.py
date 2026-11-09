from go2_control.mqtt_mission_bridge import nav_failure_reason


def test_dropped_goal_is_not_called_unreachable():
    assert nav_failure_reason('WP3', 0.28, True) == 'Nav2 dropped the goal for WP3'


def test_real_failure_names_the_distance_left():
    assert nav_failure_reason('WP3', 0.28, False) == 'Nav2 gave up on WP3 with 0.28 m left'
