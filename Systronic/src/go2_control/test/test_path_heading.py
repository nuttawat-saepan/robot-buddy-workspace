import math

from go2_control.mqtt_mission_bridge import path_bearing, path_end_heading

# Leaves south, goes around something, and arrives heading north.
AROUND = [(0.0, -i * 0.1) for i in range(21)] \
    + [(i * 0.1, -2.0) for i in range(1, 21)] \
    + [(2.0, -2.0 + i * 0.1) for i in range(1, 16)]


def test_bearing_follows_the_route_not_the_straight_line():
    assert math.isclose(path_bearing(AROUND, 0.0, 0.0, 0.5), -math.pi / 2,
                        abs_tol=0.01)


def test_bearing_starts_from_the_nearest_point():
    bearing = path_bearing(AROUND, 1.0, -2.0, 0.5)
    assert math.isclose(bearing, 0.0, abs_tol=0.01)


def test_bearing_of_a_short_path_is_its_end():
    assert math.isclose(path_bearing([(0.0, 0.0), (0.2, 0.0)], 0.0, 0.0, 0.5),
                        0.0, abs_tol=0.01)


def test_no_bearing_without_a_path():
    assert path_bearing([], 0.0, 0.0, 0.5) is None


def test_arrival_heading_is_the_last_stretch():
    assert math.isclose(path_end_heading(AROUND, 0.5), math.pi / 2,
                        abs_tol=0.01)


def test_no_arrival_heading_on_a_path_too_short():
    assert path_end_heading([(0.0, 0.0), (0.1, 0.0)], 0.5) is None
