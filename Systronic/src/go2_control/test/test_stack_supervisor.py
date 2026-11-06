"""What the settings API may change, and what a restart is given."""
import pytest

from go2_control import stack_supervisor as ss

BASE = {k: v for k, v in {
    'mission_max_linear': 0.2, 'mission_max_angular': 0.3,
    'bridge_max_linear': 0.2, 'bridge_max_angular': 0.35,
    'scan_min_height': 0.25, 'scan_max_height': 1.05,
    'amcl_scan_min_height': 0.6, 'amcl_scan_max_height': 1.1,
    'alpha1': 0.03, 'alpha2': 0.1, 'alpha3': 0.2, 'alpha4': 0.08,
    'planner_tolerance': 0.5, 'xy_goal_tolerance': 0.35,
    'yaw_goal_tolerance': 0.3, 'localization_spread_limit': 0.5}.items()}


def status_of(changes, allow=True):
    with pytest.raises(ss.ApplyError) as caught:
        ss.validate_changes(changes, BASE, allow)
    return caught.value.status


@pytest.mark.parametrize('key', ['UNITREE_AUTO_ARM', 'ROBOT_ACK', 'ROBOT_IP',
                                 'MQTT_BROKER', 'GO2_ENFORCE_SPREAD_LIMIT'])
def test_motion_gates_and_addresses_are_not_settings(key):
    assert status_of({key: 1}) == 422


@pytest.mark.parametrize('value', [float('nan'), float('inf'), None, '0.1', True])
def test_bad_values_are_refused(value):
    assert status_of({'alpha3': value}) == 422


def test_out_of_range_and_band_order():
    assert status_of({'alpha3': 9}) == 422
    assert status_of({'scan_min_height': 1.2}) == 422
    assert status_of({'amcl_scan_min_height': 1.2}) == 422


def test_mission_speed_cannot_exceed_the_bridge():
    assert status_of({'mission_max_linear': 0.3}) == 422
    assert status_of({'bridge_max_linear': 0.1}) == 422


def test_restart_field_needs_permission():
    assert status_of({'alpha4': 0.04}, allow=False) == 409
    merged, keys = ss.validate_changes({'mission_max_linear': 0.1}, BASE, False)
    assert merged['mission_max_linear'] == 0.1 and keys == ['mission_max_linear']


def test_only_named_keys_change():
    merged, _ = ss.validate_changes({'alpha4': 0.04}, BASE, True)
    assert {k for k in BASE if merged[k] != BASE[k]} == {'alpha4'}


def test_revision_follows_config_only():
    assert ss.revision_of(BASE) == ss.revision_of(dict(BASE))
    assert ss.revision_of(BASE) != ss.revision_of(dict(BASE, alpha4=0.04))


def test_generated_config_leaves_the_base_alone():
    base = {'amcl': {'ros__parameters': {'alpha3': 0.4, 'keep': 1}}}
    out = ss.amcl_config(base, dict(BASE, alpha3=0.1))
    assert out['amcl']['ros__parameters']['alpha3'] == 0.1
    assert out['amcl']['ros__parameters']['keep'] == 1
    assert base['amcl']['ros__parameters']['alpha3'] == 0.4


def test_launch_gets_the_bands():
    args = ss.launch_args(BASE, '/m.yaml', 'A', 'N', ['replay:=false'])
    assert 'amcl_scan_min_height:=0.6' in args and 'scan_max_height:=1.05' in args
    assert 'map:=/m.yaml' in args and args[-1] == 'replay:=false'


def test_bridge_caps_come_from_the_running_command():
    line = 'x unitree_udp_bridge --max-linear 0.2 --max-angular 0.35 --mode api'
    assert ss.bridge_caps(line) == (0.2, 0.35)
    assert ss.bridge_caps('x unitree_udp_bridge') is None


def test_stillness_uses_measured_values():
    assert ss.is_still([(0.03, 1.02)] * 60)
    assert not ss.is_still([(0.69, 1.8)] * 60)
    assert not ss.is_still([])


GOOD = {'now': 100.0, 'stack_up': True, 'stationary': True, 'lio_speed': 0.01,
        'pose_set_at': 90.0, 'position_at': 99.0, 'mission_active': False,
        'other_lidar_user': False, 'spread_limit': 0.5,
        'position': {'localizationSpread': 0.3,
                     'localizationCheck': {'ok': True, 'distance': 0.1,
                                           'limit': 0.5}}}


def test_preflight_passes_when_everything_is_good():
    assert ss.preflight_problems(GOOD) == []


@pytest.mark.parametrize('change', [
    {'stack_up': False}, {'stationary': False}, {'lio_speed': None},
    {'lio_speed': 1.5}, {'pose_set_at': 0.0}, {'position_at': 10.0},
    {'mission_active': True}, {'other_lidar_user': True},
    {'position': {'localizationSpread': 0.9}},
    {'position': {'localizationCheck': {'ok': False, 'distance': 0.7,
                                        'limit': 0.5}}}])
def test_preflight_stops_on_each_problem(change):
    assert ss.preflight_problems(dict(GOOD, **change))


@pytest.mark.parametrize('change,expected', [
    ({}, 'ready'),
    ({'pose_set_at': 0.0}, 'not_set'),
    ({'position_at': 10.0}, 'failed'),
    ({'position': {}}, 'searching'),
    ({'position': {'localizationSpread': 0.9}}, 'searching'),
    ({'position': {'localizationSpread': 0.1,
                   'localizationCheck': {'ok': False, 'distance': 0.7}}},
     'failed')])
def test_localization_state_guides_the_operator(change, expected):
    assert ss.localization_state(dict(GOOD, **change))['state'] == expected
