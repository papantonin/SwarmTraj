"""Profiles bind one common envelope; no hidden overrides or fleet claims."""

import importlib
import json
from pathlib import Path
import runpy
import sys

import numpy as np
import pytest

import swarm_sim.trajectory2d as trajectory


ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT / 'examples/aircraft/fixed_wing_reference.json'
TRAINING = ROOT / 'examples/aircraft/fixed_wing_training.json'


@pytest.fixture
def launcher():
    return runpy.run_path(str(ROOT / 'examples/trajectory_2d.py'))


def launch(launcher, monkeypatch, tmp_path, *arguments):
    monkeypatch.setattr(sys, 'argv', ['trajectory_2d.py', '--no-gui', '--output', str(tmp_path), *map(str, arguments)])
    return launcher['main']()


@pytest.mark.parametrize('path', (REFERENCE, TRAINING))
def test_profile_parameters_reach_all_four_drone_configuration(launcher, monkeypatch, tmp_path, path):
    profile = trajectory.load_aircraft_profile(path)
    class PlanningReached(Exception):
        pass
    def inspect(snapshots, config):
        assert len(snapshots) == 4
        profile.require_matches(config)
        assert config.target == (140, 50) and config.target_altitude == 30
        assert config.descent_angle_deg == 30 and config.horizon == 90
        assert config.max_altitude == 200 and config.position_error == 2
        raise PlanningReached
    monkeypatch.setitem(launcher['main'].__globals__, 'plan_mission', inspect)
    with pytest.raises(PlanningReached):
        launch(launcher, monkeypatch, tmp_path, '--aircraft-profile', path)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('option,value', (
    ('stall-speed', 6), ('stall-margin', 1.15), ('max-load-factor', 2.5), ('max-bank', 50),
    ('max-roll-rate', 60), ('max-flight-path-angle', 45), ('max-flight-path-rate', 8),
    ('max-thrust-acceleration', 5), ('max-airbrake-acceleration', 5),
    ('parasitic-drag', .003), ('induced-drag', 20), ('max-climb-rate', 3),
    ('max-descent-rate', 10), ('max-vertical-acceleration', 1),
))
def test_physical_overrides_rejected_even_when_equal_to_profile(launcher, monkeypatch, tmp_path, option, value):
    monkeypatch.setitem(launcher['main'].__globals__, 'plan_mission', lambda *a: pytest.fail('invalid configuration must not plan'))
    with pytest.raises(SystemExit) as error:
        launch(launcher, monkeypatch, tmp_path, '--aircraft-profile', REFERENCE, f'--{option}={value}')
    assert error.value.code == 2 and not list(tmp_path.iterdir())


@pytest.mark.parametrize('arguments', (['--dimensions', '2'], ['--flight-model', 'legacy'],
    ['--aircraft-profile', str(TRAINING)], ['--drone-speed', '8', '--drone-speed', '12', '--drone-speed', '12', '--drone-speed', '12']))
def test_incompatible_modes_multiple_profiles_or_speeds_rejected(launcher, monkeypatch, tmp_path, arguments):
    monkeypatch.setitem(launcher['main'].__globals__, 'plan_mission', lambda *a: pytest.fail('invalid profile usage must not plan'))
    with pytest.raises(SystemExit) as error:
        launch(launcher, monkeypatch, tmp_path, '--aircraft-profile', TRAINING, *arguments)
    assert error.value.code == 2 and not list(tmp_path.iterdir())


def test_missing_profile_is_input_error_before_output(launcher, monkeypatch, tmp_path):
    with pytest.raises(SystemExit) as error:
        launch(launcher, monkeypatch, tmp_path, '--aircraft-profile', tmp_path / 'missing.json')
    assert error.value.code == 2 and not list(tmp_path.iterdir())


def test_mission_controls_remain_configurable_without_changing_airframe(launcher, monkeypatch, tmp_path):
    class PlanningReached(Exception):
        pass
    def inspect(snapshots, config):
        trajectory.load_aircraft_profile(TRAINING).require_matches(config)
        assert all(s.speed == 11 for s in snapshots)
        assert config.max_altitude == 170 and config.descent_angle_deg == 0
        raise PlanningReached
    monkeypatch.setitem(launcher['main'].__globals__, 'plan_mission', inspect)
    with pytest.raises(PlanningReached):
        launch(launcher, monkeypatch, tmp_path, '--aircraft-profile', TRAINING,
               '--airspeed', 11, '--max-altitude', 170, '--descent-angle', 0)


def test_failed_mission_keeps_profile_identity_and_no_individual_payload(launcher, monkeypatch, tmp_path):
    monkeypatch.setitem(launcher['main'].__globals__, 'plan_mission', lambda *a: trajectory.Result('no_candidate', 'Synthetic profile export test'))
    assert launch(launcher, monkeypatch, tmp_path, '--aircraft-profile', TRAINING) == 2
    document = json.loads((tmp_path / 'mission.json').read_text())
    report = json.loads((tmp_path / 'planning_report.json').read_text())
    assert document['aircraft_profile'] == trajectory.load_aircraft_profile(TRAINING).record()
    assert report['aircraft_profile'] == document['aircraft_profile']
    assert document['config']['flight']['max_brake_acceleration'] == 0
    assert not list(tmp_path.glob('drone_*.json'))


def test_individual_payloads_get_only_common_profile_and_own_plan(launcher, monkeypatch, tmp_path):
    plans = tuple(trajectory.Trajectory(k, 0, 12, .2, np.zeros((2, 4)), np.zeros(1), 1, 0,
                                      cruise_altitude=60) for k in range(4))
    monkeypatch.setitem(launcher['main'].__globals__, 'plan_mission', lambda *a: trajectory.Result('validated', 'Synthetic payload contract', plans))
    class PayloadsWritten(Exception):
        pass
    def stop(*args, **kwargs):
        raise PayloadsWritten
    monkeypatch.setitem(launcher['main'].__globals__, 'simulate', stop)
    with pytest.raises(PayloadsWritten):
        launch(launcher, monkeypatch, tmp_path, '--aircraft-profile', REFERENCE)
    document = json.loads((tmp_path / 'mission.json').read_text())
    for k in range(4):
        payload = json.loads((tmp_path / f'drone_{k}.json').read_text())
        assert payload['aircraft_profile'] == document['aircraft_profile']
        assert payload['mission_sha256'] == document['sha256'] and payload['plan']['drone_id'] == k
        assert set(payload) == {'schema', 'mission_sha256', 'config', 'plan', 'aircraft_profile'}


@pytest.fixture
def vertical(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / 'scripts'))
    return importlib.import_module('validate_vertical_flight')


def test_profile_runs_real_isolated_level_and_climb_fixtures(vertical, tmp_path):
    output = tmp_path / 'profile-vertical.json'
    assert vertical.main(['--aircraft-profile', str(TRAINING), '--speeds', '10', '13',
                          '--cases', 'level', 'climb_recovery', '--refine', '--output', str(output)]) == 0
    report = json.loads(output.read_text())
    assert report['all_trials_passed'] and report['aircraft_profile'] == trajectory.load_aircraft_profile(TRAINING).record()
    assert report['flight_limits']['max_brake_acceleration'] == 0


def test_missing_airbrake_not_compensated_by_changing_fixture(vertical, tmp_path):
    output = tmp_path / 'rejected-descent.json'
    assert vertical.main(['--aircraft-profile', str(TRAINING), '--speeds', '12',
                          '--cases', 'descent_recovery', '--output', str(output)]) == 2
    report = json.loads(output.read_text())
    assert report['completed'] and not report['all_trials_passed']
    assert report['trials'][0]['minimum_envelope_margins']['maximum_brake'] < 0
    assert report['fixture']['peak_slope_deg'] == pytest.approx(8)


def test_nonhistorical_airspeed_range_not_hardcoded(vertical, tmp_path):
    data = trajectory.reference_aircraft_profile().to_dict()
    data['name'] = 'laboratory_fast_variant'
    data['limits'].update(min_airspeed_m_s=20, max_airspeed_m_s=30)
    profile = tmp_path / 'profile.json'
    profile.write_text(json.dumps(data))
    output = tmp_path / 'level.json'
    assert vertical.main(['--aircraft-profile', str(profile), '--speeds', '24',
                          '--cases', 'level', '--output', str(output)]) == 0
    report = json.loads(output.read_text())
    assert report['kinematic_limits']['min_speed'] == 20 and report['trials'][0]['total_airspeed_m_s'] == 24


def test_vertical_bench_refuses_multiple_profiles(vertical, tmp_path):
    with pytest.raises(SystemExit) as error:
        vertical.main(['--aircraft-profile', str(REFERENCE), '--aircraft-profile', str(TRAINING),
                       '--output', str(tmp_path / 'not-created.json')])
    assert error.value.code == 2 and not list(tmp_path.iterdir())
