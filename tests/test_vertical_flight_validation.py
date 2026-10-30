"""Laboratory-only vertical verification; no missions, targets or guidance changes."""

from dataclasses import replace
import importlib
import json
from pathlib import Path

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config
from swarm_sim.trajectory2d.flight3d import air_velocity


@pytest.fixture(scope='module')
def bench():
    import sys
    folder = str(Path(__file__).resolve().parents[1] / 'scripts')
    sys.path.insert(0, folder)
    try:
        yield importlib.import_module('validate_vertical_flight')
    finally:
        sys.path.remove(folder)


@pytest.mark.parametrize('case', ('level', 'climb_recovery', 'descent_recovery'))
@pytest.mark.parametrize('speed', (10., 11., 12., 13.))
def test_isolated_fixtures_pass_unchanged_physics(bench, case, speed):
    report, trace = bench.run_case(speed, case)
    assert report['trial_passed'], report
    assert report['independent_position_error_m'] < 2e-6
    assert report['energy_rate_residual_w_kg'] < 1e-10
    assert report['level_section_maximum_vertical_speed_m_s'] < .1
    assert trace['time_s'][-1] == 16
    assert not any(array.flags.writeable for array in trace.values())
    velocity = air_velocity(trace['actual'])
    np.testing.assert_allclose(np.linalg.norm(velocity, axis=1), trace['actual'][:, 3], atol=1e-12)
    if case == 'level':
        np.testing.assert_allclose(trace['actual'][:, 2], 100, atol=1e-10)
    elif case == 'climb_recovery':
        assert trace['actual'][-1, 2] > 100 and velocity[:, 2].max() > 1
    else:
        assert trace['actual'][-1, 2] < 100 and velocity[:, 2].min() < -1


@pytest.mark.parametrize('case', ('climb_recovery', 'descent_recovery'))
def test_halving_command_interval_reduces_reference_error_not_an_rk4_order_claim(bench, case):
    reports = [bench.run_case(12., case, dt)[0] for dt in (.2, .1, .05)]
    errors = [r['maximum_reference_position_error_m'] for r in reports]
    assert all(r['trial_passed'] for r in reports)
    assert errors[2] < errors[1] < errors[0]
    # Identical held-control integration is independently checked in every run.
    assert max(r['independent_position_error_m'] for r in reports) < 2e-6


def test_repeat_is_exactly_reproducible_and_never_uses_planner(bench, monkeypatch):
    import swarm_sim.trajectory2d as public
    import swarm_sim.trajectory2d.planning as planning
    import swarm_sim.trajectory2d.flight3d as physics
    def forbidden(*args, **kwargs):
        pytest.fail('laboratory fixtures must not call planning or guidance')
    monkeypatch.setattr(public, 'plan_mission', forbidden)
    monkeypatch.setattr(planning, 'plan_mission', forbidden)
    monkeypatch.setattr(physics, 'guidance', forbidden, raising=False)
    first, trace = bench.run_case(13., 'descent_recovery')
    second, again = bench.run_case(13., 'descent_recovery')
    assert first == second
    for key in trace:
        np.testing.assert_array_equal(trace[key], again[key])


def test_tight_physical_limit_is_reported_without_clipping_forces(bench):
    config = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', max_descent_rate=10)
    report, _ = bench.run_case(12., 'climb_recovery', config=replace(config, max_climb_rate=.1))
    assert report['checks']['physics_consistent']
    assert not report['checks']['sampled_reference_envelope_ok']
    assert not report['checks']['sampled_held_envelope_ok']
    assert not report['trial_passed']
    assert report['minimum_envelope_margins']['climb_rate'] < 0
    report, _ = bench.run_case(12., 'descent_recovery', config=replace(config,
                                flight=replace(config.flight, max_brake_acceleration=0)))
    assert not report['trial_passed'] and report['minimum_envelope_margins']['maximum_brake'] < 0


def test_invalid_samples_rejected_and_disabled_checker_cannot_pass(bench, monkeypatch):
    config = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', max_descent_rate=10)
    checks = bench.negative_envelope_checks(config)
    assert len(checks) == 12 and all(row['check_passed'] for row in checks)
    assert all(not row['sampled_envelope_ok'] for row in checks)
    monkeypatch.setattr(bench, 'envelope_ok', lambda *args: True)
    assert not any(row['check_passed'] for row in bench.negative_envelope_checks(config))


@pytest.mark.parametrize('speed,case,dt', ((np.nan, 'level', .2), (0, 'level', .2),
    (12, 'unknown', .2), (12, 'level', np.nan), (12, 'level', 0), (12, 'level', .3)))
def test_invalid_fixture_parameters_rejected(bench, speed, case, dt):
    with pytest.raises(ValueError):
        bench.run_case(speed, case, dt)


def test_legacy_mode_cannot_be_confused_with_physics_validation(bench):
    with pytest.raises(ValueError, match='coupled 3D'):
        bench.run_case(12, 'level', config=Config(dimensions=2))


def test_different_independent_dynamics_exposes_model_disagreement(bench, monkeypatch):
    original = bench.independent_held_rhs
    def wrong(t, state, control, flight):
        rates = original(t, state, control, flight)
        rates[1] += .01
        return rates
    monkeypatch.setattr(bench, 'independent_held_rhs', wrong)
    report, _ = bench.run_case(12, 'level')
    assert not report['trial_passed'] and not report['checks']['physics_consistent']
    assert report['independent_position_error_m'] > .1


def test_cli_real_report_refinement_and_exclusive_output(bench, tmp_path):
    path = tmp_path / 'vertical.json'
    arguments = ['--speeds', '12', '--cases', 'climb_recovery', '--refine', '--output', str(path)]
    assert bench.main(arguments) == 0
    report = json.loads(path.read_text())
    assert report['schema'] == 'swarm-vertical-physics-validation-v1'
    assert report['all_trials_passed'] and len(report['trials']) == 1
    assert report['completed']
    trial = report['trials'][0]
    assert trial['halved_command_interval']['dt_s'] == .1
    assert trial['command_refinement_not_worse'] and trial['command_refinement_error_ratio'] < 1
    assert len(report['negative_checks']) == 12
    assert report['source_sha256']
    # Summaries are not deployable control sequences or mission plans.
    assert 'flight_controls' not in path.read_text() and 'target' not in report
    before = path.read_bytes()
    with pytest.raises(SystemExit) as error:
        bench.main(arguments)
    assert error.value.code == 2 and path.read_bytes() == before


@pytest.mark.parametrize('arguments', (['--speeds', 'nan'], ['--speeds', '7'], ['--speeds', '17'],
    ['--speeds', '12', '12'], ['--cases', 'level', 'level'], ['--dt', 'nan'], ['--dt', '.3']))
def test_cli_invalid_arguments_do_not_create_output(bench, tmp_path, arguments):
    path = tmp_path / 'invalid.json'
    with pytest.raises(SystemExit) as error:
        bench.main([*arguments, '--output', str(path)])
    assert error.value.code == 2 and not path.exists()


def test_error_preserved_without_success_claim(bench, tmp_path, monkeypatch):
    path = tmp_path / 'error.json'
    def broken(*args):
        raise RuntimeError('synthetic integration error')
    monkeypatch.setattr(bench, 'run_case', broken)
    with pytest.raises(RuntimeError, match='synthetic integration error'):
        bench.main(['--speeds', '12', '--cases', 'level', '--output', str(path)])
    report = json.loads(path.read_text())
    assert report['all_trials_passed'] is False and report['error']['type'] == 'RuntimeError'
    assert not report['completed']
    assert report['trials'] == []


def test_source_change_invalidates_result(bench, tmp_path, monkeypatch):
    path = tmp_path / 'changed.json'
    versions = iter(({'source': 'before'}, {'source': 'after'}))
    monkeypatch.setattr(bench, 'source_fingerprint', lambda: next(versions))
    with pytest.raises(RuntimeError, match='source changed'):
        bench.main(['--speeds', '12', '--cases', 'level', '--output', str(path)])
    report = json.loads(path.read_text())
    assert report['all_trials_passed'] is False and report['trials'] == []
    assert not report['completed']


def test_interruption_writes_incomplete_not_successful_evidence(bench, tmp_path, monkeypatch):
    path = tmp_path / 'interrupted.json'
    def interrupt(*args):
        raise KeyboardInterrupt
    monkeypatch.setattr(bench, 'run_case', interrupt)
    with pytest.raises(KeyboardInterrupt):
        bench.main(['--speeds', '12', '--cases', 'level', '--output', str(path)])
    report = json.loads(path.read_text())
    assert report['all_trials_passed'] is False and report['completed'] is False
