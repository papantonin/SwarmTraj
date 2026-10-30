"""Expanded laboratory classification must retain exclusions and failed requests."""

import importlib
import json
from pathlib import Path

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, FlightLimits
from swarm_sim.trajectory2d.aircraft_profiles import load_aircraft_profile


@pytest.fixture(scope='module')
def bench():
    import sys
    folder = str(Path(__file__).resolve().parents[1] / 'scripts')
    sys.path.insert(0, folder)
    try:
        yield importlib.import_module('validate_flight_envelope')
    finally:
        sys.path.remove(folder)


@pytest.fixture
def config():
    return Config(dimensions=3, generator_mode='geometric', numerical_backend='python', max_descent_rate=10)


@pytest.mark.parametrize('name', ('fixed_wing_reference', 'fixed_wing_training', 'rascal110_empty_approx', 'rascal110_fuelled_approx'))
def test_parameter_relative_boundaries_cover_every_loaded_airframe(bench, name):
    profile = load_aircraft_profile(Path(__file__).resolve().parents[1] / 'examples/aircraft' / f'{name}.json')
    config = profile.apply(Config(dimensions=3, generator_mode='geometric', numerical_backend='python'))
    rows = bench.check_boundaries(config, parameter_relative=True)
    assert {r['target_constraint'] for r in rows} == set(bench.UNITS)
    assert all(r['contract_consistent'] and r['target_classification_correct'] for r in rows)
    for rule in bench.UNITS:
        assert {r['target_region'] for r in rows if r['target_constraint'] == rule} == {'inside', 'boundary', 'outside'}
    assert any(r['other_violations'] for r in rows)  # Coupled exclusions are not hidden.


def test_profile_cli_uses_profile_relative_grid_and_records_identity(bench, tmp_path):
    profile = Path(__file__).resolve().parents[1] / 'examples/aircraft/rascal110_empty_approx.json'
    output = tmp_path / 'envelope.json'
    assert bench.main(['--aircraft-profile', str(profile), '--output', str(output)]) == 0
    document = json.loads(output.read_text())
    assert document['completed'] and document['all_checks_passed']
    assert document['requested_speeds_m_s'] == [10, 12.5, 15, 17.5, 20]
    assert document['bank_grid_deg'] == [-30, -15, 0, 15, 30]
    assert document['aircraft_profile'] == load_aircraft_profile(profile).record()
    assert document['boundary_fixture'] == 'parameter_relative'
    assert document['counts']['accepted_trim_samples'] > 0


def test_profile_cli_does_not_hide_invalid_explicit_speed_requests(bench, tmp_path):
    profile = Path(__file__).resolve().parents[1] / 'examples/aircraft/rascal110_empty_approx.json'
    output = tmp_path / 'invalid.json'
    with pytest.raises(SystemExit):
        bench.main(['--aircraft-profile', str(profile), '--speeds', '9', '--output', str(output)])
    assert not output.exists()


def test_all_seventeen_rules_have_inside_boundary_outside_samples(bench, config):
    rows = bench.check_boundaries(config)
    assert len(rows) == 66
    assert {row['target_constraint'] for row in rows} == set(bench.UNITS)
    assert all(row['contract_consistent'] and row['target_classification_correct'] for row in rows)
    assert all(abs(row['target_margin']) <= bench.EPS for row in rows if row['target_region'] == 'boundary')
    for rule in bench.UNITS:
        assert {row['target_region'] for row in rows if row['target_constraint'] == rule} == {'inside', 'boundary', 'outside'}


def test_coupled_exclusions_not_falsely_presented_as_boundary_acceptance(bench, config):
    rows = bench.check_boundaries(config)
    blocked = {row['case'] for row in rows if row['target_region'] == 'boundary' and not row['expected_accepted']}
    assert blocked == {'minimum_load', 'structural_load'}
    for row in rows:
        if row['case'] in blocked and row['target_region'] == 'inside':
            assert row['other_violations'] and not row['expected_accepted']
    active = [row for row in rows if row['case'] not in blocked and row['target_region'] == 'inside']
    assert all(row['expected_accepted'] for row in active)
    assert not any(row['expected_accepted'] for row in rows if row['target_region'] == 'outside')


def test_oracle_is_independent_of_production_rhs_velocity_drag_and_stall_methods(bench, config, monkeypatch):
    import swarm_sim.trajectory2d.flight3d as flight
    def forbidden(*args, **kwargs):
        pytest.fail('oracle must not call production equations or helpers')
    monkeypatch.setattr(flight, 'derivative', forbidden)
    monkeypatch.setattr(flight, 'air_velocity', forbidden)
    monkeypatch.setattr(FlightLimits, 'drag', forbidden)
    monkeypatch.setattr(FlightLimits, 'load_ceiling', forbidden)
    result = bench.oracle(np.array([0, 0, 100, 12, 0, 0, 0]), np.array([.6, 0, 1, 0]), config)
    assert result['accepted'] and len(result['margins']) == 17


@pytest.mark.parametrize('speed,slope,bank,accepted,violation', (
    (12, 0, 0, True, None), (12, -10, 25, True, None),
    (8, 0, 50, False, 'loaded_stall_or_structure'),
    (12, 30, 0, False, 'climb_rate'), (12, -45, 0, False, 'maximum_brake'),
))
def test_selected_equilibrium_classifications(bench, config, speed, slope, bank, accepted, violation):
    row = bench.check_equilibrium(speed, slope, bank, config)
    assert row['contract_consistent']
    assert row['expected_accepted'] == accepted and row['integrated'] == accepted
    if accepted:
        assert row['analytic_position_error_m'] < 2e-6
    else:
        assert violation in row['violated_constraints']


@pytest.mark.parametrize('speed', (8., 12., 16.))
def test_equilibrium_subgrid_includes_both_turn_directions(bench, config, speed):
    rows = [bench.check_equilibrium(speed, slope, bank, config) for slope in bench.SLOPES for bank in bench.BANKS]
    assert len(rows) == 55 and all(row['contract_consistent'] for row in rows)
    assert any(row['expected_accepted'] for row in rows) and any(not row['expected_accepted'] for row in rows)


def test_nonfinite_and_nonpositive_samples_are_rejected(bench, config):
    rows = bench.check_guards(config)
    assert len(rows) == 6 and all(row['contract_consistent'] and not row['expected_accepted'] for row in rows)
    with pytest.raises(ValueError, match='state'):
        bench.oracle([1, 2], [0, 0, 1, 0], config)


@pytest.mark.parametrize('gate', (True, False))
def test_always_accepting_or_rejecting_production_gate_is_detected(bench, config, monkeypatch, gate):
    monkeypatch.setattr(bench, 'envelope_ok', lambda *args, **kwargs: gate)
    assert any(not row['contract_consistent'] for row in bench.check_boundaries(config))


def test_incorrect_margin_is_detected_even_with_correct_gate(bench, config, monkeypatch):
    original = bench.envelope_margins
    def incorrect(*args):
        values = original(*args)
        values['climb_rate'] += .01
        return values
    monkeypatch.setattr(bench, 'envelope_margins', incorrect)
    result = bench.compare_sample(bench._state(), np.array([.6, 0, 1, 0]), config)
    assert result['observed_accepted'] == result['expected_accepted']
    assert not result['contract_consistent']


@pytest.mark.parametrize('speed,case,accepted', ((12., 'climb_recovery', True),
    (8., 'descent_recovery', False), (16., 'climb_recovery', False), (16., 'descent_recovery', False)))
def test_dynamic_limits_preserved_and_first_violation_reported(bench, config, speed, case, accepted):
    row = bench.check_dynamic(speed, case, .2, config)
    assert row['contract_consistent'] and row['trial_accepted'] == accepted
    if not accepted:
        rule = 'minimum_airspeed' if speed == 8 else 'maximum_airspeed'
        assert rule in row['first_observed_violations_s']
        assert 0 <= row['first_observed_violations_s'][rule] <= 16
        assert row['minimum_oracle_margins'][rule] < -bench.EPS
        assert not row['vertical_report']['checks']['sampled_held_envelope_ok']


def test_no_planner_guidance_or_plan_mutation(bench, config, monkeypatch):
    import swarm_sim.trajectory2d as public
    import swarm_sim.trajectory2d.planning as planning
    import swarm_sim.trajectory2d.flight3d as physics
    def forbidden(*args, **kwargs):
        pytest.fail('the expanded bench must not invoke planning/guidance')
    monkeypatch.setattr(public, 'plan_mission', forbidden)
    monkeypatch.setattr(planning, 'plan_mission', forbidden)
    monkeypatch.setattr(physics, 'guidance', forbidden)
    first = bench.check_dynamic(12, 'climb_recovery', .2, config)
    assert first == bench.check_dynamic(12, 'climb_recovery', .2, config)


def test_incorrect_dynamic_summary_is_not_trusted(bench, config, monkeypatch):
    original = bench.vertical.run_case
    def incorrect(*args):
        report, trace = original(*args)
        report['trial_passed'] = not report['trial_passed']
        return report, trace
    monkeypatch.setattr(bench.vertical, 'run_case', incorrect)
    assert not bench.check_dynamic(12, 'level', .2, config)['contract_consistent']


@pytest.mark.parametrize('speed,case', ((8., 'climb_recovery'), (12., 'descent_recovery')))
def test_coarse_step_precision_failure_not_mislabelled_as_classifier_disagreement(bench, config, speed, case):
    row = bench.check_dynamic(speed, case, .5, config)
    assert row['classification_consistent'] and row['summary_consistent']
    assert not row['numerical_precision_passed'] and not row['contract_consistent']
    assert not row['trial_accepted']
    assert row['vertical_report']['independent_position_error_m'] > 2e-6


def test_time_step_changes_forces_and_tolerance_can_change_edge_acceptance(bench, config):
    coarse = bench.check_dynamic(16., 'descent_recovery', .2, config)
    fine = bench.check_dynamic(16., 'descent_recovery', .025, config)
    assert coarse['contract_consistent'] and fine['contract_consistent']
    assert not coarse['trial_accepted'] and fine['trial_accepted']
    assert coarse['minimum_oracle_margins']['maximum_airspeed'] < -bench.EPS
    assert fine['minimum_oracle_margins']['maximum_airspeed'] >= -bench.EPS


def test_cli_report_static_contract_success_and_no_overwrite(bench, tmp_path):
    path = tmp_path / 'static.json'
    args = ['--speeds', '12', '--output', str(path)]
    assert bench.main(args) == 0
    document = json.loads(path.read_text())
    assert document['completed'] and document['all_checks_passed'] and document['all_contract_checks_passed']
    assert document['all_requested_dynamic_trials_accepted'] is None
    assert document['counts']['trim_samples'] == 55
    assert document['counts']['rejected_trim_samples'] > 0
    assert document['counts']['dynamic_trials'] == 0
    assert document['source_sha256']
    assert document['numerical_tolerances']['integration_position_m'] == 2e-6
    assert document['equilibrium_fixture']['duration_s'] == 2
    assert document['dynamic_fixture']['peak_slope_deg'] == pytest.approx(8)
    before = path.read_bytes()
    with pytest.raises(SystemExit) as error:
        bench.main(args)
    assert error.value.code == 2 and path.read_bytes() == before


def test_failed_dynamic_request_retained_not_relabelled_as_expected_negative(bench, tmp_path):
    path = tmp_path / 'dynamic.json'
    assert bench.main(['--speeds', '8', '--time-steps', '.2', '--dynamic', '--output', str(path)]) == 2
    document = json.loads(path.read_text())
    assert document['completed'] and document['all_contract_checks_passed']
    assert document['all_checks_passed'] is False and document['all_requested_dynamic_trials_accepted'] is False
    assert document['counts']['dynamic_trials'] == 3 and document['counts']['rejected_dynamic_trials'] == 1
    assert all('expected_rejection' not in row for row in document['dynamic_trials'])
    assert all('flight_controls' not in row for row in document['dynamic_trials'])


@pytest.mark.parametrize('arguments', (['--speeds', 'nan'], ['--speeds', '7'], ['--speeds', '17'],
    ['--speeds', '12', '12'], ['--time-steps', '.2', '.2'], ['--time-steps', '.3'],
    ['--time-steps', 'inf'], ['--time-steps', '0']))
def test_cli_invalid_arguments_rejected_before_output(bench, tmp_path, arguments):
    path = tmp_path / 'invalid.json'
    with pytest.raises(SystemExit) as error:
        bench.main([*arguments, '--output', str(path)])
    assert error.value.code == 2 and not path.exists()


def test_numerical_failure_is_recorded_and_propagated(bench, tmp_path, monkeypatch):
    path = tmp_path / 'error.json'
    def failure(*args):
        raise RuntimeError('synthetic integration failure')
    monkeypatch.setattr(bench, 'check_equilibrium', failure)
    with pytest.raises(RuntimeError, match='synthetic integration failure'):
        bench.main(['--speeds', '12', '--output', str(path)])
    document = json.loads(path.read_text())
    assert not document['completed'] and not document['all_checks_passed']
    assert document['error']['type'] == 'RuntimeError'


def test_source_change_never_claims_completion(bench, tmp_path, monkeypatch):
    path = tmp_path / 'changed.json'
    fingerprints = iter(({'source': 'before'}, {'source': 'after'}))
    monkeypatch.setattr(bench, 'source_fingerprint', lambda: next(fingerprints))
    with pytest.raises(RuntimeError, match='source changed'):
        bench.main(['--speeds', '12', '--output', str(path)])
    document = json.loads(path.read_text())
    assert not document['completed'] and not document['all_checks_passed']


def test_keyboard_interrupt_retains_incomplete_evidence(bench, tmp_path, monkeypatch):
    path = tmp_path / 'interrupted.json'
    def interrupt(*args):
        raise KeyboardInterrupt
    monkeypatch.setattr(bench, 'check_equilibrium', interrupt)
    with pytest.raises(KeyboardInterrupt):
        bench.main(['--speeds', '12', '--output', str(path)])
    document = json.loads(path.read_text())
    assert not document['completed'] and not document['all_checks_passed']
