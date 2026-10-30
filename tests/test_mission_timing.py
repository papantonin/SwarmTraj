"""Explicit timing limits, independent rejection, advice and local execution."""

from dataclasses import replace

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, Snapshot, Trajectory, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.dynamics import propagate
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.mission_timing import recovery_completion
from swarm_sim.trajectory2d.planning import validate_individual
from swarm_sim.trajectory2d.vertical import build_profile


@pytest.mark.parametrize("values", [
    {"arrival_deadline": 0}, {"arrival_deadline": -1}, {"arrival_deadline": 91},
    {"arrival_deadline": np.nan}, {"arrival_deadline": np.inf},
    {"post_arrival_time": -1}, {"post_arrival_time": np.nan}, {"post_arrival_time": np.inf},
    {"obstacle_routing": "invalid"},
])
def test_invalid_timing_and_routing_settings(values):
    with pytest.raises(ValueError):
        Config(**values)


def straight_plan():
    config = Config(target=(120, 0), horizon=20, activation_time=0, max_extra_loiter=0)
    snapshot = Snapshot(0, (0, 0, 0, 0), 12)
    times = np.arange(round(config.horizon / config.dt) + 1) * config.dt
    states = propagate(snapshot.state, snapshot.speed, 0, times)
    plan = Trajectory(0, 0, 12, config.dt, states, np.zeros(len(times) - 1), 10, 0)
    return snapshot, plan, config


def test_deadline_and_post_arrival_time_are_independent_and_clock_guarded():
    snapshot, plan, config = straight_plan()
    assert validate_individual(plan, snapshot, config)
    assert validate_individual(plan, snapshot, replace(config, arrival_deadline=10.2))
    for changed in (replace(config, arrival_deadline=10), replace(config, post_arrival_time=10),
                    replace(config, arrival_deadline=9), replace(config, horizon=40, arrival_deadline=9)):
        diagnostic = {}
        assert not validate_individual(plan, snapshot, changed, diagnostic)
        assert diagnostic['rejection'] == 'arrival_timing'
    assert config.latest_nominal_arrival == pytest.approx(14.85)
    assert replace(config, arrival_deadline=11).latest_nominal_arrival == pytest.approx(10.85)


def test_independent_validator_rejects_truncated_horizon_without_obstacles():
    snapshot, plan, config = straight_plan()
    shortened = replace(plan, states=plan.states[:-1], accelerations=plan.accelerations[:-1])
    diagnostic = {}
    assert not validate_individual(shortened, snapshot, config, diagnostic)
    assert diagnostic['rejection'] == 'plan_horizon'


def test_observer_reports_arrival_deadline_even_for_unvalidated_test_plan():
    _, plan, config = straight_plan()
    report = simulate((plan,), replace(config, arrival_deadline=9)).report
    assert report['all_arrived']
    assert not report['arrival_deadline_verified_in_execution']
    assert report['post_arrival_time_verified_in_execution']
    report = simulate((plan,), replace(config, post_arrival_time=15)).report
    assert report['arrival_deadline_verified_in_execution']
    assert not report['post_arrival_time_verified_in_execution']


def test_vertical_rejections_identify_climb_and_post_passage_recovery():
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target_altitude=30, descent_angle_deg=30,
                    max_descent_rate=10, horizon=60, activation_time=0)
    snapshot = Snapshot(0, (0, 0, 0, 0), 12, altitude=80, cruise_altitude=80)
    diagnostic = {}
    assert build_profile(snapshot, config, 0, 45, np.random.default_rng(1), diagnostic) is None
    assert diagnostic['rejection_detail'] == 'recovery_exceeds_horizon'
    assert diagnostic['required_horizon_s'] > config.horizon
    diagnostic = {}
    snapshot = replace(snapshot, altitude=30, cruise_altitude=120)
    assert build_profile(snapshot, config, 0, 5, np.random.default_rng(1), diagnostic) is None
    assert diagnostic['rejection_detail'] == 'climb_cannot_finish_before_arrival'


def test_advice_keeps_configuration_and_explicit_deadline_unchanged(monkeypatch):
    import swarm_sim.trajectory2d.planning as planning
    from swarm_sim.trajectory2d.catalog import reject

    def candidate(snapshot, config, candidate_id, rng, angle=None, diagnostics=None):
        diagnostics['required_horizon_s'] = 140
        return reject(diagnostics, 'vertical_profile', 'recovery_exceeds_horizon')

    monkeypatch.setattr(planning, '_candidate', candidate)
    config = Config(attempts=2, arrival_deadline=70)
    result = plan_mission(separate_loiter_snapshot(4), config)
    assert result.status == 'no_candidate'
    timing = result.diagnostics['timing']
    assert timing['rejection_details'] == {'recovery_exceeds_horizon': 8}
    assert timing['suggested_horizon_s'] >= 140
    assert not timing['suggestion_is_feasibility_guarantee']
    assert any('deadline' in a for a in timing['advice'])
    assert config.horizon == 90 and config.arrival_deadline == 70


def test_recovery_metadata_is_after_last_unsettled_sample():
    _, plan, _ = straight_plan()
    vertical = np.tile([50., 0.], (len(plan.states), 1))
    vertical[75] = [49, 0]
    plan = replace(plan, vertical_states=vertical, cruise_altitude=50)
    assert recovery_completion(plan) == pytest.approx(15.2)
    vertical[-1] = [49, 0]
    assert recovery_completion(replace(plan, vertical_states=vertical)) is None


def test_editor_timing_fields_apply_reset_and_reject_invalid_deadline():
    import matplotlib.pyplot as plt
    from swarm_sim.trajectory2d.scenario_editor import ScenarioEditor

    editor = ScenarioEditor(separate_loiter_snapshot(4), (140, 50), Config())
    try:
        editor.horizon_input.set_val('180')
        editor.deadline_input.set_val('120')
        editor.post_arrival_input.set_val('20')
        assert editor.apply_fields()
        assert editor.config.horizon == 180 and editor.config.arrival_deadline == 120
        assert editor.config.post_arrival_time == 20
        editor.deadline_input.set_val('190')
        assert not editor.apply_fields()
        assert editor.config.arrival_deadline == 120
        editor.reset()
        assert editor.horizon_input.text == '90' and editor.deadline_input.text == 'Auto'
        assert editor.post_arrival_input.text == '5'
    finally:
        plt.close(editor.figure)
