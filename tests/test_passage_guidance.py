"""Intermediate windows guide new physical rollouts, never selected-plan retiming."""

from dataclasses import replace
import pickle

import numpy as np
import pytest
from scipy.integrate import cumulative_trapezoid

from swarm_sim.trajectory2d import Config, Trajectory, separate_loiter_snapshot
from swarm_sim.trajectory2d.catalog import Catalog, extend_catalog
from swarm_sim.trajectory2d.coupled_planning import convert_candidate
from swarm_sim.trajectory2d.dynamics import Snapshot
from swarm_sim.trajectory2d.passage_guidance import (
    PassageWindows, passage_windows, speed_schedule_for_window, verify_passage)
from swarm_sim.trajectory2d.planning import validate_individual
from swarm_sim.trajectory2d.reservations import Occupancy


def config(**kw):
    values = dict(dimensions=3, generator_mode='geometric', numerical_backend='python', target=(1320, 0), target_altitude=60, horizon=140,
                  activation_time=0, max_extra_loiter=0, cruise_altitude_range=(60, 60),
                  speed_profile='smooth', speed_profile_range=(10, 14),
                  reservation_policy='grid', passage_guidance='safe_windows')
    return Config(**(values | kw))


def straight():
    times = np.arange(701) * .2
    states = np.zeros((701, 4))
    states[:, 0] = 12 * times
    vertical = np.zeros((701, 2))
    vertical[:, 0] = 60
    return Trajectory(0, 0, 12, .2, states, np.zeros(700), 110, 0,
                      vertical_states=vertical, vertical_accelerations=np.zeros(700), cruise_altitude=60)


def windows():
    return PassageWindows(0, (1, 2, 3), (43, 0, 3), (864., 0., 60.), 72., 4.,
                          ((0., 70.999999), (73.000001, 140.)), 1)


@pytest.mark.parametrize('kw', [dict(passage_guidance='other'), dict(reservation_policy='none'),
                              dict(speed_profile='constant'), dict(flight_model='legacy')])
def test_invalid_passage_configuration(kw):
    with pytest.raises(ValueError):
        config(**kw)


def test_conflicting_cell_free_windows_exclude_closed_boundaries():
    own = Occupancy(0, 0, {(43, 0, 3): ((70., 74.),)}, 10)
    other = Occupancy(1, 0, {(43, 0, 3): ((71., 73.),)}, 10)
    hint = passage_windows(straight(), own, [other], config())
    assert hint.cell == (43, 0, 3) and hint.nominal_time_s == 72
    assert hint.anchor == (864., 0., 60.)
    assert hint.intervals_s[0][1] < 71 and hint.intervals_s[1][0] > 73
    assert hint.reference_drone_ids == (1,) and not hint.budget_exhausted
    assert pickle.loads(pickle.dumps(hint)) == hint


def test_no_conflict_or_terminal_only_conflict_does_not_create_a_hint():
    cell = (43, 0, 3)
    own = Occupancy(0, 0, {cell: ((70., 74.),)}, 10)
    free = Occupancy(1, 0, {cell: ((75., 76.),)}, 10)
    assert passage_windows(straight(), own, [free], config()) is None
    terminal = Occupancy(0, 0, {cell: ((100., 104.),)}, 10)
    assert passage_windows(straight(), terminal, [terminal], config()) is None


def test_hint_budget_exhaustion_is_not_a_mission_infeasibility_claim():
    cell = (43, 0, 3)
    own = Occupancy(0, 0, {cell: ((70., 74.),)}, 10)
    other = Occupancy(1, 0, {cell: ((71., 73.), (80., 82.))}, 10)
    hint = passage_windows(straight(), own, [other], config(interval_max_constraints=1))
    assert hint.budget_exhausted and hint.cell is None and hint.intervals_s == ()


def test_local_bump_fits_reference_window_and_respects_bounds_and_joins():
    fine = np.linspace(0, 140, 2801)
    diagnostic = {}
    speeds, derivative = speed_schedule_for_window(straight(), config(), fine,
                                                   np.full_like(fine, 12), windows(), diagnostic)
    fitted = diagnostic['passage_fit']
    a, b = fitted['selected_window_s']
    assert a < fitted['reference_passage_s'] < b
    assert not 71 <= fitted['reference_passage_s'] <= 73
    assert speeds.min() >= 10 and speeds.max() <= 14 and np.ptp(speeds) >= .2
    assert np.max(np.abs(derivative * speeds / 12)) <= config().speed_profile_acceleration + 1e-10
    start, end = fitted['bump_interval_s']
    assert np.all(speeds[fine <= start] == 12) and np.all(speeds[fine >= end] == 12)
    assert np.all(derivative[(fine <= start) | (fine >= end)] == 0)
    clock = cumulative_trapezoid(12 / speeds, fine, initial=0)
    assert np.interp(fitted['geometric_passage_s'], fine, clock) == pytest.approx(fitted['reference_passage_s'])


def test_unreachable_or_too_early_window_is_rejected_not_teleported():
    fine = np.linspace(0, 140, 2801)
    far = replace(windows(), intervals_s=((120., 140.),))
    assert speed_schedule_for_window(straight(), config(), fine, np.full_like(fine, 12), far) is None
    early = replace(windows(), anchor=(12., 0., 60.))
    assert speed_schedule_for_window(straight(), config(), fine, np.full_like(fine, 12), early) is None


def test_fitted_rollout_is_physically_valid_with_unchanged_departure_and_real_cell_window():
    snapshot = Snapshot(0, (0., 0., 0., 0.), 12, 60, 60)
    diagnostic = {}
    plan = convert_candidate(straight(), snapshot, config(), diagnostic, passage=windows())
    assert plan is not None, diagnostic
    assert validate_individual(plan, snapshot, config()), diagnostic
    assert plan.release_time == 0
    assert plan is not straight() and plan.flight_states is not None
    valid, visits = verify_passage(plan, windows(), diagnostic['passage_fit']['selected_window_s'], config())
    assert valid and visits
    assert not verify_passage(plan, windows(), (120., 140.), config())[0]


def test_failed_guided_attempt_is_not_repeated_and_keeps_budget(monkeypatch):
    import swarm_sim.trajectory2d.planning as planning
    import swarm_sim.trajectory2d.coupled_planning as coupled
    calls = []
    def ordinary(*args, **kw):
        calls.append('ordinary')
        return None
    def guided(*args, **kw):
        calls.append('guided')
        assert args[0].release_time == 0 and kw['passage'] == windows()
        return None
    monkeypatch.setattr(planning, '_candidate', ordinary)
    monkeypatch.setattr(coupled, 'convert_candidate', guided)
    catalog = Catalog(geometric_references={0: straight()})
    result = extend_catalog((separate_loiter_snapshot()[0], config(attempts=4), None, catalog,
                              2, None, None, 'conflict', windows()))
    assert calls == ['guided', 'ordinary', 'ordinary', 'ordinary']
    assert result.attempted == 4 and not result.plans
    assert len(result.passage_proposals) == 1 and len(result.passage_trials) == 1
    assert sum(result.rejections.values()) == 4


def test_disabled_policy_does_not_use_a_supplied_hint_or_cache_references(monkeypatch):
    import swarm_sim.trajectory2d.planning as planning
    def ordinary(*a, **kw):
        assert 'cache_reference' not in kw
        return None
    monkeypatch.setattr(planning, '_candidate', ordinary)
    result = extend_catalog((separate_loiter_snapshot()[0], config(attempts=1, passage_guidance='none'),
                              None, Catalog(), 1, None, None, 'initial', windows()))
    assert result.attempted == 1 and not result.passage_contexts and not result.passage_proposals
    assert not result.geometric_references


def test_four_aircraft_fixed_departure_corridor_selects_window_guided_plans_and_executes():
    """Curated straight references isolate intermediate timing, not route sampling.

    No validator/compatibility check is mocked. All aircraft share one target,
    release simultaneously, keep moving after passage and use actual controls.
    """
    from itertools import combinations
    from swarm_sim.trajectory2d.execution import simulate
    from swarm_sim.trajectory2d.parallel import generate_catalogs
    from swarm_sim.trajectory2d.planning import continuous_distance_lower_bound, required_distance
    from swarm_sim.trajectory2d.reservations import occupancy, first_conflict
    from swarm_sim.trajectory2d.search import JointSearch
    c = config(seed=1, horizon=220, planning_strategy='adaptive', initial_candidates=1,
               candidates=2, attempts=2)
    times = np.arange(1101) * c.dt
    snapshots, catalogs = [], []
    for k, offset in enumerate((0., -50., -300., -500.)):
        snapshot = Snapshot(k, (offset, 0., 0., 0.), 12, 60, 60)
        states = np.column_stack((offset + 12*times, np.zeros((len(times), 3))))
        vertical = np.column_stack((np.full(len(times), 60.), np.zeros(len(times))))
        reference = Trajectory(k, 0, 12, c.dt, states, np.zeros(len(times)-1), (1320-offset)/12, 0,
            vertical_states=vertical, vertical_accelerations=np.zeros(len(times)-1), cruise_altitude=60)
        parent = convert_candidate(reference, snapshot, c)
        assert parent is not None and validate_individual(parent, snapshot, c)
        snapshots.append(snapshot)
        catalogs.append(Catalog(plans=[parent], attempted=1, geometric_references={0: reference}))
    search = JointSearch(c)
    assert search.solve([cat.plans for cat in catalogs]) is None
    indices = search.expansion_indices(catalogs, snapshots)
    assert indices == [0, 1]
    hints = search.passage_hints(catalogs, indices)
    transferred = pickle.loads(pickle.dumps(catalogs))
    for cat in transferred:
        assert not cat.geometric_references[0].states.flags.writeable
    tasks = [(snapshots[i], c, None, catalogs[i], 2, None, None, 'conflict', hints[i]) for i in indices]
    serial = generate_catalogs(extend_catalog, tasks, 1)
    parallel = generate_catalogs(extend_catalog, [
        (snapshots[i], c, None, transferred[i], 2, None, None, 'conflict', hints[i]) for i in indices], 2)
    for i, a, b in zip(indices, serial, parallel):
        assert a.passage_proposals == b.passage_proposals
        assert [p.to_dict() for p in a.plans] == [p.to_dict() for p in b.plans]
        assert a.attempted == b.attempted == 2
        assert a.passage_proposals[0]['accepted'] and a.passage_proposals[0]['window_verified']
        catalogs[i] = a
    selected = search.solve([cat.plans for cat in catalogs])
    assert selected is not None
    assert [p.candidate_id for p in selected] == [1, 1, 0, 0]
    assert all(p.release_time == 0 for p in selected)
    for p, snapshot in zip(selected, snapshots):
        assert validate_individual(p, snapshot, c)
    for a, b in combinations(selected, 2):
        assert continuous_distance_lower_bound(a, b) >= required_distance(a, b, c)
        assert first_conflict(occupancy(a, c), occupancy(b, c)) is None
    execution = simulate(selected, c)
    assert execution.report['execution_validated'], execution.report['execution_failures']
