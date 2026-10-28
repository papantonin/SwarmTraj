"""Free resource windows guide proposals before rollout, never grant safety."""

from dataclasses import replace
import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, Trajectory, separate_loiter_snapshot
from swarm_sim.trajectory2d.catalog import Catalog, extend_catalog
from swarm_sim.trajectory2d.interval_guidance import DepartureWindows, departure_windows
from swarm_sim.trajectory2d.reservations import Occupancy
from swarm_sim.trajectory2d.parallel import generate_catalogs


def parent():
    return Trajectory(0, 0, 12, .2, np.zeros((2, 4)), np.zeros(1), 12, 3)


def test_forbidden_release_equation_and_closed_boundaries():
    config = Config(reservation_policy='grid', interval_guidance='safe_windows', max_extra_loiter=10)
    own = Occupancy(0, 0, {(0, 0): ((8., 10.),)}, 1)
    blocker = Occupancy(1, 0, {(0, 0): ((9., 12.),)}, 1)
    windows = departure_windows(parent(), own, [blocker], config)
    assert windows.constraints == 1
    assert len(windows.intervals_s) == 1
    low, high = windows.intervals_s[0]
    assert low > 7 and high == 13
    rng = np.random.default_rng(7)
    for _ in range(10):
        release, interval = windows.choose(rng)
        assert low <= release <= high
        assert interval == (low, high)


def test_fixed_departure_is_never_widened_and_budget_exhaustion_is_only_a_hint_failure():
    config = Config(reservation_policy='grid', interval_guidance='safe_windows', max_extra_loiter=0)
    own = Occupancy(0, 0, {(0, 0): ((8., 10.),)}, 1)
    blocker = Occupancy(1, 0, {(0, 0): ((9., 12.), (14., 15.))}, 1)
    assert not departure_windows(parent(), own, [blocker], config).intervals_s
    assert departure_windows(parent(), own, [], config).intervals_s == ((3, 3),)
    failed = departure_windows(parent(), own, [blocker], replace(config, interval_max_constraints=1))
    assert failed.budget_exhausted and failed.intervals_s == ()


def test_guidance_is_applied_before_candidate_construction_and_keeps_attempt_budget(monkeypatch):
    import swarm_sim.trajectory2d.planning as planning
    calls = []
    def candidate(snapshot, config, candidate_id, rng, *args, **kw):
        calls.append(kw.get('departure_override'))
        return None
    monkeypatch.setattr(planning, '_candidate', candidate)
    config = Config(reservation_policy='grid', interval_guidance='safe_windows', attempts=4)
    windows = DepartureWindows(0, (1, 2, 3), ((8., 9.),), 1)
    catalog = extend_catalog((separate_loiter_snapshot()[0], config, None, Catalog(), 2, None, windows))
    assert catalog.attempted == 4 and not catalog.plans
    assert 8 <= calls[0] <= 9 and 8 <= calls[2] <= 9
    assert calls[1] is calls[3] is None
    assert len(catalog.interval_proposals) == 2
    assert not any(p['accepted'] for p in catalog.interval_proposals)


def test_empty_window_falls_back_without_claiming_infeasibility(monkeypatch):
    import swarm_sim.trajectory2d.planning as planning
    calls = []
    def candidate(snapshot, config, candidate_id, rng, *args, **kw):
        calls.append(kw)
        return None
    monkeypatch.setattr(planning, '_candidate', candidate)
    config = Config(reservation_policy='grid', interval_guidance='safe_windows', attempts=2)
    catalog = extend_catalog((separate_loiter_snapshot()[0], config, None, Catalog(), 2, None,
                              DepartureWindows(0, (1,), (), 2)))
    assert catalog.attempted == 2
    assert all('departure_override' not in kw for kw in calls)


def test_disabled_policy_ignores_supplied_window_context(monkeypatch):
    import swarm_sim.trajectory2d.planning as planning
    def candidate(snapshot, config, candidate_id, rng, *args, **kw):
        assert 'departure_override' not in kw
        return None
    monkeypatch.setattr(planning, '_candidate', candidate)
    config = Config(attempts=1)
    windows = DepartureWindows(0, (1,), ((8., 9.),), 1)
    catalog = extend_catalog((separate_loiter_snapshot()[0], config, None, Catalog(), 2, None, windows))
    assert catalog.attempted == 1 and catalog.interval_contexts == []


def test_window_proposals_are_spawn_deterministic():
    config = Config(reservation_policy='grid', interval_guidance='safe_windows', candidates=2, attempts=12)
    snapshots = separate_loiter_snapshot()[:2]
    windows = DepartureWindows(0, (2, 3), ((8., 10.),), 1)
    serial = [extend_catalog((s, config, None, Catalog(), 2, None, windows)) for s in snapshots]
    parallel = generate_catalogs(extend_catalog,
        [(s, config, None, Catalog(), 2, None, windows) for s in snapshots], 2)
    for a, b in zip(serial, parallel):
        assert a.interval_proposals == b.interval_proposals and a.attempted == b.attempted
        assert [p.to_dict() for p in a.plans] == [p.to_dict() for p in b.plans]
        assert a.plans and all(not p.states.flags.writeable for p in b.plans)
