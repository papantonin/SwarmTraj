"""Parallelism/batching must not change plans, budgets, safety checks or RNGs."""

from dataclasses import replace
import pickle
import sys

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.parallel import generate_catalogs, worker_count
from swarm_sim.trajectory2d.planning import _candidate, _generate_catalog


def assert_same_catalog(first, second):
    assert len(first) == len(second)
    for a, b in zip(first, second):
        assert a.to_dict() == b.to_dict()
        for name in ("states", "accelerations", "vertical_states", "vertical_accelerations", "flight_states", "flight_controls"):
            x, y = getattr(a, name), getattr(b, name)
            if x is not None:
                np.testing.assert_array_equal(x, y)
                assert not y.flags.writeable


@pytest.mark.parametrize("parameters", [
    {"planning_workers": -1}, {"planning_workers": 1.5}, {"planning_workers": True},
    {"planning_workers": np.nan}, {"rollout_check_interval": 0},
    {"rollout_check_interval": -1}, {"rollout_check_interval": False}, {"rollout_check_interval": 1.5},
])
def test_invalid_performance_configuration_is_rejected(parameters):
    with pytest.raises(ValueError):
        Config(**parameters)


@pytest.mark.parametrize("requested,count,expected", [(0, 4, 4), (1, 4, 1), (2, 4, 2), (12, 4, 4), (4, 2, 2), (0, 1, 1)])
def test_worker_count_respects_cpu_and_drone_count(monkeypatch, requested, count, expected):
    monkeypatch.setattr("swarm_sim.trajectory2d.parallel.available_cpus", lambda: 4)
    assert worker_count(requested, count) == expected


def test_interactive_call_warns_and_falls_back_to_serial(monkeypatch):
    monkeypatch.setattr(sys.modules["__main__"], "__file__", "<stdin>", raising=False)
    monkeypatch.setattr("swarm_sim.trajectory2d.parallel.available_cpus", lambda: 4)
    with pytest.warns(RuntimeWarning, match="using serial"):
        assert generate_catalogs(abs, [-1, -2], 2) == [1, 2]


def test_spawn_catalogs_and_joint_search_are_exactly_reproducible():
    snapshots = separate_loiter_snapshot()[:2]
    config = Config(candidates=3, attempts=8)
    tasks = [(s, config, None) for s in snapshots]
    serial = generate_catalogs(_generate_catalog, tasks, 1)
    parallel = generate_catalogs(_generate_catalog, tasks, 2)
    for a, b in zip(serial, parallel):
        assert a
        assert_same_catalog(a, b)
    one = plan_mission(snapshots, config)
    many = plan_mission(snapshots[::-1], replace(config, planning_workers=2))
    assert one.status == many.status
    assert one.counts == many.counts
    assert one.search_nodes == many.search_nodes
    assert one.minimum_distance_lower_bound == many.minimum_distance_lower_bound
    assert_same_catalog(one.plans, many.plans)


def coupled_config():
    return Config(dimensions=3, target_altitude=30, max_descent_rate=10, descent_angle_deg=30,
                  cruise_altitude_range=(30, 120), horizon=120, candidates=3, attempts=12)


def test_batched_checks_preserve_entire_coupled_catalog_and_pickle_immutability():
    snapshot = replace(separate_loiter_snapshot()[0], altitude=30)
    config = coupled_config()
    original = _generate_catalog((snapshot, replace(config, rollout_check_interval=1), None))
    batched = _generate_catalog((snapshot, config, None))
    assert original
    assert_same_catalog(original, batched)
    assert_same_catalog(original, pickle.loads(pickle.dumps(original)))


def test_spawn_preserves_complete_coupled_catalogs_not_just_selected_plans():
    config = coupled_config()
    snapshots = tuple(replace(s, altitude=30 + 25 * k) for k, s in enumerate(separate_loiter_snapshot()[:2]))
    tasks = [(s, config, None) for s in snapshots]
    serial = generate_catalogs(_generate_catalog, tasks, 1)
    parallel = generate_catalogs(_generate_catalog, tasks, 2)
    for first, second in zip(serial, parallel):
        assert first
        assert_same_catalog(first, second)


def test_batch_covers_both_endpoints_of_every_primitive_including_last_partial_block(monkeypatch):
    import swarm_sim.trajectory2d.coupled_planning as coupled
    original_check = coupled.envelope_ok
    calls = []

    def record(states, controls, config):
        calls.append((states.copy(), controls.copy()))
        return original_check(states, controls, config)

    monkeypatch.setattr(coupled, "envelope_ok", record)
    config = coupled_config()
    snapshot = replace(separate_loiter_snapshot()[0], altitude=30)
    rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([config.seed, snapshot.drone_id])))
    plan = None
    for candidate_id in range(config.attempts):
        calls.clear()
        plan = _candidate(snapshot, config, candidate_id, rng)
        if plan is not None:
            break
    assert plan is not None
    points = np.concatenate([s for s, _ in calls])
    controls = np.concatenate([u for _, u in calls])
    np.testing.assert_array_equal(points[:, 0], plan.flight_states[:-1])
    np.testing.assert_array_equal(points[:, 1], plan.flight_states[1:])
    np.testing.assert_array_equal(controls[:, 0], plan.flight_controls)
    assert len(calls[-1][0]) == len(plan.flight_controls) % config.rollout_check_interval


def _worker_failure(task):
    raise ValueError("worker failed, no partial mission")


def test_worker_exceptions_are_propagated_instead_of_returning_partial_results():
    with pytest.raises(ValueError, match="worker failed"):
        generate_catalogs(_worker_failure, [0, 1], 2)
