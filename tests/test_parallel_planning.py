"""Parallelism/batching must not change plans, budgets, safety checks or RNGs."""

from dataclasses import replace
import pickle
import sys

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, plan_mission
from swarm_sim.trajectory2d import separate_loiter_snapshot as _snapshots


def separate_loiter_snapshot(*args, **kwargs):
    return tuple(replace(s, altitude=80.) for s in _snapshots(*args, **kwargs))


def coupled_config():
    return Config(dimensions=3, target_altitude=30, max_descent_rate=10, descent_angle_deg=30,
                  cruise_altitude_range=(30,120), horizon=120, candidates=3, attempts=12)

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
        Config(dimensions=3, **parameters)


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
    config = Config(dimensions=3, candidates=3, attempts=8)
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






def test_spawn_preserves_complete_coupled_catalogs_not_just_selected_plans():
    config = coupled_config()
    snapshots = tuple(replace(s, altitude=30 + 25 * k) for k, s in enumerate(separate_loiter_snapshot()[:2]))
    tasks = [(s, config, None) for s in snapshots]
    serial = generate_catalogs(_generate_catalog, tasks, 1)
    parallel = generate_catalogs(_generate_catalog, tasks, 2)
    for first, second in zip(serial, parallel):
        assert first
        assert_same_catalog(first, second)




def _worker_failure(task):
    raise ValueError("worker failed, no partial mission")


def test_worker_exceptions_are_propagated_instead_of_returning_partial_results():
    with pytest.raises(ValueError, match="worker failed"):
        generate_catalogs(_worker_failure, [0, 1], 2)
