"""Adaptive planning: bounded work, exact RNG continuation and unchanged checks."""

from dataclasses import replace
import json

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, Trajectory, plan_mission
from swarm_sim.trajectory2d import separate_loiter_snapshot as _snapshots
from swarm_sim.trajectory2d.catalog import Catalog, extend_catalog
from swarm_sim.trajectory2d.parallel import generate_catalogs
from swarm_sim.trajectory2d.planning import _generate_catalog, mission_document
from swarm_sim.trajectory2d.search import JointSearch
from swarm_sim.trajectory2d.execution import simulate

def separate_loiter_snapshot(*args, **kwargs):
    return tuple(replace(s, altitude=80.) for s in _snapshots(*args, **kwargs))



@pytest.mark.parametrize("values", [
    {"planning_strategy": "unknown"}, {"search_order": "unknown"},
    {"initial_candidates": 0}, {"initial_candidates": -1},
    {"initial_candidates": True}, {"initial_candidates": 1.5},
])
def test_invalid_adaptive_settings(values):
    with pytest.raises(ValueError):
        Config(dimensions=3, **values)


def test_resumed_catalogs_equal_full_generation_with_same_rng_and_attempts():
    config = Config(dimensions=3, candidates=4, attempts=15)
    for snapshot in separate_loiter_snapshot():
        catalog = extend_catalog((snapshot, config, None, Catalog(), 1))
        first_ids = [p.candidate_id for p in catalog.plans]
        catalog = extend_catalog((snapshot, config, None, catalog, config.candidates))
        full = _generate_catalog((snapshot, config, None))
        assert [p.to_dict() for p in full] == [p.to_dict() for p in catalog.plans]
        assert first_ids == [p.candidate_id for p in catalog.plans[:len(first_ids)]]
        assert catalog.attempted <= config.attempts
        assert catalog.attempted == len(catalog.plans) + sum(catalog.rejections.values())


def test_adaptive_four_drone_plans_and_diagnostics_are_spawn_reproducible():
    config = Config(dimensions=3, planning_strategy="adaptive", search_order="mrv", initial_candidates=2,
                    candidates=6, attempts=20)
    snapshots = separate_loiter_snapshot()
    serial = plan_mission(snapshots, config)
    parallel_config = replace(config, planning_workers=2)
    parallel = plan_mission(snapshots[::-1], parallel_config)
    assert serial.validated and parallel.validated
    assert serial.diagnostics == parallel.diagnostics
    assert serial.search_nodes == parallel.search_nodes
    assert [p.to_dict() for p in serial.plans] == [p.to_dict() for p in parallel.plans]
    assert [p.drone_id for p in serial.plans] == [0, 1, 2, 3]
    assert serial.timings["total_wall_s"] > 0
    # Timings cannot leak into the reproducible manifest, even if overwritten.
    changed = replace(serial, timings={"total_wall_s": 999})
    assert mission_document(serial, config, snapshots) == mission_document(changed, config, snapshots)
    again = plan_mission(snapshots, config)
    assert json.dumps(mission_document(serial, config, snapshots)) == json.dumps(mission_document(again, config, snapshots))
















def test_unreachable_preflight_has_timing_without_catalog_work():
    result = plan_mission(separate_loiter_snapshot(), Config(dimensions=3, target=(100000, 0)))
    assert result.status == "destination_unreachable_within_horizon"
    assert result.timings["total_wall_s"] > 0
    assert not result.diagnostics


def test_no_candidate_accounts_for_all_rejections():
    config = Config(dimensions=3, horizon=3, activation_time=0, attempts=2, target=(0, 0), planning_strategy="adaptive")
    snapshots = tuple(replace(s, state=(x, y, 0, 0)) for s, (x, y) in zip(
        separate_loiter_snapshot(), [(-10, -10), (-10, 10), (10, -10), (10, 10)]))
    result = plan_mission(snapshots, config)
    assert result.status == "no_candidate"
    for catalog in result.diagnostics["catalogs"]:
        assert catalog["attempted"] == 2
        assert sum(catalog["rejections"].values()) == 2
        assert all(reason.startswith("physical_") for reason in catalog["rejections"])


def test_spawn_resumes_existing_catalog_and_read_only_plans():
    config = Config(dimensions=3, candidates=3, attempts=10)
    snapshots = separate_loiter_snapshot()
    tasks = [(s, config, None, Catalog(), 1) for s in snapshots]
    first = generate_catalogs(extend_catalog, tasks, 2)
    resumed = generate_catalogs(extend_catalog, [(s, config, None, c, 3) for s, c in zip(snapshots, first)], 2)
    for snapshot, catalog in zip(snapshots, resumed):
        assert [p.to_dict() for p in catalog.plans] == [p.to_dict() for p in _generate_catalog((snapshot, config, None))]
        assert all(not p.states.flags.writeable for p in catalog.plans)


def test_four_coupled_aircraft_expand_conflicting_pair_and_execute():
    config = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', target=(500, 300), target_altitude=30, horizon=150,
                    cruise_altitude_range=(40, 110), descent_angle_deg=30, max_descent_rate=10,
                    activation_time=0, max_extra_loiter=0, approach_spread_deg=180, candidates=8,
                    planning_strategy="adaptive", initial_candidates=1, search_order="mrv")
    snapshots = tuple(replace(s, altitude=30 + 25 * k) for k, s in enumerate(separate_loiter_snapshot(4)))
    result = plan_mission(snapshots, config)
    assert result.validated
    rounds = result.diagnostics["rounds"]
    assert len(rounds) == 2 and not rounds[0]["solved"] and rounds[1]["solved"]
    assert rounds[1]["expanded_drone_ids"] == [1, 2]
    assert result.counts == (1, 2, 2, 1)
    execution = simulate(result.plans, config)
    for key in ("all_arrived", "tube_conditions_verified", "separation_verified_in_execution_model",
                "physical_limits_verified", "vertical_limits_verified", "approach_sectors_verified_in_execution",
                "descent_angle_verified_in_execution"):
        assert execution.report[key], key
