"""Offline witness hints remain bounded proposals, not accepted trajectories."""

from dataclasses import replace
import json

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, Trajectory, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.catalog import Catalog, extend_catalog
from swarm_sim.trajectory2d.conflict_guidance import ConflictWitness, RepairHint, witness
from swarm_sim.trajectory2d.parallel import generate_catalogs
from swarm_sim.trajectory2d.search import JointSearch


def hint(drone_id=0):
    return RepairHint(ConflictWitness((0, 1), (2, 3), 10., ((50., 0., 60.), (50., 0., 60.)),
                                      0., 17.), drone_id)


def test_guidance_policy_is_explicit_and_validated():
    assert Config(dimensions=3, ).conflict_guidance == "none"
    with pytest.raises(ValueError, match="conflict_guidance"):
        Config(dimensions=3, conflict_guidance="unknown")


def test_witness_is_symmetric_and_reports_sample_not_collision_root():
    first = Trajectory(0, 2, 12, .2, np.array([[0., 0, 0, 0], [5, 0, 0, 0]]), np.zeros(1), .2, 0)
    second = replace(first, drone_id=1, candidate_id=3,
                     states=np.array([[20., 0, 0, 0], [6, 0, 0, 0]]))
    item = witness(first, second, -1.4, 17)
    assert item == witness(second, first, -1.4, 17)
    assert item.sample_time_s == .2
    assert item.positions == ((5., 0.), (6., 0.))
    assert item.candidate_ids == (2, 3)
    assert item.slack == -18.4














def test_guided_catalog_spawn_is_deterministic_and_plans_stay_readonly():
    config = Config(dimensions=3, conflict_guidance="witness", candidates=2, attempts=10)
    snapshots = tuple(replace(s, altitude=80.) for s in separate_loiter_snapshot()[:2])
    serial = [extend_catalog((s, config, None, Catalog(), 2, hint(s.drone_id))) for s in snapshots]
    parallel = generate_catalogs(extend_catalog,
                                 [(s, config, None, Catalog(), 2, hint(s.drone_id)) for s in snapshots], 2)
    for a, b in zip(serial, parallel):
        assert a.attempted == b.attempted
        assert a.guided_proposals == b.guided_proposals
        assert [p.to_dict() for p in a.plans] == [p.to_dict() for p in b.plans]
        assert a.plans
        assert all(not p.states.flags.writeable for p in b.plans)


def test_four_drone_guided_expansion_revalidates_and_is_spawn_reproducible():
    config = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', target=(500, 300), target_altitude=30, horizon=150,
                    cruise_altitude_range=(40, 110), descent_angle_deg=30, max_descent_rate=10,
                    activation_time=0, max_extra_loiter=0, approach_spread_deg=180, candidates=8,
                    planning_strategy="adaptive", initial_candidates=1, search_order="mrv",
                    conflict_guidance="witness")
    snapshots = tuple(replace(s, altitude=30 + 25 * k) for k, s in enumerate(separate_loiter_snapshot(4)))
    serial = plan_mission(snapshots, config)
    parallel = plan_mission(snapshots[::-1], replace(config, planning_workers=2))
    assert serial.validated and parallel.validated
    assert serial.diagnostics == parallel.diagnostics
    assert [p.to_dict() for p in serial.plans] == [p.to_dict() for p in parallel.plans]
    guided = [g for c in serial.diagnostics["catalogs"] for g in c["guided_proposals"]]
    assert guided and any(g["accepted"] for g in guided)
    assert all(p.release_time == 0 for p in serial.plans)
    assert all(40 <= p.cruise_altitude <= 110 for p in serial.plans)
    assert all("witness" not in json.dumps(p.to_dict()) for p in serial.plans)
    from swarm_sim.trajectory2d.execution import simulate
    assert simulate(serial.plans, config).report["execution_validated"]
