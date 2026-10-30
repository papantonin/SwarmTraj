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
    assert Config().conflict_guidance == "none"
    with pytest.raises(ValueError, match="conflict_guidance"):
        Config(conflict_guidance="unknown")


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


def test_hints_respect_cruise_and_departure_bounds_and_separate_ties():
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', cruise_altitude_range=(40, 110), min_extra_loiter=2, max_extra_loiter=4)
    first, second = hint(), hint(1)
    assert first.modes(config) == ["altitude", "lateral", "departure"]
    assert first.cruise_level(40, 110) == 40
    assert second.cruise_level(40, 110) == 110
    assert first.release_time(config) == config.activation_time + 2
    assert second.release_time(config) == config.activation_time + 4
    assert first.modes(replace(config, cruise_altitude_range=None, detour=0, max_extra_loiter=2)) == []


def test_lateral_hint_changes_only_one_optional_waypoint_with_bounded_offset():
    points = [np.array([30., 0]), np.array([65., 0]), np.array([100., 0])]
    original = [p.copy() for p in points]
    hint().lateral_waypoints(np.zeros(2), np.array([100., 0]), np.array([0., 1]), points, 45)
    assert np.array_equal(points[0], original[0])
    assert np.array_equal(points[2], original[2])
    assert np.array_equal(points[1], [50., -45.])


def test_guided_rejections_consume_attempt_budget_and_keep_ordinary_exploration(monkeypatch):
    import swarm_sim.trajectory2d.planning as planning
    calls = []

    def candidate(snapshot, config, candidate_id, rng, angle, diagnostics, **kwargs):
        calls.append(kwargs.get("repair_mode"))
        return Trajectory(snapshot.drone_id, candidate_id, snapshot.speed, config.dt,
                          np.tile(snapshot.state, (2, 1)), np.zeros(1), 1, 0)

    monkeypatch.setattr(planning, "_candidate", candidate)
    # Every proposal fails independent validation: hints must not bypass it.
    monkeypatch.setattr(planning, "validate_individual", lambda *args, **kwargs: False)
    config = Config(conflict_guidance="witness", attempts=6)
    catalog = extend_catalog((separate_loiter_snapshot()[0], config, None, Catalog(), 4, hint()))
    assert calls == ["lateral", None, "departure", None, "lateral", None]
    assert catalog.attempted == 6
    assert catalog.rejections == {"individual_validation": 6}
    assert len(catalog.guided_proposals) == 3
    assert all(not p["accepted"] for p in catalog.guided_proposals)
    assert not catalog.plans


def test_search_records_actual_failed_pair_and_builds_opposed_hints():
    config = Config(conflict_guidance="witness")
    snapshot = separate_loiter_snapshot()[0]
    plan = Trajectory(0, 0, 12, .2, np.tile(snapshot.state, (2, 1)), np.zeros(1), 1, 0)
    other = replace(plan, drone_id=1)
    search = JointSearch(config)
    assert not search.compatible(plan, other)
    hints = search.repair_hints([0, 1, 2])
    assert set(hints) == {0, 1}
    assert hints[0].conflict == hints[1].conflict
    assert hints[0].side != hints[1].side
    assert search.repair_hints([3]) == {}
    assert JointSearch(Config()).repair_hints([0, 1]) == {}


def test_margin_hints_use_current_incumbent_not_historical_rejected_pair():
    config = Config(conflict_guidance="witness")
    snapshot = separate_loiter_snapshot()[0]
    first = Trajectory(0, 0, 12, .2, np.tile(snapshot.state, (2, 1)), np.zeros(1), 1, 0)
    second = replace(first, drone_id=1)
    search = JointSearch(config)
    assert not search.compatible(first, second)
    moved = second.states.copy()
    moved[:, 0] += 100
    incumbent = (replace(first, candidate_id=4), replace(second, states=moved, candidate_id=5))
    hints = search.repair_hints([0, 1], incumbent)
    assert hints[0].conflict.candidate_ids == (4, 5)
    assert hints[0].conflict.slack > 0
    assert hints[0].conflict == hints[1].conflict


def test_rejected_margin_repairs_keep_feasible_incumbent_and_attempt_caps(monkeypatch):
    """Synthetic graph tests selection bookkeeping, not physical flight."""
    import swarm_sim.trajectory2d.planning as planning

    def candidate(snapshot, config, candidate_id, rng, *args, **kwargs):
        return Trajectory(snapshot.drone_id, candidate_id, snapshot.speed, config.dt,
                          np.tile(snapshot.state, (2, 1)), np.zeros(1), 1, 0)

    monkeypatch.setattr(planning, "_candidate", candidate)
    monkeypatch.setattr(planning, "validate_individual", lambda p, *args, **kwargs: p.candidate_id == 0)
    monkeypatch.setattr(planning, "continuous_distance_lower_bound", lambda *args: 100.)
    config = Config(conflict_guidance="witness", planning_strategy="adaptive", initial_candidates=1,
                    selection_policy="max_margin", candidates=2, attempts=3)
    result = plan_mission(separate_loiter_snapshot(), config)
    assert result.validated
    assert [p.candidate_id for p in result.plans] == [0, 0, 0, 0]
    assert result.counts == (1, 1, 1, 1)
    assert result.search_nodes <= config.max_search_nodes
    repaired = [c for c in result.diagnostics["catalogs"] if c["guided_proposals"]]
    assert len(repaired) == 2
    assert all(c["attempted"] == 3 and c["attempt_budget_exhausted"] for c in repaired)
    assert all(not g["accepted"] for c in repaired for g in c["guided_proposals"])


def test_guided_catalog_spawn_is_deterministic_and_plans_stay_readonly():
    config = Config(conflict_guidance="witness", candidates=2, attempts=10)
    snapshots = separate_loiter_snapshot()[:2]
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
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target=(500, 300), target_altitude=30, horizon=150,
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
