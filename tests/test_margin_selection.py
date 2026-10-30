"""Finite-catalog leximin selection, bounded refinement and safety regressions."""

from dataclasses import replace
from itertools import combinations, product

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, Obstacle, Trajectory, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.planning import continuous_distance_lower_bound, required_distance
from swarm_sim.trajectory2d.search import JointSearch


@pytest.mark.parametrize("values", [
    {"selection_policy": "unknown"}, {"margin_search_nodes": 0},
    {"margin_search_nodes": True}, {"margin_search_nodes": 1.5},
    {"margin_expansion_rounds": -1}, {"margin_expansion_rounds": True},
    {"margin_expansion_rounds": 0.5},
])
def test_invalid_margin_configuration(values):
    with pytest.raises(ValueError):
        Config(**values)


def artificial_catalogs(monkeypatch, config, fixed_bottleneck=False):
    """Artificial pair scores for search proofs; not physically valid flights."""
    import swarm_sim.trajectory2d.planning as planning
    catalogs = [[Trajectory(i, c, 12, 0.2, np.zeros((2, 4)), np.zeros(1), 1, 0)
                 for c in range(2)] for i in range(4)]

    def slack(a, b):
        a, b = sorted((a, b), key=lambda p: p.drone_id)
        if fixed_bottleneck and (a.drone_id, b.drone_id) == (0, 1):
            return 1.0
        if not fixed_bottleneck and (a.drone_id, a.candidate_id, b.drone_id, b.candidate_id) == (0, 1, 1, 0):
            return -1.0
        return float(1 + 10 * (a.candidate_id + b.candidate_id))

    monkeypatch.setattr(planning, "continuous_distance_lower_bound", lambda a, b: required_distance(a, b, config) + slack(a, b))
    return catalogs, slack


@pytest.mark.parametrize("fixed_bottleneck", [False, True])
def test_branch_and_bound_matches_independent_exhaustive_enumeration(monkeypatch, fixed_bottleneck):
    config = Config(selection_policy="max_margin", margin_expansion_rounds=0)
    catalogs, slack = artificial_catalogs(monkeypatch, config, fixed_bottleneck)
    initial = tuple(domain[0] for domain in catalogs)
    search = JointSearch(config)
    selected, info = search.refine(catalogs, initial)
    feasible = [tuple(sorted(slack(a, b) for a, b in combinations(plans, 2)))
                for plans in product(*catalogs) if all(slack(a, b) >= 1e-7 for a, b in combinations(plans, 2))]
    np.testing.assert_allclose(search.score(selected), max(feasible), rtol=0, atol=1e-12)
    assert [p.candidate_id for p in selected] == [1, 1, 1, 1]
    assert info["catalog_optimal"]
    assert search.score(selected) > search.score(initial)
    if fixed_bottleneck:
        assert search.score(selected)[0] == search.score(initial)[0]  # Secondary margins still improve.


def test_refinement_retains_safe_incumbent_when_global_budget_is_used(monkeypatch):
    config = Config(selection_policy="max_margin", max_search_nodes=4)
    catalogs, slack = artificial_catalogs(monkeypatch, config)
    search = JointSearch(config)
    search.nodes = 4
    initial = tuple(c[0] for c in catalogs)
    selected, info = search.refine(catalogs, initial)
    assert selected == initial
    assert search.nodes == 4 and search.optimization_nodes == 0
    assert not info["catalog_optimal"] and search.optimization_budget_exhausted
    assert not search.exhausted  # Optimization limit is not a failed feasible search.
    assert all(slack(a, b) > 0 for a, b in combinations(selected, 2))


@pytest.mark.parametrize("seed", range(8))
def test_random_four_drone_compatibility_graphs_match_brute_force(monkeypatch, seed):
    import swarm_sim.trajectory2d.planning as planning
    config = Config(selection_policy="max_margin", separation=8, position_error=0, clock_error=0)
    catalogs, _ = artificial_catalogs(monkeypatch, config)
    rng = np.random.default_rng(seed)
    weights = {}
    for i, j in combinations(range(4), 2):
        for a, b in product(range(2), repeat=2):
            weights[i, a, j, b] = int(rng.integers(1 if a == b == 0 else -3, 40))

    def slack(a, b):
        a, b = sorted((a, b), key=lambda p: p.drone_id)
        return weights[a.drone_id, a.candidate_id, b.drone_id, b.candidate_id]

    monkeypatch.setattr(planning, "continuous_distance_lower_bound", lambda a, b: 8 + slack(a, b))
    search = JointSearch(config)
    selected, info = search.refine(catalogs, tuple(c[0] for c in catalogs))
    expected = max(tuple(sorted(slack(a, b) for a, b in combinations(plans, 2)))
                   for plans in product(*catalogs) if all(slack(a, b) > 0 for a, b in combinations(plans, 2)))
    assert search.score(selected) == expected
    assert info['catalog_optimal']


def test_refinement_budget_is_cumulative_and_does_not_lose_incumbent(monkeypatch):
    config = Config(selection_policy="max_margin", margin_search_nodes=3)
    catalogs, _ = artificial_catalogs(monkeypatch, config)
    search = JointSearch(config)
    initial = tuple(c[0] for c in catalogs)
    selected, info = search.refine(catalogs, initial)
    again, _ = search.refine(catalogs, selected)
    assert search.optimization_nodes == 3
    assert search.score(again) >= search.score(initial)
    assert search.score(again) >= search.score(selected)
    assert not info["catalog_optimal"]


def test_single_aircraft_has_no_pairwise_optimization(monkeypatch):
    config = Config(selection_policy="max_margin")
    catalogs, _ = artificial_catalogs(monkeypatch, config)
    search = JointSearch(config)
    selected, info = search.refine(catalogs[:1], catalogs[0][:1])
    assert selected == tuple(catalogs[0][:1])
    assert info["sorted_slack_m"] == []
    assert search.optimization_nodes == 0


def test_mission_stays_validated_when_only_refinement_budget_is_exhausted(monkeypatch):
    import swarm_sim.trajectory2d.planning as planning
    # Synthetic compatibility graph, not a physical flight validation.
    monkeypatch.setattr(planning, '_candidate', lambda s, c, i, rng, *args, **kw:
                        Trajectory(s.drone_id, i, 12, c.dt, np.zeros((2, 4)), np.zeros(1), 1, 0))
    monkeypatch.setattr(planning, 'validate_individual', lambda *args, **kw: True)
    monkeypatch.setattr(planning, 'continuous_distance_lower_bound', lambda *args: 100.0)
    config = Config(planning_strategy='adaptive', initial_candidates=1,
                    selection_policy='max_margin', max_search_nodes=4)
    result = plan_mission(separate_loiter_snapshot(4), config)
    assert result.validated and len(result.plans) == 4
    assert result.search_nodes == 4
    assert result.diagnostics['selection']['optimization_budget_exhausted']
    assert result.diagnostics['selection']['optimization_nodes'] == 0
    assert result.counts == (1, 1, 1, 1)


def test_margin_selection_is_spawn_and_input_order_reproducible():
    config = Config(planning_strategy="adaptive", search_order="mrv", initial_candidates=2,
                    candidates=4, attempts=15, selection_policy="max_margin")
    snapshots = separate_loiter_snapshot(4)
    serial = plan_mission(snapshots, config)
    parallel = plan_mission(snapshots[::-1], replace(config, planning_workers=2))
    assert serial.validated and parallel.validated
    assert serial.diagnostics == parallel.diagnostics
    assert [p.to_dict() for p in serial.plans] == [p.to_dict() for p in parallel.plans]
    assert serial.search_nodes <= config.max_search_nodes
    assert serial.diagnostics['selection']['optimization_nodes'] <= config.margin_search_nodes
    for catalog in serial.diagnostics['catalogs']:
        assert catalog['attempted'] <= config.attempts and catalog['accepted'] <= config.candidates


def test_four_coupled_aircraft_prefer_margin_without_relaxing_obstacles_or_flight():
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target=(500, 300), target_altitude=30, horizon=180,
                    cruise_altitude_range=(40, 110), descent_angle_deg=30, max_descent_rate=10,
                    activation_time=0, max_extra_loiter=0, approach_spread_deg=180, candidates=8,
                    planning_strategy="adaptive", search_order="mrv", selection_policy="max_margin",
                    obstacles=(Obstacle.cylinder(200, 120, 100, 0, 150), Obstacle.box(260, 130, 0, 380, 250, 130)))
    snapshots = tuple(replace(s, altitude=30 + 25 * k) for k, s in enumerate(separate_loiter_snapshot(4)))
    result = plan_mission(snapshots, config)
    assert result.validated, (result.status, result.diagnostics)
    score = tuple(sorted(continuous_distance_lower_bound(a, b) - required_distance(a, b, config)
                         for a, b in combinations(result.plans, 2)))
    selection = result.diagnostics['selection']
    assert score == tuple(selection['selected_sorted_slack_m'])
    assert score >= tuple(selection['initial_sorted_slack_m'])
    assert result.search_nodes <= config.max_search_nodes
    execution = simulate(result.plans, config)
    for key in ("all_arrived", "physical_limits_verified", "tube_conditions_verified", "vertical_limits_verified",
                "separation_verified_in_execution_model", "obstacle_clearance_verified_in_execution_model",
                "approach_sectors_verified_in_execution", "descent_angle_verified_in_execution"):
        assert execution.report[key], key
