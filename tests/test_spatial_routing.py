"""Altitude proposals are bounded heuristics, never a replacement for validation."""

from dataclasses import replace

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, Obstacle, Snapshot, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.obstacles import avoidance_waypoints
from swarm_sim.trajectory2d.planning import validate_individual
from swarm_sim.trajectory2d.routing3d import spatial_cruise_proposal, spatial_waypoints


def scene(obstacle):
    start = np.array([-200., 0])
    waypoints = [np.array(p, dtype=float) for p in [(-80, 0), (100, 0), (300, 0), (400, 0), (500, 0)]]
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', obstacle_routing='spatial', cruise_altitude_range=(50, 140),
                    obstacles=(obstacle,))
    snapshot = Snapshot(0, (*start, 0, 0), 12, altitude=50)
    return start, waypoints, config, snapshot


def test_overflight_selects_roof_clearance_and_shorter_cruise_prefix():
    start, points, config, snapshot = scene(Obstacle.box(-20, -500, 0, 20, 500, 60))
    mode, cruise = spatial_cruise_proposal(snapshot, config, 12, 1, start, points)
    assert mode == 'overflight' and 70 < cruise < 90
    assert config.cruise_altitude_range[0] <= cruise <= config.cruise_altitude_range[1]
    spatial = spatial_waypoints(start, points, config, 12, 1, cruise)
    lateral = avoidance_waypoints(start, points, config, 12, 1, optional_count=2)
    def length(route):
        return np.linalg.norm(np.diff([start, *route], axis=0), axis=1).sum()
    assert length(spatial) < length(lateral)
    np.testing.assert_equal(spatial, spatial_waypoints(start, points, config, 12, 1, cruise))


def test_underflight_respects_suspended_base_and_manual_level():
    start, points, config, snapshot = scene(Obstacle.box(-20, -500, 100, 20, 500, 180))
    mode, cruise = spatial_cruise_proposal(snapshot, config, 12, 2, start, points)
    assert mode == 'underflight' and cruise == 50
    manual = replace(config, cruise_altitude_range=None)
    assert spatial_cruise_proposal(snapshot, manual, 12, 2, start, points) == ('underflight', 50)


def test_impossible_vertical_options_fall_back_without_widening_cruise_range():
    start, points, config, snapshot = scene(Obstacle.box(-20, -500, 0, 20, 500, 180))
    for candidate_id in (0, 1, 2):
        assert spatial_cruise_proposal(snapshot, config, 12, candidate_id, start, points) == ('lateral', None)
    for changed in (replace(config, dimensions=2, cruise_altitude_range=None),
                    replace(config, obstacle_routing='lateral'), replace(config, obstacles=())):
        assert spatial_cruise_proposal(snapshot, changed, 12, 1, start, points) == ('lateral', None)


def test_mixed_route_retains_lateral_blockers_at_selected_altitude():
    low = Obstacle.box(-20, -500, 0, 20, 500, 60)
    high = Obstacle.box(170, -30, 0, 230, 30, 200)
    start, points, config, _ = scene(low)
    config = replace(config, obstacles=(low, high))
    route = spatial_waypoints(start, points, config, 12, 1, 80)
    for a, b in zip([start, *route[:-1]], route):
        samples = a + np.linspace(0, 1, 100)[:, None] * (b - a)
        assert high.distance(samples).min() > config.obstacle_clearance


@pytest.fixture(scope='module', params=['overflight', 'underflight'])
def four_aircraft_spatial_mission(request):
    mode = request.param
    obstacle = (Obstacle.box(220, -500, 0, 260, 500, 60) if mode == 'overflight' else
                Obstacle.box(220, -500, 180, 260, 500, 250))
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target=(600, 0), target_altitude=80, horizon=120,
                    activation_time=0, max_extra_loiter=0, cruise_altitude_range=(80, 170),
                    max_descent_rate=10, obstacle_routing='spatial', obstacles=(obstacle,),
                    planning_strategy='adaptive', search_order='mrv', candidates=8, attempts=24,
                    selection_policy='max_margin', margin_expansion_rounds=0)
    snapshots = tuple(replace(s, altitude=80 + 25*i) for i, s in enumerate(separate_loiter_snapshot(4)))
    result = plan_mission(snapshots, config)
    assert result.validated, (result.status, result.diagnostics)
    return mode, config, snapshots, result


def test_four_aircraft_explore_vertical_routes_and_pass_all_execution_checks(four_aircraft_spatial_mission):
    mode, config, snapshots, result = four_aircraft_spatial_mission
    assert all(c['accepted_routing_modes'].get(mode, 0) > 0 for c in result.diagnostics['catalogs'])
    for plan, snapshot in zip(result.plans, snapshots):
        assert validate_individual(plan, snapshot, config)
        # Check that actual integrated paths use the solid's XY footprint.
        overlap = config.obstacles[0].distance(plan.positions[:, :2]) < 0
        assert np.any(overlap)
        z = plan.positions[overlap, 2]
        assert np.all(z > 60) if mode == 'overflight' else np.all(z < 180)
    report = simulate(result.plans, config).report
    for key in ('all_arrived', 'tube_conditions_verified', 'separation_verified_in_execution_model',
                'physical_limits_verified', 'vertical_limits_verified', 'approach_sectors_verified_in_execution',
                'descent_angle_verified_in_execution', 'obstacle_clearance_verified_in_execution_model',
                'arrival_deadline_verified_in_execution', 'post_arrival_time_verified_in_execution'):
        assert report[key], key


def test_spatial_catalogs_solve_cases_that_lateral_catalogs_do_not(four_aircraft_spatial_mission):
    _, config, snapshots, result = four_aircraft_spatial_mission
    lateral = plan_mission(snapshots, replace(config, obstacle_routing='lateral'))
    assert not lateral.validated
    assert lateral.status in ('no_candidate', 'catalog_exhausted', 'search_budget_exhausted')
    assert result.validated  # Failure of the lateral finite catalog is not an impossibility proof.


def test_vertical_routing_is_spawn_reproducible(four_aircraft_spatial_mission):
    _, config, snapshots, result = four_aircraft_spatial_mission
    parallel = plan_mission(snapshots[::-1], replace(config, planning_workers=2))
    assert parallel.validated
    assert parallel.diagnostics == result.diagnostics
    assert [p.to_dict() for p in parallel.plans] == [p.to_dict() for p in result.plans]
    assert all(not p.flight_states.flags.writeable for p in parallel.plans)


def test_new_vertical_route_cannot_bypass_independent_obstacle_validation(four_aircraft_spatial_mission):
    _, config, snapshots, result = four_aircraft_spatial_mission
    changed = replace(config, obstacles=(Obstacle.box(220, -500, 0, 260, 500, 200),))
    for snapshot, plan in zip(snapshots, result.plans):
        diagnostic = {}
        assert not validate_individual(plan, snapshot, changed, diagnostic)
        assert diagnostic['rejection'] == 'obstacle_clearance'
