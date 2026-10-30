"""Native vertical proposals and full integrated/joint checks are separate."""
from dataclasses import replace
import numpy as np
import pytest
from swarm_sim.trajectory2d import Config, Obstacle, Snapshot, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.planning import _candidate, validate_individual


@pytest.mark.parametrize('mode,candidate', [('overflight', 1), ('underflight', 2)])
def test_native_vertical_proposal_crosses_the_actual_footprint_safely(mode, candidate):
    obstacle = (Obstacle.box(220, -500, 0, 260, 500, 60) if mode == 'overflight'
                else Obstacle.box(220, -500, 180, 260, 500, 250))
    config = Config(dimensions=3, target=(600., 0.), target_altitude=80., horizon=120.,
        activation_time=0., max_extra_loiter=0., cruise_altitude_range=(80., 80.),
        max_descent_rate=10., obstacle_routing='spatial', obstacles=(obstacle,))
    snapshot = Snapshot(0, (0., 0., 0., 0.), 12., 80., 80.)
    plan = _candidate(snapshot, config, candidate, np.random.default_rng(7))
    assert plan is not None and plan.routing_mode == mode
    assert validate_individual(plan, snapshot, config)
    overlap = obstacle.distance(plan.positions[:, :2]) < 0
    assert np.any(overlap)
    altitude = plan.positions[overlap, 2]
    assert np.all(altitude > 60.) if mode == 'overflight' else np.all(altitude < 180.)
    assert simulate((plan,), config).report['execution_validated']


@pytest.fixture(scope='module', params=['overflight', 'underflight'])
def four_aircraft_spatial_mission(request):
    obstacle = (Obstacle.box(220, -500, 0, 260, 500, 60) if request.param == 'overflight'
                else Obstacle.box(220, -500, 180, 260, 500, 250))
    config = Config(dimensions=3, target=(600, 0), target_altitude=80, horizon=120,
        activation_time=0, max_extra_loiter=0, cruise_altitude_range=(80, 170),
        max_descent_rate=10, obstacle_routing='spatial', obstacles=(obstacle,),
        planning_strategy='adaptive', search_order='mrv', candidates=8, attempts=24,
        selection_policy='max_margin', margin_expansion_rounds=0)
    snapshots = tuple(replace(s, altitude=80+25*i) for i,s in enumerate(separate_loiter_snapshot(4)))
    result = plan_mission(snapshots, config)
    assert result.validated, (result.status, result.diagnostics)
    return config, snapshots, result


def test_joint_selection_validates_whichever_route_family_it_selects(four_aircraft_spatial_mission):
    config, snapshots, result = four_aircraft_spatial_mission
    # Spatial proposals are a heuristic, never a request to force every selected
    # aircraft over/under a solid or a guarantee that lateral planning must fail.
    for plan, snapshot in zip(result.plans, snapshots):
        assert validate_individual(plan, snapshot, config)
    report = simulate(result.plans, config).report
    assert report['execution_validated'], report['execution_failures']
    assert report['obstacle_clearance_verified_in_execution_model']


def test_vertical_catalogs_are_spawn_reproducible(four_aircraft_spatial_mission):
    config, snapshots, result = four_aircraft_spatial_mission
    parallel = plan_mission(snapshots[::-1], replace(config, planning_workers=2))
    assert parallel.validated and parallel.diagnostics == result.diagnostics
    assert [p.to_dict() for p in parallel.plans] == [p.to_dict() for p in result.plans]
    assert all(not p.flight_states.flags.writeable for p in parallel.plans)


def test_post_arrival_obstacle_is_rejected_for_every_selected_route(four_aircraft_spatial_mission):
    config, snapshots, result = four_aircraft_spatial_mission
    for snapshot, plan in zip(snapshots, result.plans):
        x,y,z = plan.flight_states[-1,:3]
        changed = replace(config, obstacles=(Obstacle.box(x-2,y-2,z-2,x+2,y+2,z+2),))
        diagnostic = {}
        assert not validate_individual(plan, snapshot, changed, diagnostic)
        assert diagnostic['rejection'] == 'obstacle_clearance'
