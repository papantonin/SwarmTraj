"""Four-aircraft integrated checks for contact, moving preparation and reservations."""

from dataclasses import replace
from itertools import combinations
import numpy as np

from swarm_sim.trajectory2d import Config, Preparation, prepare_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.obstacles import load_obstacles
from swarm_sim.trajectory2d.reservations import occupancy, first_conflict


def test_four_aircraft_preparation_reservations_and_contact_events_are_reproducible():
    config = Config(dimensions=3, target=(500, 500), target_altitude=30, horizon=210,
                    activation_time=30, descent_angle_deg=30, max_descent_rate=10,
                    cruise_altitude_range=(30, 120), planning_strategy='adaptive', search_order='mrv',
                    initial_candidates=2, selection_policy='max_margin', conflict_guidance='witness',
                    obstacle_routing='spatial', reservation_policy='grid',
                    obstacles=load_obstacles('examples/scenarios/static_obstacles.json'))
    snapshots = tuple(replace(s, altitude=30 + 25*k) for k, s in enumerate(separate_loiter_snapshot(4)))
    preparation = Preparation(simulated_planning_s=5)
    result = prepare_mission(snapshots, config, preparation)
    parallel = prepare_mission(snapshots[::-1], replace(config, planning_workers=2), preparation)
    assert result.validated and parallel.validated
    assert result.diagnostics == parallel.diagnostics
    assert [p.to_dict() for p in result.plans] == [p.to_dict() for p in parallel.plans]
    assert result.diagnostics['preparation']['all_ready']
    assert result.diagnostics['reservations']['full_horizon_verified']
    allocations = [occupancy(p, config) for p in result.plans]
    assert all(first_conflict(a, b) is None for a, b in combinations(allocations, 2))
    for p, snapshot, readiness in zip(result.plans, snapshots, result.diagnostics['preparation']['aircraft']):
        assert not np.allclose(p.sample(10)[:2], snapshot.state[:2])
        np.testing.assert_allclose(p.sample(config.activation_time)[:2],
                                   readiness['predicted_activation_state'][:2], atol=1e-4)
        assert not p.flight_states.flags.writeable
    evaluation = simulate(result.plans, config)
    assert evaluation.report['execution_validated'], evaluation.report['execution_failures']
    assert evaluation.report['reservation_adherence_verified']
    # Dense control reconstruction localizes contact independently of observer dt.
    alone = simulate((result.plans[0],), config, observer_substeps=5)
    assert abs(alone.report['arrival_times_s'][0] - evaluation.report['arrival_times_s'][0]) < 2e-6
    assert abs(alone.report['actual_descent_angles_deg'][0] - evaluation.report['actual_descent_angles_deg'][0]) < 1e-5
