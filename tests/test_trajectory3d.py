from dataclasses import replace

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, Trajectory, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.dynamics import propagate
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.planning import continuous_distance_lower_bound, validate_individual
from swarm_sim.trajectory2d.vertical import transition


def layered_snapshot(cruise_offset=0):
    return tuple(replace(s, altitude=30 + 25 * k, cruise_altitude=30 + 25 * k + cruise_offset)
                 for k, s in enumerate(separate_loiter_snapshot()))


@pytest.mark.parametrize("displacement", [0, 0.01, 2, -25, 75])
def test_vertical_transition_is_continuous_rest_to_rest_and_bounded(displacement):
    dt = 0.2
    az = transition(displacement, dt, 3, 1)
    vz = np.r_[0, np.cumsum(az) * dt]
    z = np.r_[0, np.cumsum(vz[:-1] * dt + 0.5 * az * dt**2)]
    assert z[-1] == pytest.approx(displacement, abs=1e-10)
    assert vz[-1] == pytest.approx(0, abs=1e-10)
    assert np.max(np.abs(vz)) <= 3 + 1e-10
    if len(az):
        assert np.max(np.abs(az)) <= 1 + 1e-10
    assert z.min() >= min(0, displacement) - 1e-10
    assert z.max() <= max(0, displacement) + 1e-10


def test_layers_resolve_recorded_2d_conflict_without_delaying_departures():
    # User's recorded failure: target (500,250), speeds 16/16/16/12, no delay.
    snapshots = tuple(s.with_speed(speed, Config().limits) for s, speed in
                      zip(layered_snapshot(), [16, 16, 16, 12]))
    config = Config(dimensions=3, flight_model="legacy", target=(500, 250), activation_time=0,
                    max_extra_loiter=0, approach_spread_deg=180)
    result = plan_mission(snapshots, config)
    assert result.validated
    assert all(p.release_time == 0 for p in result.plans)
    for snapshot, plan in zip(snapshots, result.plans):
        assert validate_individual(plan, snapshot, config)
        np.testing.assert_allclose(plan.sample(np.arange(len(plan.states)) * plan.dt)[:, 4:],
                                   plan.vertical_states, atol=1e-8)
        assert not plan.vertical_states.flags.writeable
    # Projecting these same selected paths back to 2D exposes a conflict.
    projected = tuple(replace(p, vertical_states=None, vertical_accelerations=None) for p in result.plans)
    assert any(continuous_distance_lower_bound(a, b) < config.separation
               for i, a in enumerate(projected) for b in projected[i + 1:])
    evaluation = simulate(result.plans, config)
    assert evaluation.report["all_arrived"]
    assert evaluation.report["separation_verified_in_execution_model"]
    assert evaluation.report["tube_conditions_verified"]
    assert evaluation.report["vertical_limits_verified"]
    assert evaluation.report["minimum_sampled_distance_m"] > 25
    alone = simulate((result.plans[0],), config)
    np.testing.assert_array_equal(alone.actual[:, 0], evaluation.actual[:, 0])


def test_common_altitude_requires_real_climb_descent_and_safe_exit():
    snapshots = layered_snapshot(cruise_offset=10)
    config = Config(dimensions=3, flight_model="legacy", target=(500, 250), target_altitude=30, horizon=150,
                    approach_spread_deg=180)
    result = plan_mission(snapshots, config)
    assert result.validated, (result.status, result.counts)
    for snapshot, plan in zip(snapshots, result.plans):
        assert validate_individual(plan, snapshot, config)
        assert plan.sample(plan.arrival_time)[4] == pytest.approx(30, abs=1e-8)
        np.testing.assert_allclose(plan.vertical_states[-1], [snapshot.cruise_altitude, 0], atol=1e-8)
        assert plan.vertical_states[:, 0].max() == pytest.approx(snapshot.cruise_altitude, abs=1e-8)
    evaluation = simulate(result.plans, config, clock_offsets=np.linspace(-0.05, 0.05, 4), wind=0.05)
    assert evaluation.report["all_arrived"]
    assert evaluation.report["tube_conditions_verified"]
    assert evaluation.report["vertical_limits_verified"]
    assert evaluation.report["separation_verified_in_execution_model"]


def test_vertical_crossing_between_samples_is_not_certified():
    state = np.array([-10, 0, 0, 0])
    states = np.stack([state, propagate(state, 20, 0, 1)])
    a = Trajectory(0, 0, 20, 1, states, np.zeros(1), 1, 0,
                   vertical_states=np.array([[30, 20], [50, 20]]), vertical_accelerations=np.zeros(1))
    b = replace(a, drone_id=1, vertical_states=np.array([[50, -20], [30, -20]]))
    assert np.linalg.norm(a.positions - b.positions, axis=1).min() == 20
    np.testing.assert_allclose(a.sample(0.5)[[0, 1, 4]], b.sample(0.5)[[0, 1, 4]])
    assert continuous_distance_lower_bound(a, b) < 8


def test_altitudes_are_not_teleported_and_invalid_profiles_are_rejected():
    snapshots = layered_snapshot(cruise_offset=10)[:1]
    config = Config(dimensions=3, flight_model="legacy", activation_time=3, max_extra_loiter=0, candidates=1)
    result = plan_mission(snapshots, config)
    assert result.validated
    plan = result.plans[0]
    assert plan.sample(2.9)[4] == snapshots[0].altitude
    assert plan.sample(3.1)[4] - snapshots[0].altitude <= 0.5 * config.max_vertical_acceleration * 0.1**2 + 1e-8
    broken = plan.vertical_states.copy()
    broken[2, 0] += 10
    assert not validate_individual(replace(plan, vertical_states=broken), snapshots[0], config)
    bad_initial = (replace(snapshots[0], altitude=-1),)
    assert plan_mission(bad_initial, config).status == "invalid_initial_state"
    unreachable = (replace(snapshots[0], cruise_altitude=190),)
    refused = plan_mission(unreachable, replace(config, max_climb_rate=0.1, attempts=2))
    assert refused.status == "no_candidate"
    assert not refused.plans


@pytest.mark.parametrize("kwargs", [{"max_climb_rate": 0}, {"max_vertical_acceleration": -1},
                                   {"target_altitude": 500}, {"max_altitude": np.inf}])
def test_invalid_vertical_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        Config(dimensions=3, **kwargs)


def test_recorded_common_xyz_failure_is_resolved_with_simultaneous_departures():
    positions = [(-63.73387096774195, 281.0672043010752),
                 (114.78494623655911, 249.1532258064516),
                 (-102.629032258, -53.0322580645), (40, 15)]
    snapshots = tuple(replace(s, state=(*xy, *s.state[2:]))
                      for s, xy in zip(layered_snapshot(), positions))
    # Replay the recorded historical scenario with its original ID allocation.
    config = Config(dimensions=3, flight_model="legacy", approach_assignment="id", target=(500, 300), target_altitude=30,
                    activation_time=0, max_extra_loiter=0, approach_spread_deg=180)
    result = plan_mission(snapshots, config)
    assert result.validated, (result.status, result.counts)
    assert result.search_nodes < 1000
    for s, p in zip(snapshots, result.plans):
        assert p.release_time == 0
        assert validate_individual(p, s, config)
        assert p.sample(p.arrival_time)[4] == pytest.approx(30, abs=1e-8)
    arrivals = sorted(p.arrival_time for p in result.plans)
    assert min(np.diff(arrivals)) > 1
    evaluation = simulate(result.plans, config, clock_offsets=np.linspace(-0.05, 0.05, 4), wind=0.05)
    assert evaluation.report["arrival_mode"] == "common_xyz"
    assert evaluation.report["all_arrived"]
    assert evaluation.report["vertical_limits_verified"]
    assert evaluation.report["tube_conditions_verified"]
    assert evaluation.report["separation_verified_in_execution_model"]


def test_shared_cruise_range_is_selected_jointly_and_reaches_common_xyz():
    snapshots = tuple(replace(s, cruise_altitude=-100) for s in layered_snapshot())
    # Old per-aircraft cruise settings are irrelevant in automatic mode.
    config = Config(dimensions=3, flight_model="legacy", approach_assignment="id", target=(500, 300), target_altitude=30,
                    activation_time=0, max_extra_loiter=0, approach_spread_deg=180,
                    cruise_altitude_range=(40, 110))
    result = plan_mission(snapshots, config)
    assert result.validated, (result.status, result.counts)
    for snapshot, plan in zip(snapshots, result.plans):
        assert 40 <= plan.cruise_altitude <= 110
        assert validate_individual(plan, snapshot, config)
        assert plan.sample(0)[4] == snapshot.altitude
        assert plan.sample(plan.arrival_time)[4] == pytest.approx(30, abs=1e-8)
        assert plan.vertical_states[-1, 0] == pytest.approx(plan.cruise_altitude, abs=1e-8)
        assert not validate_individual(replace(plan, cruise_altitude=130), snapshot, config)
    assert result.plans[0].cruise_altitude != snapshots[0].altitude
    again = plan_mission(tuple(reversed(snapshots)), config)
    assert again.validated
    for first, second in zip(result.plans, again.plans):
        assert first.cruise_altitude == second.cruise_altitude
        np.testing.assert_array_equal(first.vertical_states, second.vertical_states)
        np.testing.assert_array_equal(first.states, second.states)
    evaluation = simulate(result.plans, config, clock_offsets=np.linspace(-0.05, 0.05, 4), wind=0.05)
    assert evaluation.report["all_arrived"]
    assert evaluation.report["tube_conditions_verified"]
    assert evaluation.report["separation_verified_in_execution_model"]
    assert evaluation.report["vertical_limits_verified"]
    assert evaluation.report["selected_cruise_altitudes_m"] == [p.cruise_altitude for p in result.plans]


@pytest.mark.parametrize("bounds", [(80, 40), (-1, 100), (30, 201), (0, np.inf), (np.nan, 100), (30,)])
def test_invalid_cruise_range_is_rejected(bounds):
    with pytest.raises(ValueError):
        Config(dimensions=3, cruise_altitude_range=bounds)


def test_fixed_shared_cruise_level_is_allowed():
    config = Config(dimensions=3, flight_model="legacy", target=(500, 300), target_altitude=30,
                    horizon=150, cruise_altitude_range=(60, 60), candidates=1)
    snapshots = layered_snapshot()[:1]
    result = plan_mission(snapshots, config)
    assert result.validated
    assert result.plans[0].cruise_altitude == 60
    assert validate_individual(result.plans[0], snapshots[0], config)
