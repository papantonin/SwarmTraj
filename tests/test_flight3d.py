"""Coupled flight physics, planner integration and independent local execution."""

from dataclasses import replace

import numpy as np
import pytest
from scipy.integrate import solve_ivp

from swarm_sim.trajectory2d import Config, FlightLimits, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.dynamics import G
from swarm_sim.trajectory2d.flight3d import (air_velocity, derivative, envelope_ok,
    from_snapshot, propagate, trim_control)
from swarm_sim.trajectory2d.planning import mission_document, validate_individual
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.vertical import descent_geometry


def test_total_speed_and_thirty_degree_descent_are_coupled():
    s = np.array([0, 0, 100, 12, 0, -np.pi / 6, 0])
    v = air_velocity(s)
    assert np.linalg.norm(v) == pytest.approx(12)
    assert v[2] == pytest.approx(-6)
    assert v[0] == pytest.approx(12 * np.cos(np.pi / 6))
    c = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', target_altitude=30, descent_angle_deg=30, max_descent_rate=10)
    assert descent_geometry(12, c)[0] == pytest.approx(6)
    assert descent_geometry(12, replace(c, flight_model="legacy", generator_mode="geometric", numerical_backend="python"))[0] == pytest.approx(6.92820323)


@pytest.mark.parametrize("gamma,bank", [(0, 0), (0, 30), (15, 20), (-30, -25)])
def test_trim_obeys_lift_gravity_thrust_drag_and_turn_equations(gamma, bank):
    f = FlightLimits()
    s = np.array([0, 0, 100, 12, 0, np.deg2rad(gamma), np.deg2rad(bank)])
    u = trim_control(s, f)
    d = derivative(s, u, f)
    assert d[3] == pytest.approx(0, abs=1e-12)
    assert d[5] == pytest.approx(0, abs=1e-12)
    assert u[2] == pytest.approx(np.cos(s[5]) / np.cos(s[6]))
    assert d[4] == pytest.approx(G * np.tan(s[6]) / s[3])


def test_power_and_load_change_speed_slope_and_turn_instead_of_teleporting():
    f = FlightLimits()
    state = np.array([0, 0, 100, 12, 0, -np.pi / 6, 0])
    trim = trim_control(state, f)
    without_brake = trim.copy()
    without_brake[1] = 0
    assert derivative(state, without_brake, f)[3] > 4
    assert propagate(state, without_brake, 0.2, f)[3] > 12
    banked = state.copy()
    banked[6] = 0.4
    lift = trim_control(banked, f)
    faster_lift = lift.copy()
    faster_lift[2] += 0.1
    assert derivative(banked, faster_lift, f)[5] > derivative(banked, lift, f)[5]
    assert derivative(banked, faster_lift, f)[4] > derivative(banked, lift, f)[4]


def test_wind_changes_ground_velocity_not_airspeed_and_integrator_matches_independent_solver():
    f = FlightLimits()
    s = np.array([0, 0, 80, 12, 0.4, -0.2, 0.3])
    u = np.array([1, 0.5, 1.2, -0.15])
    wind = (2, -1, 0)
    reference = solve_ivp(lambda t, x: derivative(x, u, f, wind), (0, 0.2), s,
                          rtol=1e-12, atol=1e-12).y[:, -1]
    calculated = propagate(s, u, 0.2, f, wind)
    np.testing.assert_allclose(calculated, reference, atol=2e-7, rtol=0)
    still = propagate(s, u, 0.2, f)
    gust = propagate(s, u, 0.2, f, wind)
    np.testing.assert_allclose(gust[:3] - still[:3], np.array(wind) * 0.2, atol=1e-12)
    np.testing.assert_array_equal(gust[3:], still[3:])


@pytest.mark.parametrize("parameters", [
    {"stall_speed": 0}, {"stall_margin": 0.9}, {"max_load_factor": 0.1},
    {"min_load_factor": 0}, {"max_flight_path_deg": 90}, {"max_thrust_acceleration": 0},
    {"max_brake_acceleration": -1}, {"induced_drag": np.nan}, {"parasitic_drag": np.inf},
])
def test_invalid_flight_parameters_are_rejected(parameters):
    with pytest.raises(ValueError):
        FlightLimits(**parameters)


def test_initial_loaded_stall_and_missing_descent_brake_are_rejected_explicitly():
    c = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', target_altitude=30, cruise_altitude_range=(30, 120),
               max_descent_rate=10, descent_angle_deg=30)
    s = replace(separate_loiter_snapshot()[0], altitude=30)
    missing_brake = plan_mission((s,), replace(c, flight=replace(c.flight, max_brake_acceleration=0)))
    assert missing_brake.status == "descent_not_feasible"
    assert "airbrake" in missing_brake.reason
    slow = s.with_speed(8, c.limits)
    # A speed permitted in the old XY model can still stall a loaded wing.
    stalled = plan_mission((slow,), replace(c, descent_angle_deg=0, flight=replace(c.flight, stall_speed=8)))
    assert stalled.status == "invalid_initial_state"
    assert not stalled.plans


@pytest.fixture(scope="module")
def mission():
    config = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', target_altitude=30, cruise_altitude_range=(30, 120),
                    descent_angle_deg=30, max_descent_rate=10, horizon=120, candidates=8)
    snapshots = tuple(replace(s, altitude=30 + 25 * k) for k, s in enumerate(separate_loiter_snapshot()))
    result = plan_mission(snapshots, config)
    assert result.validated, (result.status, result.counts)
    return config, snapshots, result


def test_four_coupled_plans_reach_xyz_recover_and_obey_physical_envelope(mission):
    c, snapshots, result = mission
    for p, s in zip(result.plans, snapshots):
        assert validate_individual(p, s, c)
        assert not p.flight_states.flags.writeable
        assert not p.flight_controls.flags.writeable
        assert abs(p.sample_flight(p.arrival_time)[5] + np.pi / 6) <= np.deg2rad(1)
        np.testing.assert_allclose(p.sample_flight(0), from_snapshot(s))
        np.testing.assert_allclose(p.sample_flight(np.arange(len(p.states)) * p.dt), p.flight_states, atol=1e-9)
        np.testing.assert_allclose(p.sample(np.arange(len(p.states)) * p.dt)[:, 4:], p.vertical_states, atol=1e-9)
        assert np.min(p.flight_states[:, 2]) > c.min_altitude
        assert abs(p.flight_states[-1, 2] - p.cruise_altitude) < 0.2
        assert np.max(np.abs(np.linalg.norm(air_velocity(p.flight_states), axis=1) - p.flight_states[:, 3])) < 1e-10
    assert result.minimum_distance_lower_bound >= result.required_distance
    doc = mission_document(result, c, snapshots)
    assert doc["schema"] == "swarm-flight3d-v3"
    assert doc["plans"][0]["flight_model"] == "point_mass"


@pytest.mark.parametrize("corruption", ["state", "control", "projection", "limit", "parameters"])
def test_independent_verifier_refuses_corrupt_coupled_plans(mission, corruption):
    c, snapshots, result = mission
    p = result.plans[0]
    if corruption == "state":
        states = p.flight_states.copy()
        states[20, 2] += 1
        broken = replace(p, flight_states=states)
    elif corruption == "control":
        control = p.flight_controls.copy()
        control[20, 2] = 10
        broken = replace(p, flight_controls=control)
    elif corruption == "projection":
        states = p.states.copy()
        states[20, 0] += 1
        broken = replace(p, states=states)
    elif corruption == "limit":
        broken = replace(p, airspeed_limit=1000)
    else:
        broken = replace(p, flight_parameters=replace(c.flight, stall_speed=5))
    assert not validate_individual(broken, snapshots[0], c)


def test_local_execution_no_communication_and_small_perturbations(mission):
    c, _, result = mission
    e = simulate(result.plans, c, clock_offsets=np.linspace(-0.02, 0.02, 4), wind=0.02)
    for key in ("all_arrived", "tube_conditions_verified", "vertical_limits_verified",
                "physical_limits_verified", "separation_verified_in_execution_model",
                "descent_angle_verified_in_execution"):
        assert e.report[key], (key, e.report)
    alone = simulate((result.plans[0],), c, clock_offsets=[-0.02], wind=0.02)
    np.testing.assert_array_equal(alone.flight_actual[:, 0], e.flight_actual[:, 0])
    assert e.report["minimum_stall_margin_m_s"][0] > 0


def test_zero_error_execution_and_numerical_refinement(mission):
    c, _, result = mission
    p = result.plans[0]
    coarse = simulate((p,), c, observer_substeps=5)
    finer = simulate((p,), c, observer_substeps=10)
    assert finer.report["physical_limits_verified"]
    assert finer.report["max_sampled_tracking_errors_m"][0] < 1e-3
    np.testing.assert_allclose(coarse.flight_actual, finer.flight_actual[::2], atol=1e-8)
    # A differently partitioned RK4 must converge to the same flow.
    s, u = p.flight_states[100], p.flight_controls[100]
    np.testing.assert_allclose(propagate(s, u, 0.2, c.flight, substeps=2),
                               propagate(s, u, 0.2, c.flight, substeps=8), atol=1e-6)


def test_dense_envelope_rejects_stall_and_unbounded_actuators():
    c = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp')
    s = np.array([[0, 0, 100, 8, 0, 0, 0]])
    u = np.array([[1, 0, 2, 0]])
    assert not envelope_ok(s, u, c)
    u[0, 2] = 1
    u[0, 0] = 100
    assert not envelope_ok(s, u, c)


def test_3d_reachability_uses_total_speed_and_detects_impossible_vertical_distance():
    c = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', target=(0, 0), target_altitude=190, horizon=5)
    s = replace(separate_loiter_snapshot()[0], altitude=30)
    result = plan_mission((s,), c)
    assert result.status == "destination_unreachable_within_horizon"
    assert not result.plans


def test_coupled_generation_is_reproducible_and_layer_mode_remains_available():
    c = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', target_altitude=None, candidates=1, attempts=10)
    s = replace(separate_loiter_snapshot()[0], altitude=30)
    first = plan_mission((s,), c)
    second = plan_mission((s,), c)
    assert first.validated and second.validated
    np.testing.assert_array_equal(first.plans[0].flight_states, second.plans[0].flight_states)
    np.testing.assert_array_equal(first.plans[0].flight_controls, second.plans[0].flight_controls)
    assert validate_individual(first.plans[0], s, c)


def test_geometric_entry_allocation_with_four_coupled_simultaneous_departures():
    from swarm_sim.trajectory2d.planning import approach_angles
    config = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', target=(500, 300), target_altitude=30, horizon=150,
                    cruise_altitude_range=(40, 110), descent_angle_deg=30, max_descent_rate=10,
                    activation_time=0, max_extra_loiter=0, approach_spread_deg=180, candidates=8)
    snapshots = tuple(replace(s, altitude=30 + 25 * k) for k, s in enumerate(separate_loiter_snapshot()))
    result = plan_mission(snapshots, config)
    assert result.validated, (result.status, result.counts)
    assignments = approach_angles(snapshots, config)
    for plan, snapshot in zip(result.plans, snapshots):
        assert plan.release_time == 0
        assert plan.approach_angle == assignments[snapshot.drone_id]
        assert validate_individual(plan, snapshot, config)
    evaluation = simulate(result.plans, config)
    for key in ("all_arrived", "physical_limits_verified", "approach_sectors_verified_in_execution",
                "separation_verified_in_execution_model", "tube_conditions_verified"):
        assert evaluation.report[key], (key, evaluation.report)
    assert evaluation.report["approach_assignment"] == "geometric"
