from dataclasses import replace
import json

import numpy as np
import pytest
from scipy.integrate import quad

from swarm_sim.trajectory2d import (
    Config, Limits, Snapshot, Trajectory, plan_mission, ring_snapshot, separate_loiter_snapshot,
)
from swarm_sim.trajectory2d.dynamics import holding_circle, propagate
from swarm_sim.trajectory2d.execution import Executor, simulate
from swarm_sim.trajectory2d.planning import (
    approach_angles, continuous_distance_lower_bound, mission_document, required_distance, validate_individual,
)


@pytest.fixture(scope="module")
def mission():
    config = Config()
    snapshots = separate_loiter_snapshot()
    result = plan_mission(snapshots, config)
    assert result.validated
    return config, snapshots, result


@pytest.mark.parametrize("count", [1, 4, 5])
def test_separate_holding_circles(count):
    snapshots = separate_loiter_snapshot(count)
    centres = []
    for snapshot in snapshots:
        centre, radius = holding_circle(snapshot.state, snapshot.speed)
        centres.append(centre)
        assert radius == pytest.approx(25)
        times = np.linspace(0, 2 * np.pi * radius / snapshot.speed, 101)
        states = propagate(snapshot.state, snapshot.speed, 0, times)
        np.testing.assert_allclose(np.linalg.norm(states[:, :2] - centre, axis=1), radius)
        np.testing.assert_allclose(states[-1], snapshot.state, atol=1e-12)
    np.testing.assert_allclose(np.mean(centres, axis=0), 0, atol=1e-12)
    for i, centre in enumerate(centres):
        for other in centres[i + 1:]:
            # Any points on these circles are at least 30 m apart, regardless of phase.
            assert np.linalg.norm(centre - other) - 2 * radius >= 30 - 1e-12


@pytest.mark.parametrize("kwargs", [
    {"count": 0}, {"count": 2.5}, {"radius": 0}, {"speed": -1},
    {"spacing": 50}, {"spacing": 40}, {"spacing": np.inf}, {"radius": np.nan},
])
def test_invalid_separate_loiter_layout_is_rejected(kwargs):
    with pytest.raises(ValueError):
        separate_loiter_snapshot(**kwargs)


def test_plans_stay_on_own_circle_until_release(mission):
    _, snapshots, result = mission
    for snapshot, plan in zip(snapshots, result.plans):
        centre, radius = holding_circle(snapshot.state, snapshot.speed)
        times = np.linspace(0, plan.release_time, 101)
        np.testing.assert_allclose(np.linalg.norm(plan.sample(times)[:, :2] - centre, axis=1),
                                   radius, atol=1e-8)


def test_straight_motion_has_no_holding_circle():
    assert holding_circle((0, 0, 0, 0), 12) is None


def test_speed_configuration_checks_turn_feasibility():
    snapshot = separate_loiter_snapshot(radius=15)[0]
    with pytest.raises(ValueError, match="radius"):
        snapshot.with_speed(16, Limits())
    clockwise = Snapshot(0, (0, 0, 0, -12 / 25), 12)
    assert clockwise.with_speed(10, Limits()).state[3] == pytest.approx(-10 / 25)


@pytest.mark.parametrize("alpha", [0.0, 0.6, -0.7, 1e-5])
def test_motion_integral_matches_independent_quadrature(alpha):
    initial = np.array([2, -4, 0.7, -0.3])
    dt, speed = 0.4, 12
    state = propagate(initial, speed, alpha, dt)
    phase = lambda t: initial[2] + initial[3] * t + 0.5 * alpha * t**2
    expected = initial[:2] + speed * np.array([
        quad(lambda t: np.cos(phase(t)), 0, dt)[0],
        quad(lambda t: np.sin(phase(t)), 0, dt)[0],
    ])
    np.testing.assert_allclose(state[:2], expected, atol=2e-9, rtol=0)


def test_same_snapshot_seed_and_order_generate_same_plan(mission):
    config, snapshots, first = mission
    second = plan_mission(tuple(reversed(snapshots)), config)
    for a, b in zip(first.plans, second.plans):
        np.testing.assert_array_equal(a.states, b.states)
        assert a.candidate_id == b.candidate_id
    first_doc = mission_document(first, config, snapshots)
    # Serialized mission is bitwise reproducible for identical inputs/versions.
    assert json.dumps(first_doc) == json.dumps(mission_document(second, config, snapshots))


def test_different_seed_changes_trajectory(mission):
    config, snapshots, first = mission
    second = plan_mission(snapshots, replace(config, seed=19))
    assert second.validated
    assert any(not np.array_equal(a.states, b.states) for a, b in zip(first.plans, second.plans))


def test_all_primitives_and_all_pairs_are_certified(mission):
    config, snapshots, result = mission
    for plan, snapshot in zip(result.plans, snapshots):
        assert validate_individual(plan, snapshot, config)
        assert not plan.states.flags.writeable
        np.testing.assert_allclose(plan.sample(np.arange(len(plan.states)) * config.dt),
                                   plan.states, atol=1e-8)
        # Arrived drones remain in the model and move away, not disappear.
        assert np.linalg.norm(plan.states[-1, :2] - config.target) > 30
    for i, a in enumerate(result.plans):
        for b in result.plans[i+1:]:
            assert continuous_distance_lower_bound(a, b) >= required_distance(a, b, config)


def test_between_sample_collision_is_not_missed():
    a0 = np.array([-10.0, 0, 0, 0])
    b0 = np.array([0, -10.0, np.pi/2, 0])
    def trajectory(drone_id, initial):
        states = np.stack([initial, propagate(initial, 20, 0, 1)])
        return Trajectory(drone_id, 0, 20, 1, states, np.zeros(1), 1, 0)
    a, b = trajectory(0, a0), trajectory(1, b0)
    assert np.linalg.norm(a.states[:, :2] - b.states[:, :2], axis=1).min() > 8
    assert np.linalg.norm(a.sample(.5)[:2] - b.sample(.5)[:2]) < 1e-10
    assert continuous_distance_lower_bound(a, b) < 8


def test_nominal_execution_arrives_without_other_drone_inputs(mission):
    config, _, result = mission
    evaluation = simulate(result.plans, config)
    assert evaluation.report["all_arrived"]
    assert evaluation.report["tube_conditions_verified"]
    assert evaluation.report["separation_verified_in_execution_model"]
    np.testing.assert_allclose(evaluation.actual, evaluation.nominal, atol=1e-8)
    # Executing drone zero alone has the same outcome: no hidden swarm feedback.
    alone = simulate((result.plans[0],), config)
    np.testing.assert_array_equal(alone.actual[:, 0], evaluation.actual[:, 0])
    local = Executor(result.plans[0], config.limits)
    assert not hasattr(local, "plans") and not hasattr(local, "other_drones")


def test_bounded_clock_error_and_wind_remain_inside_tubes(mission):
    config, _, result = mission
    evaluation = simulate(result.plans, config,
                          clock_offsets=np.linspace(-.1, .1, 4), wind=.1)
    assert evaluation.report["all_arrived"]
    assert evaluation.report["tube_conditions_verified"]
    assert evaluation.report["separation_verified_in_execution_model"]


def test_excess_clock_error_invalidates_assumptions(mission):
    config, _, result = mission
    evaluation = simulate(result.plans, config, clock_offsets=[1, 0, 0, 0])
    assert not evaluation.report["tube_conditions_verified"]


def test_impossible_initial_separation_is_rejected():
    initial = (Snapshot(0, (0, 0, 0, 0), 12), Snapshot(1, (1, 0, 0, 0), 12))
    assert plan_mission(initial).status == "initial_separation_violation"


def test_unreachable_destination_has_a_model_based_reason():
    result = plan_mission(ring_snapshot(), Config(target=(10000, 0)))
    assert result.status == "destination_unreachable_within_horizon"
    assert not result.plans


def test_budget_exhaustion_is_not_declared_impossible():
    result = plan_mission(ring_snapshot(), Config(candidates=2, attempts=2, max_search_nodes=1))
    assert result.status == "search_budget_exhausted"


def test_nonfinite_inputs_and_duplicate_ids_are_rejected():
    with pytest.raises(ValueError):
        Config(target=(np.nan, 0))
    with pytest.raises(ValueError):
        plan_mission((ring_snapshot()[0], ring_snapshot()[0]))


@pytest.mark.parametrize("spread", [180, 360])
def test_requested_arrival_sides_are_reached_and_jointly_safe(spread):
    snapshots = separate_loiter_snapshot()
    config = Config(approach_spread_deg=spread, horizon=120)
    result = plan_mission(snapshots, config)
    assert result.validated, (result.status, result.counts)
    assignments = approach_angles(snapshots, config)
    assert assignments == approach_angles(tuple(reversed(snapshots)), config)
    for snapshot, plan in zip(snapshots, result.plans):
        assert plan.approach_angle == assignments[snapshot.drone_id]
        assert validate_individual(plan, snapshot, config)
        # Changing only the claimed entry side must fail the independent check.
        wrong_side = replace(plan, approach_angle=plan.approach_angle + np.pi)
        assert not validate_individual(wrong_side, snapshot, config)
    for i, first in enumerate(result.plans):
        for second in result.plans[i + 1:]:
            assert continuous_distance_lower_bound(first, second) >= required_distance(first, second, config)
    evaluation = simulate(result.plans, config)
    assert evaluation.report["all_arrived"]
    assert evaluation.report["approach_sectors_verified_in_execution"]
    assert evaluation.report["separation_verified_in_execution_model"]
    bearings = np.mod(evaluation.report["actual_contact_bearings_deg"], 360)
    if spread == 360:
        gaps = np.diff(np.r_[np.sort(bearings), np.sort(bearings)[0] + 360])
        assert np.all((gaps > 60) & (gaps < 120))


@pytest.mark.parametrize("spread", [-1, 361, np.inf, np.nan])
def test_invalid_approach_spread_is_rejected(spread):
    with pytest.raises(ValueError, match="approach_spread"):
        Config(approach_spread_deg=spread)


@pytest.mark.parametrize("interval,activation", [((0, 0), 0), ((0, 0), 3), ((1, 3), 3)])
def test_departure_interval_is_respected_without_relaxing_safety(interval, activation):
    config = Config(min_extra_loiter=interval[0], max_extra_loiter=interval[1],
                    activation_time=activation)
    snapshots = separate_loiter_snapshot()
    result = plan_mission(snapshots, config)
    assert result.validated, (result.status, result.counts)
    for snapshot, plan in zip(snapshots, result.plans):
        assert activation + interval[0] <= plan.release_time <= activation + interval[1]
        assert validate_individual(plan, snapshot, config)
        assert not validate_individual(replace(plan, release_time=activation + interval[1] + 1),
                                       snapshot, config)
    for i, first in enumerate(result.plans):
        for second in result.plans[i + 1:]:
            assert continuous_distance_lower_bound(first, second) >= required_distance(first, second, config)


@pytest.mark.parametrize("minimum,maximum", [(3, 2), (-1, 2), (0, -1), (np.nan, 2), (0, np.inf)])
def test_invalid_departure_interval_is_rejected(minimum, maximum):
    with pytest.raises(ValueError):
        Config(min_extra_loiter=minimum, max_extra_loiter=maximum)


def test_simultaneous_conflicting_routes_are_rejected_without_adding_delay():
    snapshots = (Snapshot(0, (-100, 0, 0, 0), 12),
                 Snapshot(1, (100, 0, -np.pi, 0), 12))
    config = Config(target=(0, 0), activation_time=0, min_extra_loiter=0,
                    max_extra_loiter=0, detour=0, candidates=1, attempts=1)
    result = plan_mission(snapshots, config)
    assert result.status == "catalog_exhausted"
    assert result.counts == (1, 1)
    assert not result.plans
