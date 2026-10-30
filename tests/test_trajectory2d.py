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


















@pytest.mark.parametrize("spread", [-1, 361, np.inf, np.nan])
def test_invalid_approach_spread_is_rejected(spread):
    with pytest.raises(ValueError, match="approach_spread"):
        Config(approach_spread_deg=spread)




@pytest.mark.parametrize("minimum,maximum", [(3, 2), (-1, 2), (0, -1), (np.nan, 2), (0, np.inf)])
def test_invalid_departure_interval_is_rejected(minimum, maximum):
    with pytest.raises(ValueError):
        Config(min_extra_loiter=minimum, max_extra_loiter=maximum)
