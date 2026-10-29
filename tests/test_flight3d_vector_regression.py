"""Exact compatibility of dense RK4 evaluations with the historical backend."""
import numpy as np
import pytest

from swarm_sim.trajectory2d import Config
from swarm_sim.trajectory2d.dynamics import G
import swarm_sim.trajectory2d.flight3d as flight


def historical_derivative(state, control, parameters, wind=(0., 0., 0.)):
    """Frozen pre-optimization broadcast implementation, for exact regressions."""
    s, u = np.asarray(state), np.asarray(control)
    v, gamma, bank = s[..., 3], s[..., 5], s[..., 6]
    thrust, brake, load, roll = np.moveaxis(u, -1, 0)
    velocity = flight.air_velocity(s) + np.asarray(wind)
    return np.concatenate((velocity, np.stack((
        thrust - brake - parameters.drag(v, load) - G * np.sin(gamma),
        G * load * np.sin(bank) / (v * np.cos(gamma)),
        G * (load * np.cos(bank) - np.cos(gamma)) / v,
        np.broadcast_to(roll, np.shape(v)),
    ), axis=-1)), axis=-1)


@pytest.mark.parametrize('shape', [(5,), (3, 5), (100,)])
@pytest.mark.parametrize('held_control', [False, True])
def test_dense_rollouts_and_envelope_classification_are_bit_identical(monkeypatch, shape, held_control):
    rng = np.random.default_rng(731)
    config = Config(dimensions=3)
    states = rng.uniform([-200., -200., 20., 8., -3., -.65, -.8],
                         [200., 200., 180., 16., 3., .65, .8], (*shape, 7))
    controls = rng.uniform([0., 0., .2, -.5], [5., 5., 2.5, .5], (4,) if held_control else (*shape, 4))
    wind = np.array([.1, -.2, 0.])
    durations = np.broadcast_to(np.linspace(0., .2, shape[-1]), shape)
    optimized = flight.derivative(states, controls, config.flight, wind)
    rolled = flight.propagate(states, controls, durations, config.flight, wind=wind, substeps=4)
    classifications = [flight.envelope_ok(s, controls if held_control else controls.reshape(-1,4)[k], config)
                       for k,s in enumerate(states.reshape(-1,7))]
    monkeypatch.setattr(flight, 'derivative', historical_derivative)
    np.testing.assert_array_equal(optimized, flight.derivative(states, controls, config.flight, wind))
    np.testing.assert_array_equal(rolled, flight.propagate(states, controls, durations, config.flight, wind=wind, substeps=4))
    assert classifications == [flight.envelope_ok(s, controls if held_control else controls.reshape(-1,4)[k], config)
                               for k,s in enumerate(states.reshape(-1,7))]


def test_batched_physical_step_retains_nominal_and_refined_switch_checks():
    from swarm_sim.trajectory2d.physical_generation import _step
    rng = np.random.default_rng(802)
    config = Config(dimensions=3)
    for _ in range(100):
        state = rng.uniform([-10., -10., .01, 7.9, -3., -.7, -.9],
                            [10., 10., 180., 16.1, 3., .7, .9])
        control = rng.uniform([0., 0., .2, -.9], [5., 5., 2.5, .9])
        following = flight.propagate(state, control, config.dt, config.flight)
        checked = flight.propagate(state, control, np.array([0., .25, .5, .75, 1.])*config.dt,
                                   config.flight, substeps=4)
        expected = flight.envelope_ok(checked, control, config) and flight.envelope_ok(following, control, config)
        actual = _step(state, control, config)
        assert (actual is not None) == expected
        if expected:
            np.testing.assert_array_equal(actual, following)
