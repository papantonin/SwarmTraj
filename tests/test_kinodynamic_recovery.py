"""Focused Phase 2 laboratory tests; full planner regressions still required."""

from dataclasses import replace

import numpy as np
import pytest

from swarm_sim.trajectory2d.flight3d import envelope_ok, propagate
from swarm_sim.trajectory2d.kinodynamic_recovery import (
    recovery_seed_acceleration,
    simulate_pullout,
)
from swarm_sim.trajectory2d.planning import Config


@pytest.mark.parametrize('speed', [13.0, 14.0])
def test_integrated_pullout_at_report_speeds(speed):
    # Matches the reported geometric stress family (30 m target, -30 degrees).
    # The relevant descent-rate allowance is explicit, not silently relaxed.
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target_altitude=30., descent_angle_deg=30.,
                    horizon=120., max_descent_rate=8., min_altitude=0.)
    result = simulate_pullout(speed, config)
    assert result is not None
    assert result.altitude_loss_m < config.target_altitude - config.min_altitude
    assert np.all(np.diff(result.states[:, 2]) <= 1e-6)
    assert abs(result.states[-1, 5]) <= 1e-3
    assert np.isclose(result.duration_s, len(result.controls) * config.dt)
    # Independent reintegration of every held command.
    reintegrated = propagate(result.states[:-1], result.controls,
                             config.dt, config.flight)
    np.testing.assert_allclose(reintegrated, result.states[1:], atol=1e-9, rtol=0)
    for fraction in (0., .25, .5, .75, 1.):
        points = propagate(result.states[:-1], result.controls,
                           fraction * config.dt, config.flight, substeps=4)
        assert envelope_ok(points, result.controls, config)
    accel = recovery_seed_acceleration(speed, config)
    assert accel is not None and accel > 0


def test_recovery_fails_closed_below_ground():
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target_altitude=2., descent_angle_deg=30.,
                    horizon=120., max_descent_rate=8., min_altitude=0.)
    assert simulate_pullout(14., config) is None
    assert recovery_seed_acceleration(14., config) is None


def test_recovery_does_not_mutate_limits():
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target_altitude=30., descent_angle_deg=30.,
                    horizon=120., max_descent_rate=8.)
    before = replace(config)
    simulate_pullout(13., config)
    assert config == before


def test_recovery_not_used_for_planar_or_level_flight():
    assert simulate_pullout(13., Config()) is None
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target_altitude=30., descent_angle_deg=0.)
    assert recovery_seed_acceleration(13., config) is None


@pytest.mark.parametrize('speed', [13.0, 14.0])
def test_physical_seed_avoids_reported_template_rejection(speed):
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target_altitude=30., descent_angle_deg=30.,
                    horizon=120., max_descent_rate=8.)
    rate = speed * np.tan(np.deg2rad(config.descent_angle_deg))
    def template_drop(acceleration):
        ramp_steps = int(np.ceil(rate / (acceleration * config.dt)))
        return rate * ramp_steps * config.dt / 2
    assert template_drop(.9 * config.max_vertical_acceleration) > 30.
    assert template_drop(recovery_seed_acceleration(speed, config)) < 30.


def test_opt_in_mode_does_not_change_default_or_allow_planar_usage():
    assert Config().generator_mode == 'geometric'
    Config(dimensions=3, numerical_backend='python', target_altitude=30., generator_mode='physical_recovery_seed')
    with pytest.raises(ValueError, match='generator_mode'):
        Config(generator_mode='physical_recovery_seed')
    with pytest.raises(ValueError, match='generator_mode'):
        Config(generator_mode='invalid')


@pytest.mark.parametrize('speed', [13., 14.])
@pytest.mark.parametrize('dt', [.1, .2])
def test_pullout_matches_independently_written_unbanked_ode(speed, dt):
    from scipy.integrate import solve_ivp
    from swarm_sim.trajectory2d.dynamics import G

    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target_altitude=30., descent_angle_deg=30.,
                    horizon=120., max_descent_rate=8., dt=dt)
    trial = simulate_pullout(speed, config)
    assert trial is not None
    reference = trial.states[0, [0, 2, 3, 5]].copy()
    for command, endpoint in zip(trial.controls, trial.states[1:]):
        def unbanked_rhs(_, state):
            _, _, airspeed, gamma = state
            thrust, brake, load, _ = command
            drag = (config.flight.parasitic_drag * airspeed**2
                    + config.flight.induced_drag * load**2 / airspeed**2)
            return [airspeed * np.cos(gamma), airspeed * np.sin(gamma),
                    thrust - brake - drag - G * np.sin(gamma),
                    G * (load - np.cos(gamma)) / airspeed]

        solution = solve_ivp(unbanked_rhs, (0, dt), reference,
                             method='DOP853', rtol=1e-12, atol=1e-12)
        assert solution.success
        reference = solution.y[:, -1]
        np.testing.assert_allclose(endpoint[[0, 2, 3, 5]], reference, atol=2e-6, rtol=0)


@pytest.mark.parametrize('speed', [0., -1., np.nan, np.inf, 7., 17.])
def test_pullout_refuses_invalid_or_out_of_envelope_speed(speed):
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target_altitude=30., descent_angle_deg=30.,
                    max_descent_rate=8.)
    assert simulate_pullout(speed, config) is None


@pytest.mark.parametrize('angle', [0., -1., 90., np.nan, np.inf])
def test_pullout_refuses_invalid_boundary_angle(angle):
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target_altitude=30., max_descent_rate=8.)
    assert simulate_pullout(13., config, angle_deg=angle) is None


def test_cached_seed_respects_airbrake_and_descent_limits():
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target_altitude=30., descent_angle_deg=30.,
                    max_descent_rate=8.)
    seed = recovery_seed_acceleration(13., config)
    assert seed is not None
    no_airbrake = replace(config, flight=replace(config.flight, max_brake_acceleration=0.))
    restricted_descent = replace(config, max_descent_rate=2.)
    assert recovery_seed_acceleration(13., no_airbrake) is None
    assert recovery_seed_acceleration(13., restricted_descent) is None
    assert recovery_seed_acceleration(13., config) == seed


def test_pullout_arrays_are_read_only():
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target_altitude=30., descent_angle_deg=30.,
                    max_descent_rate=8.)
    trial = simulate_pullout(13., config)
    assert trial is not None
    for array in (trial.states, trial.controls):
        with pytest.raises(ValueError, match='read-only'):
            array.flat[0] = 0.
