"""Exact state/control pairs at both sides of asynchronous actuator switches."""

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, Snapshot, Trajectory
import swarm_sim.trajectory2d.execution as execution
from swarm_sim.trajectory2d.flight3d import from_snapshot, project, propagate, trim_control


def straight_flight():
    config = Config(dimensions=3, target=(24., 0.), target_altitude=80., horizon=4.,
                    activation_time=0., max_extra_loiter=0., post_arrival_time=1.,
                    cruise_altitude_range=(80., 80.))
    snapshot = Snapshot(0, (0., 0., 0., 0.), 12., altitude=80.)
    initial = from_snapshot(snapshot)
    command = trim_control(initial, config.flight)
    count = round(config.horizon / config.dt)
    states = propagate(initial, command, np.arange(count + 1) * config.dt, config.flight)
    controls = np.tile(command, (count, 1))
    projected = project(states, command, config.flight)
    plan = Trajectory(0, 0, 12., config.dt, projected[:, :4], np.zeros(count), 2., 0.,
        vertical_states=projected[:, 4:], vertical_accelerations=np.zeros(count),
        cruise_altitude=80., flight_states=states, flight_controls=controls,
        flight_parameters=config.flight, airspeed_limit=config.limits.max_speed)
    return config, plan


@pytest.mark.parametrize('drift', [-.0001, .0001])
def test_valid_asynchronous_switches_keep_physical_acceptance(drift):
    config, plan = straight_flight()
    report = execution.simulate((plan,), config, clock_offsets=[.015], clock_drifts=[drift]).report
    assert report['execution_validated'], report['execution_failures']
    assert report['physical_limits_verified'] and not report['safety_events']


@pytest.mark.parametrize('boundary_index', [1, 2], ids=['incoming', 'outgoing'])
def test_bad_command_on_either_side_of_hidden_switch_is_still_rejected(monkeypatch, boundary_index):
    config, plan = straight_flight()
    healthy = execution.simulate((plan,), config, clock_offsets=[.015])
    assert healthy.report['execution_validated']
    original = execution._coupled_interval
    injected_at = []

    def inject_observed_boundary(*args, **kwargs):
        states, controls, first, boundaries = original(*args, **kwargs)
        if not injected_at:
            at, state, command = boundaries[boundary_index]
            # Manufactured unsafe boundary observation: leave the displayed
            # sample arrays and simulated dynamics untouched, so this failure
            # is visible only through the mandatory exact-boundary checks.
            command = command.copy()
            command[2] = 2.
            boundaries[boundary_index] = (at, state.copy(), command)
            injected_at.append(at)
        return states, controls, first, boundaries

    monkeypatch.setattr(execution, '_coupled_interval', inject_observed_boundary)
    unsafe = execution.simulate((plan,), config, clock_offsets=[.015])
    np.testing.assert_array_equal(healthy.flight_actual, unsafe.flight_actual)
    assert injected_at[0] == pytest.approx(.185)
    assert not np.any(np.isclose(unsafe.times, injected_at[0], atol=1e-10, rtol=0))
    assert not unsafe.report['physical_limits_verified']
    assert not unsafe.report['execution_validated']
    events = [e for e in unsafe.report['safety_events'] if e.get('constraint') == 'vertical_acceleration']
    assert events and events[0]['first_observed_time_s'] == pytest.approx(injected_at[0])
