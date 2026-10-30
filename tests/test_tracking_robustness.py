"""Own-state disturbance rejection, held-control safety and explicit failures."""

from dataclasses import replace

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, Snapshot, Trajectory, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.dynamics import propagate as planar
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.execution_safety import envelope_margins
from swarm_sim.trajectory2d.flight3d import envelope_ok, propagate, trim_control
from swarm_sim.trajectory2d.perturbations import PositionSensor, position_bias_vectors, wind_at
from swarm_sim.trajectory2d.tracking import held_control_ok
from swarm_sim.trajectory2d.native_backend import protect_control


@pytest.mark.parametrize('model', ['rotating', 'constant', 'gust'])
def test_wind_profiles_are_deterministic_horizontal_and_bounded(model):
    for drone_id in range(4):
        values = np.array([wind_at(t, drone_id, .8, model, 37) for t in np.linspace(0, 240, 1000)])
        assert np.linalg.norm(values, axis=1).max() <= .8 + 1e-12
        assert np.all(values[:, 2] == 0)
        np.testing.assert_array_equal(values[0], wind_at(0, drone_id, .8, model, 37))
    if model == 'constant':
        np.testing.assert_array_equal(wind_at(0, 0, .8, model, 37), wind_at(100, 3, .8, model, 37))


def test_position_errors_are_bounded_seeded_and_independent_of_fleet_membership():
    biases = position_bias_vectors([0, 1, 2, 3], .3, 3)
    np.testing.assert_allclose(np.linalg.norm(biases, axis=1), .3, atol=1e-15)
    np.testing.assert_array_equal(biases[[3, 1]], position_bias_vectors([3, 1], .3, 3))
    a, b = [PositionSensor(3, 3, biases[3], .1, 11) for _ in range(2)]
    samples = np.array([a.error() for _ in range(1000)])
    np.testing.assert_array_equal(samples, [b.error() for _ in range(1000)])
    assert np.linalg.norm(samples - biases[3], axis=1).max() <= .1




def test_combined_command_is_protected_across_the_whole_held_interval_without_state_clipping():
    config = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp')
    state = np.array([0., 0., 80., 12., .2, 0., .4])
    original = state.copy()
    requested = np.array([1., 0., 2., .5])
    assert not held_control_ok(state, requested, config, .2)
    command, intervened, feasible = protect_control(state, requested, config, .2)
    assert intervened and feasible
    np.testing.assert_array_equal(state, original)
    assert held_control_ok(state, command, config, .2)
    samples = propagate(state, command, np.linspace(0, .2, 101), config.flight, substeps=8)
    assert envelope_ok(samples, command, config)
    assert config.max_vertical_acceleration == 1 and config.limits.max_bank_deg == 50




def test_diagnostic_margins_match_the_existing_physical_checker():
    config = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp')
    states = np.array([[0., 0., 80., 12., .2, 0., .4], [0., 0., 80., 12., .2, 0., .4]])
    controls = np.array([trim_control(states[0], config.flight)] * 2)
    assert envelope_ok(states, controls, config)
    assert min(np.min(v) for v in envelope_margins(states, controls, config).values()) >= -1e-12
    controls[1, 2] = 2
    assert not envelope_ok(states, controls, config)
    assert envelope_margins(states, controls, config)['vertical_acceleration'][1] < 0


def straight_scene():
    from test_execution_boundaries import straight_flight
    return straight_flight()


@pytest.mark.parametrize('options', [
    {'wind_model': 'bad'}, {'wind_heading_deg': np.nan}, {'position_noise': -.1},
    {'position_noise': np.inf}, {'position_biases': [[0, 0]]},
    {'position_biases': [[0, np.nan, 0]]}, {'disturbance_seed': -1},
    {'disturbance_seed': True}, {'controller': 'bad'},
])
def test_invalid_disturbance_settings_fail_before_execution(options):
    config, plan = straight_scene()
    with pytest.raises(ValueError):
        simulate((plan,), config, **options)


def test_clock_and_tracking_assumption_losses_have_explicit_time_and_id():
    from test_execution_boundaries import straight_flight
    config, plan = straight_flight()
    report = simulate((plan,), config, clock_offsets=[.4]).report
    assert not report['execution_validated']
    event = next(e for e in report['safety_events'] if e['kind'] == 'clock_bound_exceeded')
    assert event['drone_id'] == 0 and event['first_observed_time_s'] == 0
    assert 'tube_conditions_verified' in report['execution_failures']


@pytest.fixture(scope='module', params=[7, 19])
def regression_scene(request):
    config = Config(seed=request.param, dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', target_altitude=30, horizon=150,
                    max_descent_rate=10, descent_angle_deg=30, cruise_altitude_range=(30, 120),
                    approach_spread_deg=180, planning_strategy='adaptive', search_order='mrv')
    snapshots = tuple(replace(s, altitude=30 + 25*k) for k, s in enumerate(separate_loiter_snapshot(4)))
    result = plan_mission(snapshots, config)
    assert result.validated
    return config, result


def test_previously_failing_four_drone_wind_clock_trials_pass_unchanged_limits(regression_scene):
    config, result = regression_scene
    evaluation = simulate(result.plans, config, wind=.05, clock_offsets=np.linspace(-.05, .05, 4))
    assert evaluation.report['execution_validated'], evaluation.report['execution_failures']
    assert not evaluation.report['safety_events']
    assert config.position_error == 2 and config.clock_error == .15
    assert config.max_vertical_acceleration == 1
    with pytest.raises(ValueError, match='legacy'):
        simulate(result.plans, config, wind=.05, clock_offsets=np.linspace(-.05,.05,4), controller='legacy')


def test_sensor_noise_is_reproducible_and_no_other_drone_inputs_are_used(regression_scene):
    config, result = regression_scene
    options = dict(wind=.1, wind_model='gust', position_noise=.01, disturbance_seed=5)
    biases = position_bias_vectors([p.drone_id for p in result.plans], .05, 3)
    offsets = np.linspace(-.03, .03, 4)
    full = simulate(result.plans, config, clock_offsets=offsets, position_biases=biases, **options)
    alone = simulate((result.plans[0],), config, clock_offsets=offsets[:1], position_biases=biases[:1], **options)
    np.testing.assert_array_equal(full.flight_actual[:, 0], alone.flight_actual[:, 0])
    assert full.report['controller_protection_verified']
    assert alone.report['position_measurement_error_bound_m'][0] == pytest.approx(.06)
