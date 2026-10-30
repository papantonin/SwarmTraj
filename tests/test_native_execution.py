"""Own-state native execution agrees with Python; the observer remains independent."""

from dataclasses import replace
import importlib.util

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d import flight3d as flight
from swarm_sim.trajectory2d import native_backend as native
from swarm_sim.trajectory2d.execution import Executor, check_execution_backend, simulate
from swarm_sim.trajectory2d.tracking import held_control_ok

HAS_NATIVE = importlib.util.find_spec('swarm_sim._trajectory_native') is not None
requires_native = pytest.mark.skipif(not HAS_NATIVE, reason='optional C++ extension not built')


def test_execution_backend_is_separate_explicit_and_fails_closed(monkeypatch):
    c = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp')
    check_execution_backend(c, 'constrained', 'cpp')
    for config, controller, backend in ((c, 'constrained', 'python'), (c, 'constrained', 'bad'),
            (Config(dimensions=2), 'constrained', 'cpp'), (c, 'legacy', 'cpp')):
        with pytest.raises(ValueError):
            check_execution_backend(config, controller, backend)
    class OldKernel:
        def build_info(self):
            return {'abi': 1}
    monkeypatch.setattr(native, 'require_native', lambda: OldKernel())
    with pytest.raises(RuntimeError, match='rebuild'):
        check_execution_backend(c, 'constrained', 'cpp')


@requires_native
@pytest.mark.parametrize('substeps', [1, 4, 8])
def test_native_sampled_integration_is_independent_per_duration(substeps):
    c = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp')
    s = np.array([10., 20., 80., 12., .2, -.1, .3])
    u = flight.trim_control(s, c.flight)
    durations = np.array([.2, 0., .07, -.1, .2])
    wind = np.array([.2, -.3, 0.])
    expected = flight.propagate(s, u, durations, c.flight, wind=wind, substeps=substeps)
    original = s.copy()
    actual = native.propagate_samples(s, u, durations, c, disturbance=wind, substeps=substeps)
    np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-12)
    np.testing.assert_array_equal(s, original)
    assert not actual.flags.writeable
    with pytest.raises(ValueError):
        actual.setflags(write=True)


@requires_native
def test_execution_binding_rejects_true_wind_partial_history_and_invalid_intervals():
    c = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp')
    s = np.array([0., 0., 80., 12., 0., 0., 0.])
    u = flight.trim_control(s, c.flight)
    for dt in (0., -.2, np.nan, np.inf):
        with pytest.raises(ValueError):
            native.local_control_update(s, s, u, c, dt, .2, estimated_wind=np.zeros(3))
        with pytest.raises(ValueError):
            native.protect_control(s, u, c, dt)
    with pytest.raises(ValueError, match='both own'):
        native.local_control_update(s, s, u, c, .2, .2,
            estimated_wind=np.zeros(3), previous_measured=s)
    with pytest.raises(ValueError, match='no true simulator disturbance'):
        native.evaluate('execution_control', s, u, c, reference=s, wind=np.zeros(3))
    with pytest.raises(ValueError):
        native.propagate_samples(s, u, [np.nan], c)


@requires_native
def test_native_protection_is_independently_safe_and_impossible_fallback_is_retained():
    c = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp')
    rng = np.random.default_rng(546)
    fixtures = [(np.array([0., 0., -1., 12., 0., 0., 0.]), np.array([1., 0., 1., 0.]))]
    for _ in range(100):
        state = rng.uniform([-20, -20, 30, 8, -3, -.3, -.9], [20, 20, 100, 16, 3, .3, .9])
        request = rng.uniform([-.1, -.1, .1, -1.2], [5.2, 5.2, 2.6, 1.2])
        fixtures.append((state, request))
    outcomes = set()
    for state, request in fixtures:
        original = state.copy()
        b, bi, bf = native.protect_control(state, request, c, .2)
        if bf:
            assert held_control_ok(state, b, c, .2)
        outcomes.add((bi, bf))
        np.testing.assert_array_equal(state, original)
    assert {(True, False), (True, True), (False, True)} <= outcomes


@requires_native
def test_held_command_classification_at_envelope_boundaries():
    from swarm_sim.trajectory2d.tracking import held_control_ok
    c = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp')
    state = np.array([0., 0., 80., 12., .2, 0., 0.])
    fixtures = []
    for speed in (c.limits.min_speed, c.limits.max_speed):
        for delta in (-2e-8, 0., 2e-8):
            s = state.copy(); s[3] = speed + delta
            fixtures.append((s, flight.trim_control(s, c.flight)))
    for altitude in (c.min_altitude, c.max_altitude):
        for delta in (-2e-8, 0., 2e-8):
            s = state.copy(); s[2] = altitude + delta
            fixtures.append((s, flight.trim_control(s, c.flight)))
    for sign in (-1, 1):
        for delta in (-2e-8, 0., 2e-8):
            s = state.copy(); s[6] = sign * (np.deg2rad(c.limits.max_bank_deg) + delta)
            fixtures.append((s, flight.trim_control(s, c.flight)))
    for s, u in fixtures:
        assert native.evaluate('held_control', s, u, c, hold_dt=.2) == held_control_ok(s, u, c, .2)


@pytest.fixture(scope='module', params=['basic', 'rich', 'own_profile', 'geometric'])
def scene(request):
    if not HAS_NATIVE:
        pytest.skip('optional C++ extension not built')
    c = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp',
        target=(500., 0.), target_altitude=30., descent_angle_deg=30., max_descent_rate=8.,
        horizon=120., cruise_altitude_range=(70., 100.), max_extra_loiter=0., candidates=1, attempts=6)
    s = replace(separate_loiter_snapshot(1)[0], altitude=80.).with_speed(13., c.limits)
    if request.param == 'rich':
        c = replace(c, cruise_levels=3, speed_profile='smooth', speed_profile_range=(10., 14.),
            horizon=210., target=(500., 500.), cruise_altitude_range=(30., 120.), max_descent_rate=10.)
    elif request.param == 'own_profile':
        from swarm_sim.trajectory2d.aircraft_profiles import reference_aircraft_profile
        profile = replace(reference_aircraft_profile(), name='native execution own physics', bounding_radius_m=1.2,
            flight=replace(c.flight, parasitic_drag=.0033, max_thrust_acceleration=4.5))
        c = replace(c, aircraft_profiles=((s.drone_id, profile),))
    elif request.param == 'geometric':
        c = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', target_altitude=30., descent_angle_deg=30.,
            max_descent_rate=10., horizon=120., cruise_altitude_range=(30., 120.),
            candidates=1, attempts=30)
        s = replace(separate_loiter_snapshot()[0], altitude=30.)
    result = plan_mission((s,), c)
    assert result.validated
    return c, result.plans


@requires_native
def test_own_controller_sequence_including_clock_shifted_feedforward(scene):
    c, plans = scene
    p = plans[0]
    c = c.for_drone(p.drone_id)
    a, b = Executor(p, c.limits), Executor(p, c.limits, execution_backend='cpp')
    rng = np.random.default_rng(81)
    for t in np.arange(-.03, min(p.horizon, 30.), .2):
        measured = p.sample_flight(t).copy()
        measured[:3] += rng.uniform(-.03, .03, 3)
        ua = a.flight_command(t, .20002, c, measured, control_dt=.19998)
        ub = b.flight_command(t, .20002, c, measured, control_dt=.19998)
        np.testing.assert_allclose(ua, ub, rtol=0, atol=1e-11)
        np.testing.assert_allclose(a.wind_estimator.value, b.wind_estimator.value, rtol=0, atol=1e-11)
        assert (a.filter_interventions, a.filter_unresolved) == (b.filter_interventions, b.filter_unresolved)
    # A controller freshly initialized after the end uses terminal trim; avoid
    # fabricating a many-second held interval for the own-motion estimator.
    a, b = Executor(p, c.limits), Executor(p, c.limits, execution_backend='cpp')
    t = p.horizon + .01
    np.testing.assert_allclose(a.flight_command(t, .2, c), b.flight_command(t, .2, c), atol=1e-11, rtol=0)


@requires_native
@pytest.mark.parametrize('profile', ['nominal', 'gust_sensor', 'outside_clock'])
def test_complete_execution_decisions_and_physical_states(scene, profile):
    c, plans = scene
    from swarm_sim.trajectory2d.perturbations import position_bias_vectors
    settings = {}
    if profile == 'gust_sensor':
        settings = dict(wind=.1, wind_model='gust', clock_offsets=[.03], clock_drifts=[.0001],
            position_biases=position_bias_vectors([plans[0].drone_id], .05, 3),
            position_noise=.01, disturbance_seed=23)
    elif profile == 'outside_clock':
        settings = dict(clock_offsets=[.4])
    before = [p.to_dict() for p in plans]
    a = simulate(plans, c, **settings)
    b = simulate(plans, c, execution_backend='cpp', **settings)
    assert a.report['execution_backend'] == 'cpp'
    assert b.report['execution_backend_identity']['execution_api'] == 1
    for key in ('execution_validated', 'execution_failures', 'physical_limits_verified',
                'controller_protection_verified', 'tube_conditions_verified'):
        assert a.report[key] == b.report[key]
    assert [(e['kind'], e.get('constraint'), e.get('drone_id')) for e in a.report['safety_events']] == [
        (e['kind'], e.get('constraint'), e.get('drone_id')) for e in b.report['safety_events']]
    if profile == 'outside_clock':
        assert not b.report['execution_validated']
    np.testing.assert_allclose(a.flight_actual, b.flight_actual, rtol=0, atol=1e-8)
    np.testing.assert_allclose(a.tracking_errors, b.tracking_errors, rtol=0, atol=1e-8)
    assert before == [p.to_dict() for p in plans]


@requires_native
def test_failed_native_protection_invalidates_execution(monkeypatch):
    from test_execution_boundaries import straight_flight
    c, p = straight_flight()
    original = native.local_control_update
    def unverified(*args, **kwargs):
        command, estimate, _, _ = original(*args, **kwargs)
        return command, estimate, True, False
    monkeypatch.setattr(native, 'local_control_update', unverified)
    report = simulate((p,), c, execution_backend='cpp').report
    assert not report['execution_validated']
    assert not report['controller_protection_verified']
    assert any(e['kind'] == 'control_protection_not_verified' for e in report['safety_events'])


@requires_native
def test_native_own_execution_is_identical_alone_or_in_a_fleet():
    from test_execution_boundaries import straight_flight
    from swarm_sim.trajectory2d.perturbations import position_bias_vectors
    c, p = straight_flight()
    other = replace(p, drone_id=3)
    options = dict(wind=.1, wind_model='gust', position_noise=.01, disturbance_seed=5,
                   execution_backend='cpp')
    bias = position_bias_vectors([0, 3], .05, 3)
    full = simulate((p, other), c, clock_offsets=[.03, -.03], position_biases=bias, **options)
    alone = simulate((p,), c, clock_offsets=[.03], position_biases=bias[:1], **options)
    np.testing.assert_array_equal(full.flight_actual[:, 0], alone.flight_actual[:, 0])
