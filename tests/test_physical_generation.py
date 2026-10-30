"""Direct physical missions must satisfy the original independent validators."""

from dataclasses import replace
import pickle

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.flight3d import air_velocity, propagate
from swarm_sim.trajectory2d.kinodynamic_recovery import pullout_from_state, simulate_pullout, validate_pullout
from swarm_sim.trajectory2d.planning import mission_document, validate_individual


def mission_case(speed=13., **changes):
    config = Config(dimensions=3, numerical_backend='python', generator_mode='physical_mission', target=(500., 0.),
        target_altitude=30., descent_angle_deg=30., max_descent_rate=8.,
        horizon=120., cruise_altitude_range=(70., 100.), max_extra_loiter=0.,
        candidates=1, attempts=3)
    config = replace(config, **changes)
    snapshot = replace(separate_loiter_snapshot(1)[0], altitude=80.).with_speed(speed, config.limits)
    return snapshot, config


@pytest.mark.parametrize('changes', [{}, {'target': (500., 500.), 'horizon': 210.,
    'max_descent_rate': 10., 'cruise_altitude_range': (30., 120.), 'cruise_levels': 2,
    'speed_profile': 'smooth', 'speed_profile_range': (10., 14.)}])
def test_proposal_caching_preserves_complete_plans_and_search_signatures(monkeypatch, changes):
    import swarm_sim.trajectory2d.physical_generation as generation
    snapshot, config = mission_case(speed=12., **changes)
    cached = {name: getattr(generation, name) for name in ('_approach_headroom', '_terminal_distance')}
    for function in cached.values():
        function.cache_clear()
    cold = plan_mission((snapshot,), config)
    warm = plan_mission((snapshot,), config)
    assert cold.validated and warm.validated
    for name, function in cached.items():
        monkeypatch.setattr(generation, name, function.__wrapped__)
    uncached = plan_mission((snapshot,), config)
    assert uncached.validated
    # Compare full selected arrays, candidate counts and search work; no timing
    # assertions. Independent validation still checks every returned trajectory.
    for result in (cold, warm):
        assert result.counts == uncached.counts and result.search_nodes == uncached.search_nodes
        assert [p.to_dict() for p in result.plans] == [p.to_dict() for p in uncached.plans]
        assert all(validate_individual(p, snapshot, config) for p in result.plans)


def test_proposal_cache_keeps_mutable_config_inputs_on_uncached_path():
    snapshot, config = mission_case(speed=12.)
    expected = plan_mission((snapshot,), config)
    actual = plan_mission((snapshot,), replace(config, target=list(config.target)))
    assert expected.validated and actual.validated
    assert actual.counts == expected.counts and actual.search_nodes == expected.search_nodes
    assert [p.to_dict() for p in actual.plans] == [p.to_dict() for p in expected.plans]


@pytest.mark.parametrize('speed', [13., 14.])
def test_complete_physical_mission_without_legacy_vertical_or_scalar_seed(monkeypatch, speed):
    def forbidden(*args, **kwargs):
        pytest.fail('direct physical generation must not use legacy conversion or scalar seeds')
    monkeypatch.setattr('swarm_sim.trajectory2d.planning.build_profile', forbidden)
    monkeypatch.setattr('swarm_sim.trajectory2d.coupled_planning.convert_candidate', forbidden)
    monkeypatch.setattr('swarm_sim.trajectory2d.kinodynamic_recovery.recovery_seed_acceleration', forbidden)
    snapshot, config = mission_case(speed)
    result = plan_mission((snapshot,), config)
    assert result.validated, (result.status, result.diagnostics)
    plan, = result.plans
    assert validate_individual(plan, snapshot, config)
    assert len(plan.flight_controls) == round(config.horizon / config.dt)
    phases = plan.generation_phases
    assert {p[0] for p in phases} == {'loiter', 'transit', 'alignment', 'descent_entry',
                                   'passage', 'pullout', 'cruise_recovery', 'onward'}
    assert phases[0][1] == 0 and phases[-1][2] == len(plan.flight_controls)
    assert all(a[2] == b[1] for a, b in zip(phases, phases[1:]))
    rolled = propagate(plan.flight_states[:-1], plan.flight_controls, config.dt, config.flight)
    np.testing.assert_allclose(rolled, plan.flight_states[1:], atol=1e-9, rtol=0)
    _, start, end = next(p for p in phases if p[0] == 'pullout')
    primitive = pullout_from_state(plan.flight_states[start], config)
    assert primitive is not None
    assert validate_pullout(primitive, config)
    np.testing.assert_array_equal(primitive.states, plan.flight_states[start:end + 1])
    np.testing.assert_array_equal(primitive.controls, plan.flight_controls[start:end])
    assert plan.flight_states[:, 2].min() > config.min_altitude
    assert abs(plan.flight_states[-1, 2] - plan.cruise_altitude) < .2
    assert abs(air_velocity(plan.flight_states[-1])[2]) < .1
    payload = mission_document(result, config, (snapshot,))
    assert payload['config']['generator_mode'] == 'physical_mission'
    assert payload['plans'][0]['generation_phases'] == [list(p) for p in phases]
    received = pickle.loads(pickle.dumps(plan))
    assert received.generation_phases == phases
    assert not received.flight_states.flags.writeable


def test_pullout_preserves_arbitrary_actual_boundary_and_pickle_immutability():
    _, config = mission_case()
    state = np.array([123., -52., 73., 13.1, .7, -.45, 0.])
    before = state.copy()
    primitive = pullout_from_state(state, config)
    assert primitive is not None
    np.testing.assert_array_equal(primitive.entry_state, before)
    np.testing.assert_array_equal(state, before)
    assert abs(primitive.exit_state[5]) <= 1e-4
    assert primitive.altitude_loss_m == pytest.approx(state[2] - primitive.states[:, 2].min())
    assert primitive.exit_state[4] == pytest.approx(state[4])
    received = pickle.loads(pickle.dumps(primitive))
    assert not received.states.flags.writeable and not received.controls.flags.writeable


def test_pullout_rejects_banked_boundaries_and_preserves_exact_time_budget():
    _, config = mission_case()
    state = np.array([4., 8., 30., 13., .4, -np.pi / 6, 0.])
    trial = pullout_from_state(state, config)
    assert trial is not None
    exact = pullout_from_state(state, config, max_duration_s=trial.duration_s)
    assert exact is not None
    np.testing.assert_array_equal(exact.states, trial.states)
    assert pullout_from_state(state, config, max_duration_s=trial.duration_s - config.dt) is None
    banked = state.copy()
    banked[6] = .01
    assert pullout_from_state(banked, config) is None
    assert simulate_pullout(13., config).duration_s == pytest.approx(trial.duration_s)


def test_primitive_verifier_rejects_changed_controls_and_physics():
    _, config = mission_case()
    trial = simulate_pullout(13., config)
    assert validate_pullout(trial, config)
    controls = trial.controls.copy()
    controls[-1, 2] = 10.
    assert not validate_pullout(replace(trial, controls=controls), config)
    assert not validate_pullout(trial, replace(config, dt=.1))
    assert not validate_pullout(trial, replace(config, flight=replace(config.flight, max_brake_acceleration=0.)))


@pytest.mark.parametrize('changes', [dict(horizon=45.), dict(arrival_deadline=20.),
                                   dict(post_arrival_time=110.)])
def test_full_mission_rejects_insufficient_time_without_extending_requirements(changes):
    snapshot, config = mission_case(**changes)
    before = replace(config)
    result = plan_mission((snapshot,), config)
    assert not result.validated and not result.plans
    assert config == before


@pytest.mark.parametrize('changes', [dict(dimensions=2),
    dict(speed_profile='smooth', speed_profile_range=(10., 14.),
         reservation_policy='grid', passage_guidance='safe_windows')])
def test_unsupported_physical_requests_are_explicit_errors(changes):
    with pytest.raises(ValueError, match='generator_mode'):
        mission_case(**changes)


def test_compatibility_alias_uses_complete_physical_generation():
    snapshot, config = mission_case()
    first = plan_mission((snapshot,), config)
    alias = plan_mission((snapshot,), replace(config, generator_mode='physical_recovery_seed'))
    assert first.validated and alias.validated
    assert first.plans[0].to_dict() == alias.plans[0].to_dict()


def test_level_passage_and_layered_mode_remain_full_horizon_missions():
    snapshot, config = mission_case(descent_angle_deg=0.)
    for local in (config, replace(config, target_altitude=None)):
        result = plan_mission((snapshot,), local)
        assert result.validated, (result.status, result.diagnostics)
        assert validate_individual(result.plans[0], snapshot, local)


def test_actual_aircraft_profile_is_authoritative_and_remains_own_only():
    from swarm_sim.trajectory2d import reference_aircraft_profile
    snapshot, config = mission_case()
    reference = reference_aircraft_profile()
    profile = replace(reference, name='physical prototype fixture', bounding_radius_m=1.2,
                      performance=replace(reference.performance, max_vertical_acceleration=1.1))
    fleet_config = replace(config, aircraft_profiles=((snapshot.drone_id, profile),))
    result = plan_mission((snapshot,), fleet_config)
    assert result.validated, (result.status, result.diagnostics)
    plan, = result.plans
    own = fleet_config.for_drone(snapshot.drone_id)
    assert own.max_vertical_acceleration == 1.1 and own.aircraft_radius == 1.2
    assert not own.aircraft_profiles and fleet_config.max_vertical_acceleration == 1.
    assert plan.aircraft_profile == profile
    assert validate_individual(plan, snapshot, fleet_config)
    payload = mission_document(result, fleet_config, (snapshot,))
    assert payload['plans'][0]['aircraft_profile'] == profile.record()


def test_spawn_preserves_direct_physical_catalogs_and_boundary_metadata():
    from swarm_sim.trajectory2d.parallel import generate_catalogs
    from swarm_sim.trajectory2d.planning import _generate_catalog
    _, config = mission_case(horizon=150.)
    snapshots = tuple(replace(s, altitude=80. + 10 * k).with_speed(13. + k, config.limits)
                      for k, s in enumerate(separate_loiter_snapshot(2)))
    tasks = [(s, config, None) for s in snapshots]
    serial = generate_catalogs(_generate_catalog, tasks, 1)
    parallel = generate_catalogs(_generate_catalog, tasks, 2)
    for a, b in zip(serial, parallel):
        assert a and b
        assert [p.to_dict() for p in a] == [p.to_dict() for p in b]
        assert all(not p.flight_controls.flags.writeable and not p.flight_states.flags.writeable for p in b)


@pytest.mark.parametrize('corruption', ['state', 'final_command', 'arrival'])
def test_existing_validator_rejects_corrupted_physical_mission(corruption):
    snapshot, config = mission_case()
    result = plan_mission((snapshot,), config)
    assert result.validated
    plan = result.plans[0]
    if corruption == 'state':
        states = plan.flight_states.copy()
        states[100, 2] += .5
        damaged = replace(plan, flight_states=states)
    elif corruption == 'final_command':
        commands = plan.flight_controls.copy()
        commands[-1, 2] = 10.
        damaged = replace(plan, flight_controls=commands)
    else:
        damaged = replace(plan, arrival_time=config.latest_nominal_arrival + config.dt)
    assert not validate_individual(damaged, snapshot, config)


@pytest.mark.parametrize('levels,speed', [(2, 'constant'), (3, 'constant'),
                                        (1, 'smooth'), (2, 'smooth'), (3, 'smooth')])
def test_requested_rich_profiles_are_actually_flown_without_geometric_conversion(monkeypatch, levels, speed):
    def forbidden(*args, **kwargs):
        pytest.fail('physical rich profiles must not use geometric conversion')
    monkeypatch.setattr('swarm_sim.trajectory2d.planning.build_profile', forbidden)
    monkeypatch.setattr('swarm_sim.trajectory2d.coupled_planning.convert_candidate', forbidden)
    snapshot, config = mission_case(speed=12., target=(500., 500.), max_descent_rate=10.,
        horizon=210., cruise_altitude_range=(30., 120.), cruise_levels=levels,
        speed_profile=speed, speed_profile_range=(10., 14.), attempts=4)
    result = plan_mission((snapshot,), config)
    assert result.validated, result.diagnostics
    plan, = result.plans
    assert validate_individual(plan, snapshot, config)
    if levels > 1:
        assert len(plan.cruise_levels_m) == levels
        assert not validate_individual(replace(plan, cruise_levels_m=(plan.cruise_altitude, 119.)), snapshot, config)
        times = np.arange(len(plan.flight_states)) * plan.dt
        previous = -1
        for altitude in plan.cruise_levels_m:
            reached = (times >= plan.release_time) & (times < plan.arrival_time)
            reached &= (np.abs(plan.flight_states[:, 2] - altitude) <= .5)
            reached &= (np.abs(air_velocity(plan.flight_states)[:, 2]) <= .2)
            reached[:previous + 1] = False
            assert reached.sum() * plan.dt >= 1.
            previous = np.flatnonzero(reached)[0]
    if speed == 'smooth':
        assert np.ptp(plan.speed_reference) >= .2 and np.ptp(plan.flight_states[:, 3]) >= .1
        assert not plan.speed_reference.flags.writeable
        terminal = next(start for name, start, end in plan.generation_phases
                        if name in ('alignment', 'descent_entry'))
        np.testing.assert_array_equal(plan.speed_reference[terminal:], np.full(
            len(plan.speed_reference) - terminal, snapshot.speed))
        assert not validate_individual(replace(plan, speed_reference=None), snapshot, config)
        assert not validate_individual(replace(plan, speed_reference=np.full_like(plan.speed_reference, snapshot.speed)),
                                       snapshot, config)
    received = pickle.loads(pickle.dumps(plan))
    assert received.to_dict() == plan.to_dict()
    assert not received.flight_states.flags.writeable
    assert plan.generation_phases[0][1] == 0
    assert plan.generation_phases[-1][2] == len(plan.flight_controls)
    assert all(a[2] == b[1] for a, b in zip(plan.generation_phases, plan.generation_phases[1:]))


def test_rich_requests_fail_closed_when_time_or_level_range_is_insufficient():
    snapshot, config = mission_case(speed=12., horizon=45., cruise_levels=2,
        speed_profile='smooth', speed_profile_range=(10., 14.), attempts=2)
    result = plan_mission((snapshot,), config)
    assert not result.validated and not result.plans
    narrow = replace(config, horizon=210., cruise_altitude_range=(80., 83.))
    result = plan_mission((snapshot,), narrow)
    assert not result.validated and not result.plans
    assert result.diagnostics['catalogs'][0]['rejections'] == {'physical_cruise_levels_range': 2}


def test_real_clock_speed_bump_preserves_joins_bounds_and_acceleration():
    from swarm_sim.trajectory2d.physical_generation import _speed_bump
    snapshot, config = mission_case(speed=12., speed_profile='smooth', speed_profile_range=(10., 14.))
    times = np.linspace(0., config.horizon, 12001)
    release = 10.
    reference, derivative, end = _speed_bump(snapshot, config, 0, release, times)
    assert reference.min() >= 10. and reference.max() <= 14.
    assert np.ptp(reference) >= .2
    assert np.max(np.abs(derivative)) <= config.speed_profile_acceleration
    assert np.all(reference[times <= release + 4.] == snapshot.speed)
    assert np.all(reference[times >= end] == snapshot.speed)
    active = (times > release + 4.01) & (times < end - .01)
    np.testing.assert_allclose(np.gradient(reference, times)[active], derivative[active], atol=1e-5)


def test_rich_catalogs_are_spawn_reproducible_and_keep_own_metadata():
    from swarm_sim.trajectory2d.parallel import generate_catalogs
    from swarm_sim.trajectory2d.planning import _generate_catalog
    _, config = mission_case(speed=12., target=(500., 500.), max_descent_rate=10.,
        horizon=240., cruise_altitude_range=(30., 120.), cruise_levels=2,
        speed_profile='smooth', speed_profile_range=(10., 14.), attempts=5)
    snapshots = tuple(replace(s, altitude=80. + 10 * k) for k, s in enumerate(separate_loiter_snapshot(2)))
    tasks = [(s, config, None) for s in snapshots]
    serial = generate_catalogs(_generate_catalog, tasks, 1)
    parallel = generate_catalogs(_generate_catalog, tasks, 2)
    for a, b in zip(serial, parallel):
        assert a and b
        assert [p.to_dict() for p in a] == [p.to_dict() for p in b]
        assert all(len(p.cruise_levels_m) == 2 and not p.speed_reference.flags.writeable for p in b)


def test_physical_mixed_catalog_exports_only_explicitly_declared_attained_family():
    from swarm_sim.trajectory2d.catalog import Catalog, extend_catalog
    snapshot, config = mission_case(speed=12., target=(500., 500.), max_descent_rate=10.,
        horizon=240., cruise_altitude_range=(30., 120.), cruise_levels=2,
        speed_profile='smooth', speed_profile_range=(10., 14.), attempts=6,
        profile_policy='adaptive_mixed', planning_strategy='adaptive')
    catalog = extend_catalog((snapshot, config, None, Catalog(profile_stage=3), 1))
    assert catalog.plans
    plan = catalog.plans[0]
    assert plan.profile_family == 'levels_2_speed'
    assert validate_individual(plan, snapshot, config)
    assert not validate_individual(replace(plan, profile_family=None), snapshot, config)


def test_earliest_legal_departure_is_explored_without_widening_bounds():
    from swarm_sim.trajectory2d.planning import _candidate
    snapshot, config = mission_case(speed=12., target=(500., 500.), horizon=150.,
        min_extra_loiter=4., max_extra_loiter=10.)
    plan = _candidate(snapshot, config, 0, np.random.default_rng(7))
    assert plan is not None and validate_individual(plan, snapshot, config)
    assert plan.release_time == config.activation_time + config.min_extra_loiter


def test_original_five_aircraft_short_mission_recovers_without_relaxing_inputs():
    from itertools import combinations
    from swarm_sim.trajectory2d.planning import continuous_distance_lower_bound, required_distance
    config = Config(dimensions=3, numerical_backend='python', generator_mode='physical_mission', target_altitude=30.,
        max_descent_rate=10., descent_angle_deg=30., cruise_altitude_range=(30., 120.),
        planning_strategy='adaptive', initial_candidates=2, search_order='mrv',
        selection_policy='max_margin', conflict_guidance='witness', obstacle_routing='spatial')
    snapshots = tuple(replace(s, altitude=30. + 20 * k) for k, s in enumerate(separate_loiter_snapshot(5)))
    result = plan_mission(snapshots, config)
    assert result.validated, result.diagnostics
    assert config.horizon == 90. and config.attempts == 90 and config.candidates == 28
    assert config.position_error == 2. and config.clock_error == .15 and config.separation == 8.
    for plan, snapshot in zip(result.plans, snapshots):
        assert validate_individual(plan, snapshot, config)
        assert plan.horizon == 90. and abs(plan.flight_states[-1, 2] - plan.cruise_altitude) <= .2
    for a, b in combinations(result.plans, 2):
        assert continuous_distance_lower_bound(a, b) >= required_distance(a, b, config)
    from swarm_sim.trajectory2d.execution import simulate
    from swarm_sim.trajectory2d.perturbations import position_bias_vectors
    report = simulate(result.plans, config, wind=.1, wind_model='gust', wind_heading_deg=15.,
        clock_offsets=np.linspace(-.03, .03, 5), clock_drifts=np.linspace(-.0001, .0001, 5),
        position_biases=position_bias_vectors([p.drone_id for p in result.plans], .05, 3),
        position_noise=.01, disturbance_seed=23).report
    assert report['execution_validated'], report['execution_failures']
    assert report['physical_limits_verified'] and not report['safety_events']


def test_rich_profile_executes_nominal_and_bounded_sensor_gust_trials():
    from swarm_sim.trajectory2d.execution import simulate
    snapshot, config = mission_case(speed=12., target=(500., 500.), max_descent_rate=10.,
        horizon=210., cruise_altitude_range=(30., 120.), cruise_levels=2,
        speed_profile='smooth', speed_profile_range=(10., 14.), attempts=6)
    result = plan_mission((snapshot,), config)
    assert result.validated, result.diagnostics
    for settings in ({}, {'wind': .1, 'wind_model': 'gust', 'wind_heading_deg': 15.,
                          'clock_offsets': [-.03], 'clock_drifts': [-.0001],
                          'position_biases': [(0.03, 0., 0.)], 'position_noise': .01,
                          'disturbance_seed': 23}):
        report = simulate(result.plans, config, **settings).report
        assert report['execution_validated'], report['execution_failures']


def test_obstacle_after_target_is_avoided_through_the_entire_recovery():
    from swarm_sim.trajectory2d.obstacles import Obstacle, plan_clearance, nominal_clearance_required
    obstacle = Obstacle.cylinder(700., 0., 30., 0., 180.)
    snapshot, config = mission_case(speed=12., horizon=180., max_descent_rate=10.,
        obstacles=(obstacle,), obstacle_routing='spatial', attempts=6)
    result = plan_mission((snapshot,), config)
    assert result.validated, result.diagnostics
    plan, = result.plans
    assert validate_individual(plan, snapshot, config)
    assert plan_clearance(plan, config) >= nominal_clearance_required(plan.speed_bound, config)
    onward = plan.flight_states[round(plan.arrival_time / config.dt):]
    assert onward[:, 0].max() > 750. and np.max(np.abs(onward[:, 1])) > 30.


@pytest.mark.parametrize('candidate_id,zmin,zmax,mode', [(1, 0., 40., 'overflight'),
                                                      (2, 150., 180., 'underflight')])
def test_physical_spatial_routes_keep_integrated_obstacle_clearance(candidate_id, zmin, zmax, mode):
    from swarm_sim.trajectory2d.obstacles import Obstacle
    from swarm_sim.trajectory2d.planning import _candidate
    snapshot, config = mission_case(speed=12., max_descent_rate=10.,
        obstacles=(Obstacle.cylinder(250., 0., 25., zmin, zmax),), obstacle_routing='spatial')
    plan = _candidate(snapshot, config, candidate_id, np.random.default_rng(7))
    assert plan is not None and plan.routing_mode == mode
    assert validate_individual(plan, snapshot, config)


@pytest.mark.parametrize('drone_id', [0, 3])
def test_free_obstructed_approach_explores_without_overriding_an_assigned_sector(drone_id):
    from pathlib import Path
    from swarm_sim.px4_fleet import load_fleet
    from swarm_sim.trajectory2d.planning import _candidate
    snapshots, config = load_fleet(Path(__file__).resolve().parents[1]/'examples/scenarios/px4_four_loiter.json')
    config = replace(config, generator_mode='physical_mission')
    snapshot = snapshots[drone_id]
    def propose(angle=None):
        rng = np.random.default_rng(np.random.SeedSequence([config.seed, snapshot.drone_id]))
        return _candidate(snapshot, config, 5, rng, approach_angle=angle)
    plan = propose()
    assert plan is not None and validate_individual(plan, snapshot, config)
    assert plan.approach_angle is None and config.approach_spread_deg == 0
    assert config.attempts == 90 and config.obstacle_clearance == 5
    assigned = float(np.arctan2(config.target[1]-snapshot.state[1], config.target[0]-snapshot.state[0])-np.pi)
    restricted = propose(assigned)
    diagnostic = {}
    assert restricted is not None and restricted.approach_angle == assigned
    assert not validate_individual(restricted, snapshot, config, diagnostics=diagnostic)
    assert diagnostic['rejection'] == 'obstacle_clearance'


def test_physical_rich_grid_selection_retains_independent_full_horizon_checks():
    from itertools import combinations
    from swarm_sim.trajectory2d.reservations import occupancy, first_conflict
    from swarm_sim.trajectory2d.planning import continuous_distance_lower_bound, required_distance
    _, config = mission_case(speed=12., target=(500., 500.), max_descent_rate=10.,
        horizon=240., cruise_altitude_range=(30., 120.), cruise_levels=2,
        speed_profile='smooth', speed_profile_range=(10., 14.), planning_strategy='adaptive',
        initial_candidates=2, candidates=2, attempts=8, approach_spread_deg=180.,
        reservation_policy='grid', interval_guidance='safe_windows')
    snapshots = tuple(replace(s, altitude=80. + 10 * k) for k, s in enumerate(separate_loiter_snapshot(2)))
    result = plan_mission(snapshots, config)
    assert result.validated, result.diagnostics
    assert result.diagnostics['reservations']['full_horizon_verified']
    for snapshot, plan in zip(snapshots, result.plans):
        assert validate_individual(plan, snapshot, config)
    for a, b in combinations(result.plans, 2):
        assert continuous_distance_lower_bound(a, b) >= required_distance(a, b, config)
        assert first_conflict(occupancy(a, config), occupancy(b, config)) is None
