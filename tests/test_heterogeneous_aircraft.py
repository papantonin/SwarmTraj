"""Per-ID physics binding, conservative body geometry and own-plan isolation."""

from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import pickle
import runpy
import sys

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, load_aircraft_profile, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.aircraft_profiles import aircraft_profile_from_dict, reference_aircraft_profile
from swarm_sim.trajectory2d.catalog import Catalog, extend_catalog
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.execution_safety import flight_diagnostics
from swarm_sim.trajectory2d.flight3d import trim_control
from swarm_sim.trajectory2d.obstacles import Obstacle, nominal_clearance_required, obstacle_preflight, execution_clearance
from swarm_sim.trajectory2d.planning import mission_document, required_distance, validate_individual
from swarm_sim.trajectory2d.reservations import occupancy


ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT / 'examples/aircraft/fixed_wing_reference.json'
TRAINING = ROOT / 'examples/aircraft/fixed_wing_training.json'
RASCAL = ROOT / 'examples/aircraft/rascal110_empty_approx.json'


def mixed_config(**kwargs):
    a, b = load_aircraft_profile(REFERENCE), load_aircraft_profile(TRAINING)
    return Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', aircraft_profiles=((0, a), (1, b), (2, a), (3, b)), **kwargs)


def snapshots():
    return tuple(replace(s, altitude=30 + 25 * s.drone_id, cruise_altitude=30 + 25 * s.drone_id)
                 for s in separate_loiter_snapshot())


@pytest.fixture(scope='module')
def mission():
    config = mixed_config(candidates=2, attempts=24)
    initial = snapshots()
    result = plan_mission(initial, config)
    assert result.validated, (result.status, result.diagnostics)
    return initial, config, result


def test_local_binding_is_immutable_picklable_and_contains_no_peers():
    config = mixed_config()
    local = config.for_drone(1)
    assert not local.aircraft_profiles and local.flight.max_brake_acceleration == 0
    assert local.limits.max_bank_deg == 35 and local.max_climb_rate == 2
    assert local.target == config.target and local.separation == config.separation
    assert config.flight.max_brake_acceleration == 5
    assert pickle.loads(pickle.dumps(config)) == config
    with pytest.raises(ValueError, match='no aircraft profile'):
        config.for_drone(9)
    with pytest.raises(ValueError):
        config.profile_for(0).apply(config)


@pytest.mark.parametrize('bindings', ([(0, reference_aircraft_profile())],
    ((True, reference_aircraft_profile()),), ((-1, reference_aircraft_profile()),),
    ((0, reference_aircraft_profile()), (0, reference_aircraft_profile())), ((0, {}),)))
def test_invalid_bindings_rejected(bindings):
    with pytest.raises(ValueError):
        Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', aircraft_profiles=bindings)


def test_exact_membership_and_each_initial_envelope_are_required():
    config = mixed_config()
    with pytest.raises(ValueError, match='exactly match'):
        plan_mission(snapshots()[:-1], config)
    profile = replace(config.profile_for(1), limits=replace(config.profile_for(1).limits, min_speed=13))
    config = replace(config, aircraft_profiles=tuple((i, profile if i == 1 else p) for i, p in config.aircraft_profiles))
    assert plan_mission(snapshots(), config).status == 'invalid_initial_state'


def test_shared_speed_permission_cannot_exceed_any_airframe():
    with pytest.raises(ValueError, match='airspeed limits'):
        mixed_config(speed_profile='smooth', speed_profile_range=(8, 16))


def test_four_drones_validate_against_their_own_profiles(mission):
    initial, config, result = mission
    assert len(result.plans) == 4
    for snapshot, plan in zip(initial, result.plans):
        profile = config.profile_for(plan.drone_id)
        assert plan.aircraft_profile == profile
        assert plan.flight_parameters == profile.flight and plan.speed_bound == profile.limits.max_speed
        assert validate_individual(plan, snapshot, config)
        restored = pickle.loads(pickle.dumps(plan))
        assert restored.aircraft_profile == profile and not restored.flight_states.flags.writeable
        wrong = replace(plan, aircraft_profile=config.profile_for(1 - plan.drone_id % 2))
        assert not validate_individual(wrong, snapshot, config)


def test_heterogeneous_execution_checks_per_aircraft_and_rejects_profile_swap(mission):
    _, config, result = mission
    evaluation = simulate(result.plans, config)
    assert evaluation.report['execution_validated'], evaluation.report['execution_failures']
    assert len(evaluation.report['aircraft_profiles']) == 4
    assert evaluation.report['maximum_airbrake_acceleration_m_s2'][1] == 0
    assert evaluation.report['maximum_airbrake_acceleration_m_s2'][3] == 0
    wrong = (replace(result.plans[0], aircraft_profile=config.profile_for(1)), *result.plans[1:])
    with pytest.raises(ValueError, match='profile'):
        simulate(wrong, config)


def test_spawn_retains_identity_and_exact_candidate_arrays(mission):
    from swarm_sim.trajectory2d.parallel import generate_catalogs
    initial, config, _ = mission
    tasks = [(s, config, None, Catalog(), 1) for s in initial]
    serial = generate_catalogs(extend_catalog, tasks, 1)
    # Fresh catalogs; do not continue the already mutated serial task inputs.
    spawned = generate_catalogs(extend_catalog, [(s, config, None, Catalog(), 1) for s in initial], 2)
    assert all(c.plans for c in serial) and all(c.plans for c in spawned)
    for a, b in zip(serial, spawned):
        assert a.attempted == b.attempted
        assert a.plans[0].to_dict() == b.plans[0].to_dict()
        assert not b.plans[0].flight_states.flags.writeable


def test_metadata_is_hashed_and_local_payload_configuration_has_no_fleet(mission):
    initial, config, result = mission
    document = mission_document(result, config, initial)
    digest = document.pop('sha256')
    assert hashlib.sha256(json.dumps(document, sort_keys=True, allow_nan=False).encode()).hexdigest() == digest
    for plan in result.plans:
        own = asdict(config.for_drone(plan.drone_id))
        assert own['aircraft_profiles'] == ()
        assert plan.to_dict()['aircraft_profile'] == config.profile_for(plan.drone_id).record()


def test_geometry_v2_round_trip_and_strict_validation():
    profile = load_aircraft_profile(RASCAL)
    assert profile.bounding_radius_m == 2.5
    assert aircraft_profile_from_dict(profile.to_dict()) == profile
    assert profile.to_dict()['schema'] == 'swarm-aircraft-profile-v2'
    assert reference_aircraft_profile().to_dict()['schema'] == 'swarm-aircraft-profile-v1'
    for radius in (0, -1, True, '2', float('nan')):
        data = profile.to_dict()
        data['geometry']['bounding_radius_m'] = radius
        with pytest.raises(ValueError):
            aircraft_profile_from_dict(data)
    for geometry in ({}, {'bounding_radius_m': 2, 'wingspan': 3}):
        data = profile.to_dict()
        data['geometry'] = geometry
        with pytest.raises(ValueError):
            aircraft_profile_from_dict(data)


def test_body_radii_inflate_pair_obstacle_and_reservation_bounds(mission):
    initial, config, result = mission
    sizes = (2., 3., 1., 4.)
    enlarged = replace(config, aircraft_profiles=tuple((i, replace(p, bounding_radius_m=sizes[i]))
                                                       for i, p in config.aircraft_profiles))
    a, b = result.plans[:2]
    assert required_distance(a, b, enlarged) - required_distance(a, b, config) == 5
    assert occupancy(a, enlarged).inflation_m - occupancy(a, config).inflation_m == 2
    local = enlarged.for_drone(1)
    assert nominal_clearance_required(15, local) == config.obstacle_clearance + 3 + config.position_error + 15 * config.clock_error
    point = initial[1].position3d
    obstacle = Obstacle('box', tuple(point + (7., 0., 0.)), (1., 1., 1.))
    blocked = replace(enlarged, obstacles=(obstacle,))
    assert obstacle_preflight(initial, blocked)[0] == 'initial_obstacle_violation'
    # Center clearance 6 m exceeds the old 5 m rule, but not 5 + radius 3.
    positions = np.broadcast_to(point, (2, 1, 3))
    observed = execution_clearance(positions, (b,), blocked, .02, 0)
    assert not observed['obstacle_clearance_verified_in_execution_model']


def test_observer_does_not_use_common_bank_limit_for_every_drone():
    config = mixed_config()
    states = np.tile([0., 0., 100., 12., 0., 0., np.deg2rad(40)], (2, 4, 1))
    controls = np.tile(np.stack([trim_control(states[0, k], config.for_drone(k).flight) for k in range(4)]), (2, 1, 1))
    plans = [type('Plan', (), {'drone_id': k})() for k in range(4)]
    boundaries = [(np.array([0., 1.]), states[:, k], controls[:, k]) for k in range(4)]
    details, events = flight_diagnostics(states, controls, np.array([0., 1.]), plans, config, boundaries)
    assert {e['drone_id'] for e in events if e['constraint'] == 'bank'} == {1, 3}
    assert details['minimum_flight_envelope_margins_per_drone']['bank'][0] > 0


def test_cli_binds_profiles_before_planning_and_keeps_mission_requests(monkeypatch, tmp_path):
    launcher = runpy.run_path(str(ROOT / 'examples/trajectory_2d.py'))
    class Reached(Exception):
        pass
    def inspect(initial, config):
        assert len(initial) == 4 and config.for_drone(1).flight.max_brake_acceleration == 0
        assert config.target_altitude is None and config.descent_angle_deg == 0
        raise Reached
    monkeypatch.setitem(launcher['main'].__globals__, 'plan_mission', inspect)
    options = [v for i, path in enumerate((REFERENCE, TRAINING, REFERENCE, TRAINING))
               for v in ('--drone-profile', str(i), str(path))]
    monkeypatch.setattr(sys, 'argv', ['trajectory_2d.py', '--no-gui', '--arrival-mode', 'layers',
                                    '--output', str(tmp_path), *options])
    with pytest.raises(Reached):
        launcher['main']()
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('options', (
    ['--drone-profile', '0', str(REFERENCE)],
    ['--drone-profile', '-1', str(REFERENCE)],
    ['--drone-profile', 'x', str(REFERENCE)],
    ['--drone-profile', '0', str(REFERENCE), '--aircraft-profile', str(REFERENCE)],
    ['--drone-profile', '0', str(REFERENCE), '--max-bank', '50'],
))
def test_cli_invalid_assignments_write_nothing(monkeypatch, tmp_path, options):
    launcher = runpy.run_path(str(ROOT / 'examples/trajectory_2d.py'))
    monkeypatch.setattr(sys, 'argv', ['trajectory_2d.py', '--no-gui', '--output', str(tmp_path), *options])
    with pytest.raises(SystemExit) as error:
        launcher['main']()
    assert error.value.code == 2 and not list(tmp_path.iterdir())
