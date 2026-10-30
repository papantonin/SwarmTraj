"""Fixed-wing profile data must be complete, immutable, truthful and explicit."""

from dataclasses import FrozenInstanceError, asdict, replace
import hashlib
import json
from pathlib import Path
import pickle

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, Result, load_aircraft_profile, reference_aircraft_profile
from swarm_sim.trajectory2d.aircraft_profiles import aircraft_profile_from_dict, PerformanceLimits
from swarm_sim.trajectory2d.flight3d import envelope_ok, trim_control
from swarm_sim.trajectory2d.planning import mission_document


ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT / 'examples/aircraft/fixed_wing_reference.json'
TRAINING = ROOT / 'examples/aircraft/fixed_wing_training.json'


@pytest.fixture
def data():
    return reference_aircraft_profile().to_dict()


def test_file_reference_matches_factory_and_historical_effective_parameters():
    profile = load_aircraft_profile(REFERENCE)
    assert profile == reference_aircraft_profile()
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', max_descent_rate=10)
    assert profile.apply(config) == config
    assert pickle.loads(pickle.dumps(profile)) == profile


def test_apply_preserves_mission_fields_and_enforces_new_physical_limits():
    profile = load_aircraft_profile(TRAINING)
    original = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target=(210, -70), horizon=120, max_descent_rate=10,
                      separation=12, position_error=1, clock_error=.2, max_altitude=160)
    bound = profile.apply(original)
    for name, value in asdict(original).items():
        if name not in profile.physical_config():
            assert asdict(bound)[name] == value
    assert bound.limits.min_speed == 9 and bound.limits.max_speed == 15
    assert bound.limits.max_bank_deg == 35 and bound.flight.max_brake_acceleration == 0
    assert bound.max_climb_rate == 2 and bound.descent_rate_limit == 4
    assert original.flight.max_brake_acceleration == 5
    state = np.array([0., 0., 100., 12., 0., 0., np.deg2rad(40)])
    control = trim_control(state, original.flight)
    assert envelope_ok(state, control, original) and not envelope_ok(state, control, bound)


def test_profile_is_immutable_and_records_are_isolated():
    profile = load_aircraft_profile(REFERENCE)
    before = profile.sha256
    with pytest.raises(FrozenInstanceError):
        profile.name = 'changed'
    with pytest.raises(FrozenInstanceError):
        profile.flight.max_brake_acceleration = 0
    record = profile.record()
    record['profile']['limits']['max_airspeed_m_s'] = 999
    assert profile.sha256 == before and profile.limits.max_speed == 16


def test_canonical_digest_normalizes_numbers_and_ignores_paths(data, tmp_path):
    first = tmp_path / 'first.json'
    second = tmp_path / 'second.json'
    first.write_text(json.dumps(data))
    data['limits']['max_airspeed_m_s'] = 16
    second.write_text(json.dumps(data, indent=4, sort_keys=True))
    a, b = load_aircraft_profile(first), load_aircraft_profile(second)
    assert a.sha256 == b.sha256
    canonical = json.dumps(a.to_dict(), sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    assert hashlib.sha256(canonical).hexdigest() == a.sha256
    first.write_text('this file changed after loading')
    assert a.record() == b.record()


@pytest.mark.parametrize('field,value', (('schema', 'future'), ('model', 'six_dof'), ('unit_system', 'imperial'),
    ('name', ''), ('name', True), ('description', ''), ('calibration_status', 'certified')))
def test_invalid_metadata_rejected(data, field, value):
    data[field] = value
    with pytest.raises(ValueError):
        aircraft_profile_from_dict(data)


@pytest.mark.parametrize('section,field,value', (
    ('limits', 'min_airspeed_m_s', 0), ('limits', 'max_airspeed_m_s', 7),
    ('limits', 'stall_speed_1g_m_s', 20), ('limits', 'stall_margin', .9),
    ('limits', 'min_load_factor', 1.1), ('limits', 'max_load_factor', .9),
    ('limits', 'max_bank_deg', 90), ('limits', 'max_climb_rate_m_s', 0),
    ('limits', 'max_descent_rate_m_s', -1), ('limits', 'max_vertical_acceleration_m_s2', 0),
    ('limits', 'min_airspeed_m_s', True), ('limits', 'max_airspeed_m_s', '16'),
    ('actuators', 'max_thrust_acceleration_m_s2', 0),
    ('actuators', 'max_airbrake_acceleration_m_s2', -1),
    ('drag', 'parasitic_coefficient_per_m', float('nan')),
    ('drag', 'induced_coefficient_m3_per_s4', float('inf')),
))
def test_invalid_physical_parameters_rejected(data, section, field, value):
    data[section][field] = value
    with pytest.raises(ValueError):
        aircraft_profile_from_dict(data)


@pytest.mark.parametrize('section,field', ((None, 'mass_kg'), ('limits', 'wingspan_m'), ('drag', 'cd0')))
def test_unused_physical_fields_rejected_not_ignored(data, section, field):
    (data if section is None else data[section])[field] = 1
    with pytest.raises(ValueError, match='unknown'):
        aircraft_profile_from_dict(data)


def test_missing_or_wrong_shape_fields_rejected(data):
    del data['limits']['stall_margin']
    with pytest.raises(ValueError, match='missing'):
        aircraft_profile_from_dict(data)
    with pytest.raises(ValueError):
        aircraft_profile_from_dict([])
    with pytest.raises(ValueError):
        aircraft_profile_from_dict({0: 1})


@pytest.mark.parametrize('text', ('{"name":"a","name":"b"}', 'NaN', '{"x":Infinity}', '[]', '{'))
def test_malformed_or_ambiguous_json_rejected(tmp_path, text):
    path = tmp_path / 'bad.json'
    path.write_text(text)
    with pytest.raises(ValueError):
        load_aircraft_profile(path)


def test_overflow_encoding_and_file_size_rejected(data, tmp_path):
    path = tmp_path / 'bad.json'
    path.write_text(json.dumps(data).replace('16.0', '1e10000'))
    with pytest.raises(ValueError):
        load_aircraft_profile(path)
    path.write_bytes(b'\xff')
    with pytest.raises(ValueError):
        load_aircraft_profile(path)
    path.write_bytes(b' ' * (1024 * 1024 + 1))
    with pytest.raises(ValueError, match='1 MiB'):
        load_aircraft_profile(path)


def test_python_numeric_validation_rejects_booleans_and_overflow():
    with pytest.raises(ValueError):
        PerformanceLimits(max_climb_rate=True)
    with pytest.raises(ValueError):
        PerformanceLimits(max_climb_rate=10**10000)


@pytest.mark.parametrize('config', (Config(), Config(dimensions=3, generator_mode='geometric', numerical_backend='python', flight_model='legacy')))
def test_unsupported_runtime_backend_rejected(config):
    with pytest.raises(ValueError, match='coupled 3D'):
        reference_aircraft_profile().apply(config)


def test_manifest_includes_matching_profile_before_hash_and_legacy_default_unchanged():
    profile = load_aircraft_profile(REFERENCE)
    config = profile.apply(Config(dimensions=3, generator_mode='geometric', numerical_backend='python'))
    result = Result('no_candidate', 'Synthetic serialization test')
    original = mission_document(result, config, ())
    assert 'aircraft_profile' not in original
    document = mission_document(result, config, (), aircraft_profile=profile)
    assert document['aircraft_profile'] == profile.record()
    digest = document.pop('sha256')
    assert digest == hashlib.sha256(json.dumps(document, sort_keys=True, allow_nan=False).encode()).hexdigest()
    assert digest != original['sha256']
    with pytest.raises(ValueError, match='differs'):
        mission_document(result, replace(config, max_climb_rate=2), (), aircraft_profile=profile)


def test_profile_identity_and_physical_parameters_remain_separate(data):
    first = aircraft_profile_from_dict(data)
    data['name'] = 'same_physics_other_identity'
    second = aircraft_profile_from_dict(data)
    assert first.sha256 != second.sha256 and first.physical_config() == second.physical_config()
