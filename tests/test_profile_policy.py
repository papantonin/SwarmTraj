"""Mixed families are explicit, bounded, reproducible and strictly validated."""

from dataclasses import replace
import pickle

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.catalog import Catalog, extend_catalog
from swarm_sim.trajectory2d.planning import validate_individual, mission_document
from swarm_sim.trajectory2d.profile_policy import families, family_config


def mixed_config(**kw):
    values = dict(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', target=(500, 500), target_altitude=30, horizon=240,
                  activation_time=30, cruise_altitude_range=(30, 120), cruise_levels=2,
                  speed_profile='smooth', speed_profile_range=(10, 14),
                  planning_strategy='adaptive', initial_candidates=2, search_order='mrv',
                  profile_policy='adaptive_mixed')
    return Config(**(values | kw))


@pytest.mark.parametrize('kw', [dict(profile_policy='other'), dict(dimensions=2),
                                    dict(flight_model='legacy'), dict(planning_strategy='fixed')])
def test_invalid_mixed_policy(kw):
    with pytest.raises(ValueError):
        mixed_config(**kw)


def test_allowed_families_change_only_reference_requirements():
    config = mixed_config(cruise_levels=3)
    assert [f.name for f in families(config)] == [
        'baseline', 'levels_2', 'levels_3', 'speed', 'levels_2_speed', 'levels_3_speed']
    for family in families(config):
        effective = family_config(config, family)
        assert effective == replace(config, profile_policy='strict', cruise_levels=family.levels,
                                    speed_profile=family.speed)
        assert effective.flight == config.flight and effective.limits == config.limits
        assert effective.horizon == config.horizon and effective.separation == config.separation
        assert effective.speed_profile_range == config.speed_profile_range


@pytest.mark.parametrize('levels,speed,names', [
    (1, 'constant', ['baseline']),
    (2, 'constant', ['baseline', 'levels_2']),
    (1, 'smooth', ['baseline', 'speed']),
])
def test_no_family_is_enabled_without_explicit_permission(levels, speed, names):
    config = mixed_config(cruise_levels=levels, speed_profile=speed)
    assert [f.name for f in families(config)] == names










@pytest.fixture(scope='module')
def mixed_scene():
    from swarm_sim.trajectory2d.obstacles import load_obstacles
    config = mixed_config(descent_angle_deg=30, max_descent_rate=10, approach_spread_deg=180,
                          obstacles=load_obstacles('examples/scenarios/static_obstacles.json'),
                          obstacle_routing='spatial', selection_policy='max_margin',
                          conflict_guidance='witness', reservation_policy='grid', interval_guidance='safe_windows')
    snapshots = tuple(replace(s, altitude=30 + 25*k) for k, s in enumerate(separate_loiter_snapshot(4)))
    result = plan_mission(snapshots, config)
    assert result.validated, (result.status, result.diagnostics)
    return config, snapshots, result


def test_four_aircraft_mixed_plans_are_independently_valid_and_immutable(mixed_scene):
    config, snapshots, result = mixed_scene
    proposed = [p for c in result.diagnostics['catalogs'] for p in c['profile_proposals']]
    assert proposed and any(p['family'] != 'baseline' for p in proposed)
    assert all(c['attempted'] <= config.attempts for c in result.diagnostics['catalogs'])
    for plan, snapshot in zip(result.plans, snapshots):
        assert validate_individual(plan, snapshot, config)
        assert plan.to_dict()['profile_family'] == plan.profile_family
        transferred = pickle.loads(pickle.dumps(plan))
        assert transferred.to_dict() == plan.to_dict()
        assert not transferred.flight_states.flags.writeable
        if transferred.speed_reference is not None:
            assert not transferred.speed_reference.flags.writeable
    assert result.diagnostics['reservations']['full_horizon_verified']
    assert mission_document(result, config, snapshots) == mission_document(
        replace(result, timings={'arbitrary': 999}), config, snapshots)


@pytest.mark.parametrize('declaration', [None, 'unknown', 'levels_3', 'levels_3_speed'])
def test_missing_or_unpermitted_declarations_fail_independent_validation(mixed_scene, declaration):
    config, snapshots, result = mixed_scene
    diagnostic = {}
    assert not validate_individual(replace(result.plans[0], profile_family=declaration),
                                   snapshots[0], config, diagnostic)
    assert diagnostic['rejection'] == 'validation_profile_family'


def test_declaring_a_richer_family_does_not_bypass_attainment_checks(mixed_scene):
    config, snapshots, result = mixed_scene
    # Missing physical profile evidence must fail even for an allowed label.
    plan = replace(result.plans[0], profile_family='levels_2_speed', cruise_levels_m=(), speed_reference=None)
    assert not validate_individual(plan, snapshots[0], config)
    assert not validate_individual(result.plans[0], snapshots[0], replace(config, profile_policy='strict'))


def test_four_aircraft_mixed_mode_is_spawn_reproducible_and_executes(mixed_scene):
    from swarm_sim.trajectory2d.execution import simulate
    config, snapshots, serial = mixed_scene
    parallel = plan_mission(snapshots[::-1], replace(config, planning_workers=2))
    assert parallel.validated and parallel.diagnostics == serial.diagnostics
    assert [p.to_dict() for p in serial.plans] == [p.to_dict() for p in parallel.plans]
    report = simulate(serial.plans, config).report
    assert report['execution_validated'], report['execution_failures']
    assert report['selected_profile_families'] == [p.profile_family for p in serial.plans]
