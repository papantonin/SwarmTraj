"""Rich references must be physically rolled out and independently checked."""

from dataclasses import replace
import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.planning import validate_individual
from swarm_sim.trajectory2d.rich_profiles import speed_schedule


@pytest.mark.parametrize('values', [
    {'interval_guidance': 'unknown'}, {'interval_guidance': 'safe_windows'},
    {'interval_max_constraints': 0}, {'cruise_levels': 0}, {'cruise_levels': True},
    {'cruise_levels': 4}, {'cruise_levels': 2}, {'cruise_level_hold': 1},
    {'speed_profile': 'bad'}, {'speed_profile': 'smooth'}, {'speed_profile_acceleration': 0},
    {'speed_profile_range': (7, 14)}, {'speed_profile_range': (14, 10)},
    {'speed_profile_range': (float('nan'), 14)},
])
def test_invalid_profile_configuration(values):
    with pytest.raises(ValueError):
        Config(**values)


def test_speed_bump_is_smooth_bounded_and_keeps_holding_and_terminal_speed():
    from types import SimpleNamespace
    proposal = SimpleNamespace(speed=12, release_time=10, arrival_time=80, drone_id=0, candidate_id=0)
    config = Config(dimensions=3, speed_profile='smooth', speed_profile_range=(10, 14))
    fine = np.linspace(0, 100, 10001)
    speed, derivative = speed_schedule(proposal, config, fine)
    assert speed.min() >= 10 and speed.max() <= 14
    assert np.ptp(speed) > .2
    assert np.max(np.abs(derivative * speed / 12)) <= config.speed_profile_acceleration
    assert np.all(speed[fine <= 14] == 12) and np.all(speed[fine >= 62] == 12)
    assert derivative[1400] == derivative[6200] == 0
    # The speed is C1, not C2 at the two knots: centered differences there
    # have O(h), not O(h²), truncation error. Check the smooth interior and
    # separately check the exact zero derivatives at the joins above.
    interior = (fine > 14.01) & (fine < 61.99)
    np.testing.assert_allclose(np.gradient(speed, fine)[interior], derivative[interior], atol=2e-5)


@pytest.fixture(scope='module')
def scene():
    config = Config(dimensions=3, target=(500, 500), target_altitude=30, horizon=210,
                    activation_time=30, descent_angle_deg=30, max_descent_rate=10,
                    cruise_altitude_range=(30, 120), cruise_levels=2,
                    speed_profile='smooth', speed_profile_range=(10, 14),
                    planning_strategy='adaptive', initial_candidates=2, search_order='mrv', approach_spread_deg=180)
    snapshots = tuple(replace(s, altitude=30 + 25*k) for k, s in enumerate(separate_loiter_snapshot(4)))
    result = plan_mission(snapshots, config)
    assert result.validated, (result.status, result.diagnostics)
    return config, snapshots, result


def test_four_aircraft_actually_reach_multiple_levels_and_vary_physical_speed(scene):
    config, snapshots, result = scene
    for snapshot, plan in zip(snapshots, result.plans):
        assert validate_individual(plan, snapshot, config)
        assert len(plan.cruise_levels_m) == 2
        assert np.ptp(plan.speed_reference) > .2
        assert np.ptp(plan.flight_states[:, 3]) > .2
        assert not plan.speed_reference.flags.writeable
        assert plan.to_dict()['cruise_levels_m'] == list(plan.cruise_levels_m)
        assert plan.release_time >= config.activation_time
        assert np.allclose(plan.flight_states[0, :3], snapshot.position3d)


@pytest.mark.parametrize('tamper', ['levels', 'reference', 'acceleration'])
def test_independent_validation_rejects_false_profile_metadata(scene, tamper):
    config, snapshots, result = scene
    plan = result.plans[0]
    if tamper == 'levels':
        plan = replace(plan, cruise_levels_m=(plan.cruise_altitude, 999))
    elif tamper == 'reference':
        plan = replace(plan, speed_reference=np.full_like(plan.speed_reference, 12.))
    else:
        reference = plan.speed_reference.copy()
        reference[10] += 1
        plan = replace(plan, speed_reference=reference)
    assert not validate_individual(plan, snapshots[0], config)


def test_rich_interval_guided_four_drone_planning_is_spawn_reproducible_and_executes():
    from swarm_sim.trajectory2d.obstacles import load_obstacles
    from swarm_sim.trajectory2d.execution import simulate
    config = Config(dimensions=3, target=(500, 500), target_altitude=30, horizon=240,
                    activation_time=30, descent_angle_deg=30, max_descent_rate=10,
                    cruise_altitude_range=(30, 120), cruise_levels=2,
                    speed_profile='smooth', speed_profile_range=(10, 14),
                    planning_strategy='adaptive', initial_candidates=2, search_order='mrv',
                    approach_spread_deg=180, selection_policy='max_margin', conflict_guidance='witness',
                    obstacle_routing='spatial', reservation_policy='grid', interval_guidance='safe_windows',
                    obstacles=load_obstacles('examples/scenarios/static_obstacles.json'))
    snapshots = tuple(replace(s, altitude=30 + 25*k) for k, s in enumerate(separate_loiter_snapshot(4)))
    serial = plan_mission(snapshots, config)
    parallel = plan_mission(snapshots[::-1], replace(config, planning_workers=2))
    assert serial.validated and parallel.validated
    assert serial.diagnostics == parallel.diagnostics
    assert [p.to_dict() for p in serial.plans] == [p.to_dict() for p in parallel.plans]
    timed = [p for c in serial.diagnostics['catalogs'] for p in c['interval_proposals']]
    assert timed and any(p['accepted'] for p in timed)
    for p in timed:
        assert config.activation_time <= p['requested_departure_s'] <= config.activation_time + config.max_extra_loiter
    assert all(not p.speed_reference.flags.writeable for p in parallel.plans)
    assert serial.diagnostics['reservations']['full_horizon_verified']
    execution = simulate(serial.plans, config)
    assert execution.report['execution_validated'], execution.report['execution_failures']


def test_three_level_geometric_profiles_keep_exact_grid_kinematics():
    from swarm_sim.trajectory2d.vertical import build_profile
    config = Config(dimensions=3, horizon=180, cruise_altitude_range=(40, 120), cruise_levels=3,
                    target_altitude=30, max_descent_rate=10)
    snapshot = replace(separate_loiter_snapshot()[0], altitude=80, cruise_altitude=80)
    info = {}
    result = build_profile(snapshot, config, 3, 130, np.random.default_rng(7), profile_info=info)
    assert result is not None and len(info['levels']) == 3
    states, commands = result
    np.testing.assert_allclose(states[1:, 0], states[:-1, 0] + states[:-1, 1]*config.dt + .5*commands*config.dt**2)
    np.testing.assert_allclose(states[1:, 1], states[:-1, 1] + commands*config.dt, atol=1e-12)
