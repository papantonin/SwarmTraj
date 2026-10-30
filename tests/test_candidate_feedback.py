"""Rejection feedback adjusts references only; physical checks remain authoritative."""

from dataclasses import replace
import pytest

from swarm_sim.trajectory2d import Config
from swarm_sim.trajectory2d.obstacles import Obstacle


def rich_config(**kw):
    values = dict(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', cruise_altitude_range=(30, 120), cruise_levels=2,
                  obstacles=(Obstacle.box(100, 100, 0, 120, 120, 100),))
    return Config(**(values | kw))










@pytest.mark.parametrize('kw', [dict(candidate_guidance='unknown'),
                              dict(candidate_guidance='rejections', dimensions=2, cruise_levels=1,
                                   cruise_altitude_range=None)])
def test_invalid_candidate_guidance(kw):
    with pytest.raises(ValueError):
        rich_config(**kw)


def test_geometric_headroom_warning_is_not_a_physical_infeasibility_claim():
    from swarm_sim.trajectory2d.dynamics import Snapshot
    from swarm_sim.trajectory2d.mission_timing import timing_diagnostics
    from swarm_sim.trajectory2d.catalog import Catalog
    from swarm_sim.trajectory2d.vertical import descent_error
    config = rich_config(target_altitude=30, descent_angle_deg=30, max_descent_rate=10)
    snapshot = Snapshot(3, (0, 0, 0, 0), 13, 105)
    assert descent_error(snapshot, config) is None
    diagnostic = timing_diagnostics(config, [Catalog()], [snapshot])
    row = diagnostic['geometric_template'][0]
    assert row['conservative_template_limitation']
    assert row['geometric_recovery_drop_m'] > row['available_height_m']
    assert row['coupled_preflight_recovery_drop_m'] < row['available_height_m']
    assert 'not physical infeasibility' in diagnostic['advice'][0]
    higher = timing_diagnostics(replace(config, target_altitude=45), [Catalog()], [snapshot])
    assert not higher['geometric_template'][0]['conservative_template_limitation']






@pytest.mark.parametrize('values', [dict(level_hold_override=1), dict(level_hold_override=float('nan')),
                                     dict(routing_extra_margin=-1), dict(routing_extra_margin=float('inf'))])
def test_invalid_reference_adjustments_are_rejected(values):
    import numpy as np
    from swarm_sim.trajectory2d.planning import _candidate
    from swarm_sim.trajectory2d import separate_loiter_snapshot
    diagnostic = {}
    assert _candidate(separate_loiter_snapshot()[0], rich_config(), 0, np.random.default_rng(7),
                      diagnostics=diagnostic, **values) is None
    assert diagnostic['rejection'] == 'invalid_proposal_adjustment'


def test_four_aircraft_feedback_is_spawn_reproducible_and_physically_executes():
    from swarm_sim.trajectory2d import plan_mission, separate_loiter_snapshot
    from swarm_sim.trajectory2d.execution import simulate
    from swarm_sim.trajectory2d.obstacles import load_obstacles
    from swarm_sim.trajectory2d.planning import validate_individual, mission_document
    c = rich_config(seed=19, target=(500, 500), target_altitude=30, horizon=240, activation_time=30,
        descent_angle_deg=30, max_descent_rate=10, speed_profile='smooth', speed_profile_range=(10, 14),
        planning_strategy='adaptive', initial_candidates=2, search_order='mrv', selection_policy='max_margin',
        approach_spread_deg=180, obstacle_routing='spatial', reservation_policy='grid',
        interval_guidance='safe_windows', conflict_guidance='witness',
        candidate_guidance='rejections', obstacles=load_obstacles('examples/scenarios/static_obstacles.json'))
    snapshots = tuple(replace(s, altitude=30+25*k) for k, s in enumerate(separate_loiter_snapshot(4)))
    serial = plan_mission(snapshots, c)
    parallel = plan_mission(snapshots[::-1], replace(c, planning_workers=2))
    assert serial.validated and parallel.validated, (serial.status, serial.diagnostics)
    assert serial.diagnostics == parallel.diagnostics
    assert [p.to_dict() for p in serial.plans] == [p.to_dict() for p in parallel.plans]
    recipes = [p for cat in serial.diagnostics['catalogs'] for p in cat['candidate_proposals']]
    assert recipes and any(p['accepted'] for p in recipes)
    assert all(cat['attempted'] <= c.attempts for cat in serial.diagnostics['catalogs'])
    for plan, snapshot in zip(serial.plans, snapshots):
        assert len(plan.cruise_levels_m) == 2 and plan.speed_reference is not None
        assert validate_individual(plan, snapshot, c)
    assert mission_document(serial, c, snapshots) == mission_document(replace(serial, timings={'arbitrary': 42}), c, snapshots)
    report = simulate(serial.plans, c).report
    assert report['execution_validated'], report['execution_failures']
