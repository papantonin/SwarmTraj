"""Differential independent native validation, including deliberate corruptions."""
from dataclasses import replace
import importlib.util

import numpy as np
import pytest

from swarm_sim.trajectory2d import plan_mission
from swarm_sim.trajectory2d.planning import validate_individual
from test_native_backend import case

pytestmark = pytest.mark.skipif(importlib.util.find_spec('swarm_sim._trajectory_native') is None,
                               reason='optional C++ extension not built')


@pytest.fixture(scope='module', params=['baseline', 'rich', 'obstacle', 'profile'])
def scene(request):
    changes = dict(numerical_backend='cpp', attempts=6)
    if request.param == 'rich':
        changes.update(cruise_levels=3, speed_profile='smooth', speed_profile_range=(10., 14.),
            horizon=210., target=(500., 500.), cruise_altitude_range=(30., 120.), max_descent_rate=10.)
    elif request.param == 'obstacle':
        from swarm_sim.trajectory2d.obstacles import Obstacle
        changes.update(obstacles=(Obstacle.cylinder(700., 0., 30., 0., 180.),), obstacle_routing='spatial')
    s, c = case(**changes)
    if request.param == 'profile':
        from swarm_sim.trajectory2d.aircraft_profiles import reference_aircraft_profile
        profile = replace(reference_aircraft_profile(), name='native validator own aircraft', bounding_radius_m=1.2)
        c = replace(c, aircraft_profiles=((s.drone_id, profile),))
    result = plan_mission((s,), c)
    assert result.validated, (result.status, result.counts)
    return s, c, result.plans[0]


def compare(plan, s, c):
    a, b = {}, {}
    py = validate_individual(plan, s, c, a, backend='python')
    cpp = validate_individual(plan, s, c, b, backend='cpp')
    assert py == cpp, (a, b)
    assert a.get('rejection') == b.get('rejection'), (a, b)
    assert a.get('rejection_detail') == b.get('rejection_detail'), (a, b)
    if 'first_contact_time_s' in a:
        assert b['first_contact_time_s'] == pytest.approx(a['first_contact_time_s'], abs=1e-9)
    return cpp, b


def test_original_and_tampered_history_projection_metadata(scene):
    s, c, p = scene
    assert compare(p, s, c)[0]
    changes = [dict(drone_id=999), dict(arrival_time=np.nan), dict(release_time=0.),
        dict(arrival_time=c.horizon), dict(cruise_altitude=None),
        dict(cruise_altitude=-1.), dict(approach_angle=1.), dict(airspeed_limit=17.),
        dict(profile_family='undeclared'), dict(flight_parameters=replace(c.for_drone(p.drone_id).flight, parasitic_drag=.004))]
    for field, index, delta in [('flight_states', (1, 0), 1e-5),
            ('flight_states', (-1, 0), 1.), ('states', (1, 0), .01),
            ('vertical_states', (1, 0), .01), ('accelerations', (0,), .01),
            ('vertical_accelerations', (0,), .01), ('flight_controls', (-1, 2), 1.)]:
        array = getattr(p, field).copy(); array[index] += delta
        changes.append({field: array})
    for field in ('flight_states', 'flight_controls'):
        array = getattr(p, field).copy(); array[2, 0] = np.nan
        changes.append({field: array})
    for change in changes:
        assert not compare(replace(p, **change), s, c)[0], change.keys()


def test_requested_profiles_are_checked_against_actual_states(scene):
    s, c, p = scene
    if c.cruise_levels > 1:
        for levels in ((), (p.cruise_levels_m[0],)*3, p.cruise_levels_m[::-1]):
            assert not compare(replace(p, cruise_levels_m=levels), s, c)[0]
        reference = p.speed_reference.copy(); reference[len(reference)//2] = 100.
        assert not compare(replace(p, speed_reference=reference), s, c)[0]
        assert not compare(replace(p, speed_reference=None), s, c)[0]
    else:
        assert not compare(replace(p, cruise_levels_m=(80.,)), s, c)[0]
        assert not compare(replace(p, speed_reference=np.full(len(p.states), p.speed)), s, c)[0]


def test_post_arrival_obstacle_and_unresolved_contact_are_failures(scene):
    s, c, p = scene
    from swarm_sim.trajectory2d.obstacles import Obstacle
    x, y, z = p.flight_states[-1, :3]
    blocked = replace(c, obstacles=(Obstacle.box(x-2, y-2, z-2, x+2, y+2, z+2),))
    assert compare(p, s, blocked)[1]['rejection'] == 'obstacle_clearance'
    if not c.aircraft_profiles:
        large_bound = replace(c, limits=replace(c.limits, max_speed=1e9))
        oversized = replace(p, airspeed_limit=1e9)
        accepted, diagnostic = compare(oversized, s, large_bound)
        assert not accepted and diagnostic['rejection'] == 'validation_contact_event_unresolved'


def test_native_validation_does_not_call_python_rollout(scene, monkeypatch):
    s, c, p = scene
    import swarm_sim.trajectory2d.flight3d as flight
    monkeypatch.setattr(flight, 'propagate', lambda *a, **kw: pytest.fail('native validation called Python dynamics'))
    assert validate_individual(p, s, c, backend='cpp')
