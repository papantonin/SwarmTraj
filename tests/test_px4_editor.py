"""Shared scenario/editor/mission contracts; never arms or starts Docker."""

from dataclasses import replace
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pytest

from swarm_sim import px4_gazebo as adapter
from swarm_sim.px4_scenario import (default_scenario, validate_scenario, scenario_record,
                                   load_scenario, loiter_record, derived_loiter,
                                   select_recovery, loiter_observations)
from swarm_sim.px4_reference import LocalOrigin, generate_reference, mission_items
from swarm_sim.trajectory2d.obstacles import Obstacle
from swarm_sim.trajectory2d.scenario_editor import ScenarioEditor


def test_shared_scenario_roundtrip_keeps_all_positions_and_loiter(tmp_path):
    snapshots, config = default_scenario(19, (Obstacle.box(600, 200, 0, 650, 250, 100),))
    snapshots = (replace(snapshots[0], state=(-200., 30., .7, -.15)),)
    config = replace(config, target=(750., -300.), target_altitude=65.,
                     arrival_deadline=140., post_arrival_time=10.)
    record = scenario_record(snapshots, config)
    path = tmp_path/'scene.json'
    path.write_text(json.dumps(record, allow_nan=False))
    actual = load_scenario(path)
    assert actual == (snapshots, config)
    assert actual[0][0].state == (-200., 30., .7, -.15)
    assert actual[1].obstacles[0].half_extents == (25, 25, 50)
    assert actual[1].arrival_deadline == 140 and actual[1].post_arrival_time == 10
    # Older v1 exports predate the two optional timing fields.
    del record['mission']['arrival_deadline']
    del record['mission']['post_arrival_time']
    path.write_text(json.dumps(record))
    legacy = load_scenario(path)[1]
    assert legacy.arrival_deadline is None and legacy.post_arrival_time == 5


def test_recovery_screen_tries_alternative_and_rejects_exhausted_proposals():
    points = np.array([[1000., 500., 80.]])
    offset = np.zeros(3)
    clear = select_recovery(points, (), offset)
    np.testing.assert_allclose(clear['center_enu_m'], [-600, 0, 60])
    obstacle = Obstacle('cylinder', (-600, 0, 60), (120, 120, 100))
    alternative = select_recovery(points, (obstacle,), offset)
    assert alternative['center_enu_m'] != clear['center_enu_m']
    assert alternative['screened_minimum_distance_m'] >= 30
    assert alternative['actual_px4_clearance_validated'] is False
    blocking = Obstacle('box', (0, 0, 0), (2000, 2000, 1000))
    with pytest.raises(ValueError, match='does not prove'):
        select_recovery(points, (blocking,), offset)


def test_reported_cylinder_scene_expands_recovery_without_relaxing_buffers():
    scene_path = Path(__file__).resolve().parents[1]/'examples/scenarios/px4_cylinder_800_200.json'
    snapshots, config = load_scenario(scene_path)
    assert config.obstacles[-1] == Obstacle('cylinder', (800, 200, 75), (125, 125, 75))
    # Fixed end of the independently integrated seed-19 reference for this
    # scene. This regression isolates return selection from planner timing.
    xyz = np.array([[1787.22610918, 818.83356778, 80.]])
    offset = np.array([300., 0., 0.])
    result = select_recovery(xyz-offset, config.obstacles, offset)
    assert result == select_recovery(xyz-offset, config.obstacles, offset)
    assert result['proposals_tested'] == 8
    assert len(result['rejected_proposals']) == 7
    assert [r['heading_deg'] for r in result['rejected_proposals'][:4]] == [0, 90, 180, 270]
    assert result['selection'] == {'heading_deg': 112.5, 'distance_m': 600.}
    assert result['geometry_screen_buffer_m'] == 30
    assert result['screened_minimum_distance_m'] > 55
    assert not result['actual_px4_clearance_validated']
    # Historically rejected joins retain the 100 m join buffer, rather than
    # getting accepted by reducing it to the point-mass margin.
    assert result['rejected_proposals'][1]['obstacle_index'] == 2
    assert result['rejected_proposals'][1]['screened_minimum_distance_m'] < 8
    items, _ = mission_items(np.array([[0, 0, 80.], [1000, 500, 80.]]),
                             LocalOrigin(47, 8, 0), recovery=result)
    assert items[-1]['param4'] == 337.5
    assert items[-2]['param2'] == 100
    native_center = LocalOrigin(47, 8, 0).to_enu(
        items[-2]['x']/1e7, items[-2]['y']/1e7, items[-2]['z'])
    np.testing.assert_allclose(native_center, result['center_enu_m'], atol=.02)


def test_recovery_failure_has_bounded_search_diagnostic_and_no_weaker_buffer():
    blocked = Obstacle('box', (0, 0, 0), (2000, 2000, 1000))
    with pytest.raises(ValueError, match='48 proposals.*obstacle 0.*30 m required'):
        select_recovery(np.array([[1000., 500., 80.]]), (blocked,), np.zeros(3))
    with pytest.raises(ValueError, match='ground footprint'):
        select_recovery(np.array([[4000., 0., 80.]]), (), np.zeros(3))
    for clearance in (0, 29, math.nan):
        with pytest.raises(ValueError, match='at least 30'):
            select_recovery(np.array([[1000., 500., 80.]]), (), np.zeros(3), clearance)


@pytest.mark.parametrize('rings, expected_distance', [((500,), 800), ((500, 700), 1000)])
def test_recovery_search_can_expand_distance_without_changing_landing_height(rings, expected_distance):
    obstacles = tuple(Obstacle('cylinder',
        (radius*math.cos(k*math.pi/8), radius*math.sin(k*math.pi/8), 75), (15, 15, 75))
        for radius in rings for k in range(16))
    result = select_recovery(np.array([[1500., 1000., 80.]]), obstacles, np.zeros(3))
    assert result['selection']['distance_m'] == expected_distance
    assert result['proposals_tested'] > 16
    assert result['center_enu_m'][2] == 60
    assert result['screened_minimum_distance_m'] >= 30


@pytest.mark.parametrize('ccw', [False, True])
def test_loiter_experiment_requires_direction_duration_and_geometry(ccw):
    record = {'center_world_m': [300, -100, 80], 'radius_m': 100,
              'counter_clockwise': ccw, 'duration_s': 40, 'airspeed_m_s': 15}
    def rows(sign=1, duration=40, radius=100):
        return [{'mission_current': 2, 'sim_time_s': float(t),
                 'east_m': 300+radius*math.cos(sign*(1 if ccw else -1)*.15*t),
                 'north_m': -100+radius*math.sin(sign*(1 if ccw else -1)*.15*t),
                 'up_m': 80.} for t in np.linspace(0, duration, 401)]
    observed = loiter_observations(rows(), record, 0)
    assert observed['verified_in_px4_experiment']
    assert not observed['time_aligned_reference_following']
    assert not loiter_observations(rows(sign=-1), record, 0)['verified_in_px4_experiment']
    assert not loiter_observations(rows(duration=20), record, 0)['verified_in_px4_experiment']
    assert not loiter_observations(rows(radius=130), record, 0)['verified_in_px4_experiment']
    assert not loiter_observations(rows(), record, 50)['verified_in_px4_experiment']


@pytest.mark.parametrize('change', ({'speed': 25}, {'altitude': 0}, {'state': (0, 0, 0, 0)},
                                  {'state': (0, 0, 0, 1)}, {'drone_id': 1}))
def test_backend_limits_are_explicit_not_silently_changed(change):
    snapshots, config = default_scenario()
    with pytest.raises(ValueError):
        validate_scenario((replace(snapshots[0], **change),), config)


def test_unknown_scenario_fields_and_multiple_aircraft_rejected(tmp_path):
    snapshots, config = default_scenario()
    with pytest.raises(ValueError, match='one drone'):
        validate_scenario(snapshots*2, config)
    record = scenario_record(snapshots, config)
    record['mission']['connection_url'] = 'udp://hardware'
    path = tmp_path/'bad.json'
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='schema'):
        load_scenario(path)


@pytest.fixture(scope='module')
def shared_plan():
    return generate_reference(scenario=default_scenario(19))


def test_native_loiter_uses_same_circle_duration_speed_and_direction(shared_plan):
    document, times, points = shared_plan
    record = loiter_record(document)
    assert record == {'center_world_m': [300, -100, 80], 'radius_m': 100,
                      'counter_clockwise': False, 'duration_s': 40, 'airspeed_m_s': 15}
    offset = np.array([300, 0, 0])
    origin = LocalOrigin(47, 8, 0)
    items, indices = mission_items(points-offset, origin,
        loiter=derived_loiter(record, offset, document['config']['dt']), allow_vertical=True)
    assert [v['command'] for v in items[:3]] == [22, 178, 19]
    hold = items[2]
    assert (hold['param1'], hold['param2'], hold['param3'], hold['param4']) == (40, 0, 100, 1)
    xyz = origin.to_enu(hold['x']/1e7, hold['y']/1e7, hold['z'])+offset
    np.testing.assert_allclose(xyz, record['center_world_m'], atol=.02)
    assert min(indices)*document['config']['dt'] > record['duration_s']
    assert indices[-1] == len(points)-1
    assert items[-3]['seq'] == 3+len(indices)
    before = points.copy()
    opposite = {**record, 'counter_clockwise': True}
    opposite_items, _ = mission_items(points-offset, origin, loiter=derived_loiter(opposite, offset, .2))
    assert opposite_items[2]['param3'] == -100
    np.testing.assert_array_equal(before, points)


def test_general_editor_waypoints_keep_altitudes_without_enabling_steep_dive():
    points = np.array([[0, 0, 80], [100, 0, 75], [200, 0, 70]])
    with pytest.raises(ValueError, match='level'):
        mission_items(points, LocalOrigin(47, 8, 0))
    items, indices = mission_items(points, LocalOrigin(47, 8, 0), allow_vertical=True)
    for item, index in zip(items[2:-3], indices):
        assert item['z'] == pytest.approx(points[index, 2], abs=.01)
    snapshots, config = default_scenario()
    with pytest.raises(ValueError, match='0°'):
        validate_scenario(snapshots, replace(config, descent_angle_deg=10))


def test_editor_can_add_move_resize_and_remove_obstacles_transactionally():
    snapshots, config = default_scenario()
    editor = ScenarioEditor(snapshots, config.target, config, profile_loading=False)
    try:
        editor.add_obstacle('cylinder')
        assert editor.selected == 2
        editor.place((650., 250.))
        editor.heading_input.set_val('40')  # radius
        editor.cruise_input.set_val('55')  # half-height
        editor.altitude_input.set_val('55')
        assert editor.apply_fields()
        assert editor.config.obstacles == (Obstacle('cylinder', (650, 250, 55), (40, 40, 55)),)
        good = editor.config
        editor.heading_input.set_val('-1')
        assert not editor.apply_fields() and editor.config is good
        editor.add_obstacle('box')
        editor.place((900., 350.))
        editor.heading_input.set_val('20')
        editor.speed_input.set_val('30')
        assert editor.apply_fields()
        assert editor.config.obstacles[1].half_extents[:2] == (20, 30)
        assert editor.remove_obstacle() and len(editor.config.obstacles) == 1
        editor.reset()
        assert not editor.config.obstacles
        assert tuple(editor.snapshots) == snapshots
    finally:
        plt.close(editor.figure)


def test_editor_loiter_fields_and_failed_planning_keep_window_open():
    snapshots, config = default_scenario()
    def failed(*args):
        raise ValueError('catalog_exhausted: test rejection, not physical infeasibility')
    editor = ScenarioEditor(snapshots, config.target, config, launch_validator=failed, profile_loading=False)
    try:
        assert not editor.profile_button.active
        editor.radius_input.set_val('120')
        editor.loiter_direction.set_active(0)
        editor.activation_input.set_val('50')
        assert editor.apply_fields()
        assert editor.snapshots[0].state[3] == pytest.approx(15/120)
        assert editor.config.activation_time == 50
        editor.start()
        assert not editor.accepted and 'catalog_exhausted' in editor.status.get_text()
        editor.reset()
        assert editor.snapshots[0].state[3] == -.15
    finally:
        plt.close(editor.figure)


def test_saved_scenario_launch_is_same_plan_and_no_external_write_offline(monkeypatch, tmp_path):
    scenario = default_scenario(19, (Obstacle('cylinder', (200, 120, 75), (100, 100, 75)),))
    path = tmp_path/'input.json'
    path.write_text(json.dumps(scenario_record(*scenario)))
    monkeypatch.setattr(adapter, 'managed_run', lambda *a, **k: pytest.fail('offline must not launch Docker'))
    output = tmp_path/'run'
    assert adapter.main(['--scenario', str(path), '--output', str(output)]) == 0
    reference = json.loads((output/'reference.json').read_text())
    mission = json.loads((output/'point_mass_mission.json').read_text())
    assert reference['airborne_start'] and reference['editor_scenario']
    assert reference['seed'] == 19 and len(mission['plans']) == 1
    assert reference['loiter'] == loiter_record(mission)
    assert load_scenario(output/'scenario.json') == scenario
    scene = json.loads((output/'scene.json').read_text())
    assert scene['obstacle_count'] == 1
    np.testing.assert_array_equal(reference['positions_enu_m'], np.array(mission['plans'][0]['flight_states'])[:, :3])


def test_editor_cancellation_launches_nothing(monkeypatch, tmp_path):
    import swarm_sim.trajectory2d.scenario_editor as ui
    monkeypatch.setattr(ui, 'edit_scenario', lambda *a, **k: None)
    monkeypatch.setattr(adapter, 'managed_run', lambda *a, **k: pytest.fail('cancelled'))
    assert adapter.main(['--edit-scenario', '--run', '--output', str(tmp_path/'cancelled')]) == 0
    assert not (tmp_path/'cancelled').exists()
