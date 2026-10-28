"""Simulation-only disappearance contracts; never launches/arms an autopilot."""

import csv
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from swarm_sim import px4_termination as termination
from swarm_sim.px4_fleet import pair_observations
from swarm_sim.px4_reference import LocalOrigin, mission_items
from swarm_sim.px4_scene import scene_tree
from swarm_sim import px4_gazebo as single
from swarm_sim import px4_fleet as fleet
from test_px4_fleet import small_document


def test_mode_is_fingerprinted_and_original_plan_is_not_modified():
    doc = small_document()
    original = deepcopy(doc)
    result = termination.with_end_behavior(doc, 'remove-at-target')
    assert result['sha256'] != doc['sha256']
    assert doc == original
    assert termination.end_behavior(doc) == 'land'
    assert termination.end_behavior(result) == 'remove-at-target'
    with pytest.raises(ValueError):
        termination.with_end_behavior(doc, 'ignore-errors')


def test_fleet_target_removal_never_runs_landing_screen(monkeypatch):
    snapshots, config = fleet.default_fleet()
    monkeypatch.setattr(fleet, 'plan_mission', lambda *a: SimpleNamespace(validated=True))
    monkeypatch.setattr(fleet, 'mission_document', lambda *a, **k: small_document())
    monkeypatch.setattr(fleet, 'select_recovery', lambda *a, **k: pytest.fail('No landing screen in target-removal mode'))
    modes = []
    monkeypatch.setattr(fleet, 'mission_items', lambda *a, **k: modes.append(k['end_behavior']))
    doc = fleet.prepare_fleet(snapshots, config, 'remove-at-target')
    assert modes == ['remove-at-target']*4
    assert termination.end_behavior(doc) == 'remove-at-target'


def test_worker_contact_helper_is_not_shadowed_by_report_arrays():
    # The live loop and final statistics share a function scope. A report-local
    # array previously masked this callable and failed only on the first live row.
    assert 'target_distance' not in single.execute_worker.__code__.co_varnames


def test_reference_prefix_has_flythrough_tail_but_preserves_full_plan():
    doc = {'simulation_end_behavior': 'remove-at-target', 'plans': [
        {'dt': 1., 'arrival_time': 10., 'flight_states': [[i, 0, 80, 15, 0, 0, 0] for i in range(30)]}]}
    before = deepcopy(doc)
    points = termination.flight_reference(doc)
    assert len(points) == 16 and points[-1, 0] == 15
    points[0, 0] = -100
    assert doc == before


@pytest.mark.parametrize('mode', ['land', 'remove-at-target'])
def test_preview_has_finite_fixed_bounds_before_3d_collections(tmp_path, monkeypatch, mode):
    from swarm_sim import px4_scenario
    from swarm_sim.trajectory2d import obstacle_view
    doc = termination.with_end_behavior(small_document(), mode)
    doc['config']['obstacles'] = [{'kind': 'cylinder', 'center': [200, 120, 75], 'half_extents': [100, 100, 75]}]
    original = obstacle_view.draw_obstacles_3d
    visited = []

    def draw(axis, config):
        assert not any((axis.get_autoscalex_on(), axis.get_autoscaley_on(), axis.get_autoscalez_on()))
        assert np.all(np.isfinite([axis.get_xlim(), axis.get_ylim(), axis.get_zlim()]))

        def forbidden(*args, **kwargs):
            pytest.fail('Preview must not autoscale empty 3D collections')

        monkeypatch.setattr(axis, '_autoscale_axis', forbidden)
        visited.append(True)
        original(axis, config)

    monkeypatch.setattr(obstacle_view, 'draw_obstacles_3d', draw)
    output = tmp_path/'preview.png'
    px4_scenario.save_preview(output, doc)
    assert visited and output.stat().st_size > 0


def test_no_landing_commands_or_onward_visuals_in_trajectory_only_scene():
    doc = termination.with_end_behavior(small_document(), 'remove-at-target')
    tree = scene_tree(doc, single.ROOT/'examples/gazebo/worlds/default.sdf')
    names = [m.get('name') for m in tree.getroot().findall('world/model')]
    assert 'swarm_reference_arrival' in names and 'swarm_reference_onward' not in names
    points = np.array([[0., 0, 80], [100., 0, 80]])
    items, indices = mission_items(points, LocalOrigin(47, 8, 0), end_behavior='remove-at-target')
    assert [item['command'] for item in items] == [22, 178, *[16]*len(indices)]
    assert not {21, 31, 189} & {item['command'] for item in items}
    with pytest.raises(ValueError, match='landing pattern'):
        mission_items(points, LocalOrigin(47, 8, 0), end_behavior='remove-at-target', recovery={})


def test_contact_uses_actual_xyz_radius_and_explicit_xy_mode():
    config = {'target': [0, 0], 'target_altitude': 80, 'target_radius': 6}
    assert termination.target_distance([0, 0, 90], config) == 10
    assert termination.target_distance([3, 4, 80], config) == 5
    assert termination.target_distance([0, 0, 90], {**config, 'target_altitude': None}) == 0
    with pytest.raises(ValueError):
        termination.target_distance([np.nan, 0, 80], config)


def removal_environment(monkeypatch):
    monkeypatch.setenv('SWARM_MANAGED_SIM', '1')
    monkeypatch.setenv('SWARM_SITL_WORKER', '1')
    monkeypatch.setenv('SWARM_FLEET_SIZE', '4')
    monkeypatch.setenv('SWARM_FLEET_INDEX', '2')
    monkeypatch.setattr(termination, 'Path', lambda p: SimpleNamespace(exists=lambda: True))


def test_removal_requires_own_isolated_model_and_confirmed_absence(monkeypatch):
    removal_environment(monkeypatch)
    calls = []
    ground = 'model {\n name: "ground_plane"\n}\n'
    replies = iter([ground+'model {\n name: "rc_cessna_lidar_2"\n}',
                    'data: true', ground+'model {\n name: "rc_cessna_lidar_1"\n}'])

    def run(args, **kwargs):
        calls.append(args)
        return SimpleNamespace(stdout=next(replies))

    monkeypatch.setattr(termination.subprocess, 'run', run)
    with pytest.raises(ValueError):
        termination.remove_simulated_aircraft(1)
    assert not calls
    report = termination.remove_simulated_aircraft(2)
    assert report['absence_verified'] and not report['landing_performed']
    assert calls[1][-1] == 'name: "rc_cessna_lidar_2", type: MODEL'
    assert len(calls) == 3


@pytest.mark.parametrize('reply', ['data: false', '', 'transport error'])
def test_removal_ack_failure_never_counts_as_arrival(monkeypatch, reply):
    removal_environment(monkeypatch)
    replies = iter(['model {\n name: "ground_plane"\n}\nmodel {\n name: "rc_cessna_lidar_2"\n}', reply])
    monkeypatch.setattr(termination.subprocess, 'run', lambda *a, **k: SimpleNamespace(stdout=next(replies)))
    with pytest.raises(RuntimeError, match='rejected'):
        termination.remove_simulated_aircraft(2)


def test_acknowledged_but_still_present_model_fails_removal(monkeypatch):
    removal_environment(monkeypatch)
    scene = 'model {\n name: "ground_plane"\n}\nmodel {\n name: "rc_cessna_lidar_2"\n}'
    replies = iter([scene, 'data: true', scene])
    times = iter([0., 6.])
    monkeypatch.setattr(termination.time, 'monotonic', lambda: next(times))
    monkeypatch.setattr(termination.subprocess, 'run', lambda *a, **k: SimpleNamespace(stdout=next(replies)))
    with pytest.raises(TimeoutError, match='still present'):
        termination.remove_simulated_aircraft(2)


def test_empty_scene_is_not_evidence_of_a_removed_aircraft(monkeypatch):
    removal_environment(monkeypatch)
    calls = []

    def run(*args, **kwargs):
        calls.append(args)
        return SimpleNamespace(stdout='')

    monkeypatch.setattr(termination.subprocess, 'run', run)
    with pytest.raises(RuntimeError, match='Cannot verify'):
        termination.remove_simulated_aircraft(2)
    assert len(calls) == 1  # Never send removal after an invalid scene reply.


def test_disappearing_first_aircraft_does_not_hide_later_pair_conflicts(tmp_path):
    for index in range(4):
        directory = tmp_path/f'drone-{index}'
        directory.mkdir()
        with (directory/'telemetry.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['sim_time_s', 'east_m', 'north_m', 'up_m'])
            for t in range(2 if index == 0 else 5):
                x = 1000 if index == 0 else 2000 if index == 3 else (t-3)*(1 if index == 1 else -1)
                writer.writerow([t, x, 0, 80])
    historical = pair_observations(tmp_path, .5)
    active = pair_observations(tmp_path, .5, active_only=True)
    assert historical['sampled_separation_passed']
    assert not active['sampled_separation_passed']
    pair = next(p for p in active['pairs'] if p['drones'] == [1, 2])
    assert pair['minimum_interpolated_distance_m'] == pytest.approx(0, abs=1e-10)
    assert pair['observed_interval_s'] == [0, 4]
