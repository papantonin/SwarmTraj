"""Four-aircraft contracts; no test launches Docker or arms an autopilot."""

from dataclasses import replace
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from swarm_sim import px4_fleet as fleet
from swarm_sim import px4_gazebo as single
from swarm_sim.px4_scenario import load_scenario


def test_fleet_schema_roundtrip_and_single_scene_stays_strict(tmp_path):
    snapshots, config = fleet.default_fleet()
    path = tmp_path/'fleet.json'
    path.write_text(json.dumps(fleet.fleet_record(snapshots, config)))
    assert fleet.load_fleet(path) == (snapshots, config)
    with pytest.raises(ValueError):
        load_scenario(path)
    with pytest.raises(ValueError):
        fleet.validate_fleet(snapshots[:3], config)
    with pytest.raises(ValueError):
        fleet.validate_fleet((snapshots[0],)*4, config)
    with pytest.raises(ValueError):
        fleet.validate_fleet((replace(snapshots[0], speed=25), *snapshots[1:]), config)
    overlapping = replace(snapshots[1], state=snapshots[0].state, altitude=90)
    with pytest.raises(ValueError, match='ground spawn'):
        fleet.validate_fleet((snapshots[0], overlapping, *snapshots[2:]), config)
    record = fleet.fleet_record(snapshots, config)
    record['mission']['connection_url'] = 'udp://hardware'
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError):
        fleet.load_fleet(path)


def small_document():
    snapshots, config = fleet.default_fleet()
    plans = [{'drone_id': s.drone_id, 'dt': .2, 'arrival_time': .2, 'release_time': 40,
              'flight_states': [[*s.state[:2], s.altitude, s.speed, s.state[2], 0, -.225],
                                [s.state[0]+1, s.state[1], s.altitude, s.speed, 0, 0, -.225],
                                [s.state[0]+2, s.state[1], s.altitude, s.speed, 0, 0, -.225]]}
             for s in snapshots]
    scenario = fleet.fleet_record(snapshots, config)
    doc = {'plans': plans, 'snapshots': scenario['snapshots'], 'config': {**scenario['mission'], 'dt': .2}}
    doc['sha256'] = fleet.canonical_hash(doc)
    return json.loads(json.dumps(doc))


def test_shared_scene_has_four_unique_models_and_reference_annotations():
    document = small_document()
    before = json.dumps(document, sort_keys=True)
    tree = fleet.fleet_scene(document, single.ROOT/'examples/gazebo/worlds/default.sdf')
    world = tree.getroot().find('world')
    names = [node.get('name') for node in world.findall('model')]
    assert len(names) == len(set(names))
    assert sum(name.startswith('rc_cessna_lidar_') for name in names) == 4
    assert sum(name.startswith('swarm_reference_') for name in names) == 8
    assert names.count('swarm_target') == 1
    for index in range(4):
        model = world.find(f"model[@name='rc_cessna_lidar_{index}']")
        assert model.find('plugin/module_name').text == 'airborne_bootstrap'
        assert float(model.find('pose').text.split()[0]) == document['snapshots'][index]['state'][0]
    assert json.dumps(document, sort_keys=True) == before


def test_worker_projection_and_shared_scene_tampering_rejected(tmp_path):
    document = small_document()
    single.save_json(tmp_path/'point_mass_mission.json', document)
    tree = fleet.fleet_scene(document, single.ROOT/'examples/gazebo/worlds/default.sdf')
    tree.write(tmp_path/'scene.sdf', encoding='utf-8')
    output = tmp_path/'drone-2'
    output.mkdir()
    own = fleet.own_document(document, 2)
    assert len(own['plans']) == 1 and own['plans'][0]['drone_id'] == 2
    assert own['parent_mission_sha256'] == document['sha256']
    own.pop('sha256')
    fleet.verify_worker_scene(output, own, 2)
    own['parent_mission_sha256'] = 'wrong'
    with pytest.raises(ValueError, match='projection'):
        fleet.verify_worker_scene(output, own, 2)


def test_fleet_native_instance_selection_does_not_change_single_mode(monkeypatch):
    monkeypatch.delenv('SWARM_FLEET_SIZE', raising=False)
    assert single.native_command('ver', 'all') == ['/opt/px4-gazebo/bin/px4-ver', 'all']
    monkeypatch.setenv('SWARM_FLEET_SIZE', '4')
    for index in range(4):
        monkeypatch.setenv('SWARM_FLEET_INDEX', str(index))
        assert single.native_command('listener', 'mission_result') == [
            '/opt/px4-gazebo/bin/px4-listener', '--instance', str(index), 'mission_result']
    monkeypatch.setenv('SWARM_FLEET_INDEX', '9')
    with pytest.raises(ValueError):
        single.native_command('ver', 'all')


def test_fleet_container_keeps_network_and_device_isolation():
    args = single.container_arguments('test', Path('/tmp/example'), airborne_start=True, fleet=True)
    assert args[args.index('--network')+1] == 'none'
    assert 'SWARM_FLEET_SIZE=4' in args
    assert args[-1].endswith('/start_fleet.sh')
    assert '--privileged' not in args and '--device' not in args


def test_pair_observer_reports_crossing_not_a_certificate(tmp_path):
    import csv
    for index in range(4):
        path = tmp_path/f'drone-{index}'
        path.mkdir()
        with (path/'telemetry.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['sim_time_s', 'east_m', 'north_m', 'up_m'])
            for t in (0, 1, 2):
                writer.writerow([t, (t-1)*(1 if index == 0 else -1) if index < 2 else 100*index, 0, 80])
    report = fleet.pair_observations(tmp_path, 30.)
    assert report['minimum_interpolated_distance_m'] == pytest.approx(0)
    assert not report['sampled_separation_passed'] and not report['swarm_separation_certified']
    assert len(report['pairs']) == 6


def test_fleet_cancellation_does_not_launch_or_write(monkeypatch, tmp_path):
    import swarm_sim.trajectory2d.scenario_editor as editor
    monkeypatch.setattr(editor, 'edit_scenario', lambda *a, **kw: None)
    monkeypatch.setattr(single, 'managed_run', lambda *a: pytest.fail('cancel must not launch'))
    output = tmp_path/'cancelled'
    assert single.main(['--num-drones', '4', '--airborne-start', '--edit-scenario', '--run', '--output', str(output)]) == 0
    assert not output.exists()
