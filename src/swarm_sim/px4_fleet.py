"""Four-aircraft spatial SITL experiment, not a timed swarm-safety bridge."""

import argparse
import csv
from dataclasses import asdict, replace
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np

from . import px4_gazebo as single
from .px4_reference import LocalOrigin, mission_items
from .px4_scenario import (default_scenario, validate_scenario, MISSION_FIELDS,
    loiter_record, derived_loiter, select_recovery, save_preview)
from .px4_scene import scene_tree
from .trajectory2d import Snapshot, plan_mission
from .trajectory2d.aircraft_profiles import reference_aircraft_profile
from .trajectory2d.obstacles import Obstacle, load_obstacles
from .trajectory2d.planning import mission_document
from .px4_termination import end_behavior, with_end_behavior, flight_reference

SCHEMA = 'swarm-px4-editor-fleet-v2'
LEGACY_SCHEMA = 'swarm-px4-editor-fleet-v1'
COUNT = 4


def default_fleet(seed=19, obstacles=()):
    _, config = default_scenario(seed, obstacles)
    config = replace(config, horizon=200., separation=30., candidates=12, attempts=90,
                     selection_policy='max_margin', margin_expansion_rounds=0)
    snapshots = tuple(Snapshot(k, (x, y, 0., -.15), 15., 60.+10*k, 60.+10*k)
                      for k, (x, y) in enumerate(((300., 0.), (300., -400.),
                                                (-100., 0.), (-100., -400.))))
    return snapshots, config


def validate_fleet(snapshots, config):
    if len(snapshots) != COUNT or [s.drone_id for s in snapshots] != list(range(COUNT)):
        raise ValueError('Four-aircraft experiment requires exactly IDs 0, 1, 2, 3 in order.')
    for snapshot in snapshots:
        validate_scenario((replace(snapshot, drone_id=0),), config)
    for first in range(COUNT):
        for second in range(first+1, COUNT):
            if np.linalg.norm(np.asarray(snapshots[first].state[:2])-snapshots[second].state[:2]) < 30:
                raise ValueError('Warm fleet bootstrap requires at least 30 m XY separation between ground spawn points, even at different flight altitudes.')


def fleet_record(snapshots, config):
    validate_fleet(snapshots, config)
    mission = {name: getattr(config, name) for name in (*MISSION_FIELDS, 'separation')}
    mission['obstacles'] = [asdict(o) for o in config.obstacles]
    return {'schema': SCHEMA, 'snapshots': [asdict(s) for s in snapshots], 'mission': mission}


def load_fleet(path):
    record = json.loads(Path(path).read_text())
    fields = set(MISSION_FIELDS)|{'separation'}
    legacy = isinstance(record, dict) and record.get('schema') == LEGACY_SCHEMA
    if legacy:
        fields.remove('generator_mode')
    if (not isinstance(record, dict) or set(record) != {'schema', 'snapshots', 'mission'}
            or record['schema'] not in (SCHEMA, LEGACY_SCHEMA) or not isinstance(record['snapshots'], list)
            or not isinstance(record['mission'], dict) or set(record['mission']) != fields):
        raise ValueError('Expected swarm-px4-editor-fleet-v1/v2 JSON, not a single-aircraft scene.')
    try:
        snapshots = tuple(Snapshot(**{**s, 'state': tuple(s['state'])}) for s in record['snapshots'])
        settings = dict(record['mission'])
        if legacy:
            settings['generator_mode'] = 'geometric'
        settings['target'] = tuple(settings['target'])
        settings['obstacles'] = tuple(Obstacle(**o) for o in settings['obstacles'])
        if settings['cruise_altitude_range'] is not None:
            settings['cruise_altitude_range'] = tuple(settings['cruise_altitude_range'])
        config = replace(default_fleet()[1], **settings)
        validate_fleet(snapshots, config)
        return snapshots, config
    except (TypeError, KeyError, IndexError) as error:
        raise ValueError('Invalid four-aircraft scenario fields.') from error


def canonical_hash(document):
    return hashlib.sha256(json.dumps(document, sort_keys=True, allow_nan=False).encode()).hexdigest()


def own_document(document, index):
    """Immutable projection; never regenerate or claim independent joint validation."""
    own = {k: v for k, v in document.items() if k != 'sha256'}
    own.update(parent_mission_sha256=document['sha256'],
               snapshots=[document['snapshots'][index]], plans=[document['plans'][index]])
    own['sha256'] = canonical_hash(own)
    return own


def fleet_scene(document, template):
    tree = scene_tree(own_document(document, 0), template, airborne_start=True)
    world = tree.getroot().find('world')
    colors = ('0.02 0.85 1 1', '1 0.25 0.2 1', '0.2 1 0.3 1', '0.9 0.3 1 1')
    for index in range(1, COUNT):
        other = scene_tree(own_document(document, index), template, airborne_start=True)
        for node in other.getroot().find('world').findall('model'):
            name = node.attrib['name']
            if name.startswith('swarm_reference_'):
                node.set('name', name.replace('swarm_reference_', f'swarm_reference_{index}_'))
                if name.endswith('arrival'):
                    for material in node.findall('.//material'):
                        for color in material:
                            color.text = colors[index]
                world.append(node)
            elif name == 'rc_cessna_lidar_0':
                node.set('name', f'rc_cessna_lidar_{index}')
                world.append(node)
    ET.indent(tree, space='  ')
    return tree


def prepare_fleet(snapshots, config, termination='land'):
    validate_fleet(snapshots, config)
    result = plan_mission(tuple(snapshots), config)
    if not result.validated:
        raise ValueError(f'Joint reference generation failed: {result.status}: {result.reason}')
    document = mission_document(result, config, snapshots, aircraft_profile=reference_aircraft_profile())
    document = with_end_behavior(document, termination)
    for index in range(COUNT):
        own = own_document(document, index)
        xyz = flight_reference(own)
        if np.max(np.abs(xyz[:, :2])) > 2800:
            raise ValueError(f'Drone {index}: calculated reference leaves laboratory ground.')
        offset = np.r_[xyz[0, :2], 0.]
        recovery = select_recovery(xyz-offset, config.obstacles, offset) if termination == 'land' else None
        mission_items(xyz-offset, LocalOrigin(47.397971, 8.546164, 0.),
            airspeed=snapshots[index].speed, allow_vertical=True,
            loiter=derived_loiter(loiter_record(own), offset, config.dt), recovery=recovery,
            end_behavior=termination)
    return document


def write_fleet(output, snapshots, config, document, camera_mode='free'):
    output.mkdir(parents=True, exist_ok=False)
    single.save_json(output/'point_mass_mission.json', document)
    single.save_json(output/'scenario.json', fleet_record(snapshots, config))
    scene = fleet_scene(document, single.ROOT/'examples/gazebo/worlds/default.sdf')
    scene.write(output/'scene.sdf', encoding='utf-8', xml_declaration=True)
    single.save_json(output/'scene.json', {'mission_sha256': document['sha256'],
        'scene_sha256': hashlib.sha256((output/'scene.sdf').read_bytes()).hexdigest(),
        'aircraft_count': COUNT, 'obstacle_count': len(config.obstacles),
        'end_behavior': end_behavior(document),
        'swarm_separation_validated_in_px4': False})
    single.save_json(output/'fleet.json', {'schema': 'swarm-px4-fleet-run-v1', 'count': COUNT,
        'camera_mode': camera_mode,
        'end_behavior': end_behavior(document),
        'mission_sha256': document['sha256'], 'source_sha256': single.source_hashes(),
        'timed_tracking_supported': False, 'safety_certificate_transferred': False})
    for index in range(COUNT):
        own = own_document(document, index)
        folder = output/f'drone-{index}'
        folder.mkdir()
        single.save_json(folder/'point_mass_mission.json', own)
        single.save_json(folder/'reference.json', {'schema': 'swarm-px4-spatial-reference-v1',
            'positions_enu_m': np.asarray(own['plans'][0]['flight_states'])[:, :3].tolist(),
            'mission_sha256': own['sha256'], 'parent_mission_sha256': document['sha256'],
            'airborne_start': True, 'editor_scenario': True, 'loiter': loiter_record(own),
            'end_behavior': end_behavior(own),
            'obstacles': document['config']['obstacles'], 'timing_preserved_by_px4_mission': False})
        save_preview(folder/'planning-preview.png', own)


def barrier(output, phase, timeout=120.):
    """Preflight-only filesystem coordination, never in-flight avoidance."""
    root = output.parent
    single.save_json(output/(phase+'.json'), {'ready': True})
    def all_ready():
        if (root/'abort.json').exists():
            raise RuntimeError('Another fleet worker failed; experiment aborted.')
        return all((root/f'drone-{k}'/(phase+'.json')).exists() for k in range(COUNT))
    single.wait_for(all_ready, timeout, f'Fleet startup barrier timed out: {phase}', interval=.01)


def verify_worker_scene(output, own, instance):
    root = output.parent
    document = json.loads((root/'point_mass_mission.json').read_text())
    recorded = document['sha256']
    if canonical_hash({k: v for k, v in document.items() if k != 'sha256'}) != recorded:
        raise ValueError('Fleet mission fingerprint mismatch; no arm command sent.')
    if own != {k: v for k, v in own_document(document, instance).items() if k != 'sha256'}:
        raise ValueError('Own projection differs from the joint plan; no arm command sent.')
    expected = fleet_scene(document, single.ROOT/'examples/gazebo/worlds/default.sdf')
    if ET.tostring(expected.getroot()) != ET.tostring(ET.parse(root/'scene.sdf').getroot()):
        raise ValueError('Shared fleet scene mismatch; no arm command sent.')


def pair_observations(output, separation, active_only=False):
    """Observer interpolation, not a continuous-time distance certificate."""
    trajectories = []
    for index in range(COUNT):
        with (output/f'drone-{index}'/'telemetry.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        data = np.array([[float(r[k]) for k in ('sim_time_s', 'east_m', 'north_m', 'up_m')] for r in rows])
        # Repeated time stamps do not give interpolation intervals.
        data = data[np.r_[True, np.diff(data[:, 0]) > 0]]
        trajectories.append(data)
    low = max(v[0, 0] for v in trajectories)
    high = min(v[-1, 0] for v in trajectories)
    distances = []
    for i in range(COUNT):
        for j in range(i+1, COUNT):
            begin = max(trajectories[i][0, 0], trajectories[j][0, 0]) if active_only else low
            end = min(trajectories[i][-1, 0], trajectories[j][-1, 0]) if active_only else high
            if end <= begin:
                raise ValueError('No common observed fleet interval.')
            grid = np.arange(begin, end, .1)
            xyz = [np.column_stack([np.interp(grid, v[:, 0], v[:, k]) for k in (1, 2, 3)])
                   for v in (trajectories[i], trajectories[j])]
            distances.append({'drones': [i, j],
                'minimum_interpolated_distance_m': float(np.linalg.norm(xyz[0]-xyz[1], axis=1).min()),
                'observed_interval_s': [float(begin), float(end)]})
    minimum = min(v['minimum_interpolated_distance_m'] for v in distances)
    return {'pairs': distances, 'minimum_interpolated_distance_m': minimum,
            'required_experiment_distance_m': separation, 'sampled_separation_passed': minimum >= separation,
            'common_interval_s': [low, high], 'observer_grid_s': .1,
            'pairwise_active_intervals': active_only,
            'swarm_separation_certified': False}


def worker(output, timeout):
    """Own four local worker processes; stop the experiment on any failure."""
    if os.environ.get('SWARM_FLEET_SIZE') != str(COUNT) or not Path('/.dockerenv').exists():
        raise ValueError('Fleet worker is restricted to the managed simulator.')
    processes, logs = [], []
    camera = None
    try:
        for index in range(COUNT):
            log = (output/f'drone-{index}'/'adapter.log').open('x')
            logs.append(log)
            env = {**os.environ, 'SWARM_FLEET_INDEX': str(index),
                   'MPLCONFIGDIR': f'/tmp/swarm-matplotlib-{index}', 'HEADLESS': '1'}
            processes.append(subprocess.Popen(['/workspace/.venv/bin/python', '-m', 'swarm_sim.px4_gazebo',
                '--worker', str(output/f'drone-{index}'), '--timeout', str(timeout)], env=env,
                stdout=log, stderr=subprocess.STDOUT))
        # The window belongs to the fleet supervisor, not four competing workers.
        deadline = time.monotonic()+timeout+180
        gui_started = False
        while any(p.poll() is None for p in processes):
            if any(p.poll() not in (None, 0) for p in processes):
                raise RuntimeError('One fleet worker failed; see per-drone reports/logs.')
            if time.monotonic() > deadline:
                raise TimeoutError('Fleet wall-time budget exceeded.')
            if not gui_started and os.environ.get('HEADLESS') != '1' and all(
                    (output/f'drone-{k}'/'airborne-check.json').exists() for k in range(COUNT)):
                mode = json.loads((output/'fleet.json').read_text()).get('camera_mode', 'free')
                camera_log = (output/'camera.log').open('x')
                logs.append(camera_log)
                camera = subprocess.Popen(['/usr/bin/python3', '/workspace/src/swarm_sim/px4_camera.py',
                    '--output', str(output), '--mode', mode], stdout=camera_log, stderr=subprocess.STDOUT)
                gui_started = True
            time.sleep(.2)
        if any(p.returncode != 0 for p in processes):
            raise RuntimeError('Fleet worker did not pass its experiment gates.')
        removed = end_behavior(json.loads((output/'point_mass_mission.json').read_text())) == 'remove-at-target'
        observations = pair_observations(output, json.loads((output/'scenario.json').read_text())['mission']['separation'], active_only=removed)
        report = {'completed': True, 'aircraft_count': COUNT, 'all_aircraft_landed': not removed,
                  'all_aircraft_removed_at_target': removed,
                  'backend_test_passed': observations['sampled_separation_passed'],
                  'pair_observations': observations, 'timed_tracking_supported': False,
                  'swarm_separation_certified': False, 'safety_certificate_transferred': False}
        single.save_json(output/'report.json', report)
        return 0 if report['backend_test_passed'] else 3
    except BaseException as error:
        single.save_json(output/'abort.json', {'type': type(error).__name__, 'message': str(error)})
        if not (output/'report.json').exists():
            single.save_json(output/'report.json', {'completed': False, 'backend_test_passed': False,
                'aircraft_count': COUNT, 'swarm_separation_certified': False,
                'error': {'type': type(error).__name__, 'message': str(error)}})
        return 3
    finally:
        if camera is not None and camera.poll() is None:
            camera.terminate()
            try:
                camera.wait(timeout=3)
            except subprocess.TimeoutExpired:
                camera.kill()
                camera.wait(timeout=3)
        for process in processes:
            if process.poll() is None:
                process.send_signal(2)
        for process in processes:
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        for log in logs:
            log.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--num-drones', type=int, choices=[4], default=4)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--gpu', action='store_true')
    parser.add_argument('--camera', choices=['free', 'follow'], default='free',
                        help='initial GUI camera mode; keyboard remains available in both modes')
    parser.add_argument('--edit-scenario', action='store_true')
    parser.add_argument('--end-behavior', choices=['remove-at-target', 'land'], default='remove-at-target',
                        help='default: remove each simulated aircraft after actual target contact; no landing')
    parser.add_argument('--airborne-start', action='store_true', help='already implicit for the four-aircraft experiment')
    parser.add_argument('--scenario', type=Path)
    parser.add_argument('--obstacles', type=Path)
    parser.add_argument('--seed', type=int)
    parser.add_argument('--generator-mode', choices=['geometric', 'physical_mission'],
                        help='explicit generator override; saved v1 scenes remain geometric otherwise')
    parser.add_argument('--workers', type=int, help='catalog processes; 0 selects available CPUs')
    parser.add_argument('--timeout', type=float, default=1200)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--worker', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker:
        return worker(args.worker, args.timeout)
    if not math.isfinite(args.timeout) or not 30 <= args.timeout <= 1800:
        parser.error('timeout must be within 30..1800 seconds')
    if args.workers is not None and args.workers < 0:
        parser.error('workers must be nonnegative')
    if (args.gui or args.gpu) and not args.run:
        parser.error('gui/gpu require --run')
    if args.scenario and args.obstacles:
        parser.error('scenario already contains its obstacles')
    output = args.output or single.ROOT/'results/px4-gazebo-fleet'/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    if output.exists():
        parser.error('output must be a new directory')
    try:
        scenario = load_fleet(args.scenario) if args.scenario else default_fleet(
            19 if args.seed is None else args.seed, load_obstacles(args.obstacles) if args.obstacles else ())
        if args.seed is not None:
            scenario = scenario[0], replace(scenario[1], seed=args.seed)
        overrides = {}
        if args.generator_mode is not None:
            overrides['generator_mode'] = args.generator_mode
        if args.workers is not None:
            overrides['planning_workers'] = args.workers
        if overrides:
            scenario = scenario[0], replace(scenario[1], **overrides)
        prepared = []
        def prepare(snapshots, config):
            prepared[:] = [prepare_fleet(snapshots, config, args.end_behavior)]
        if args.edit_scenario:
            from .trajectory2d.scenario_editor import edit_scenario
            scenario = edit_scenario(scenario[0], scenario[1].target, scenario[1],
                                     launch_validator=prepare, profile_loading=False)
            if scenario is None:
                return 0
        if not prepared:
            prepare(*scenario)
        write_fleet(output, *scenario, prepared[0], camera_mode=args.camera)
        print(f'Four-aircraft joint reference generated: {output}', flush=True)
        print('EXPERIMENT: PX4 mission timing and fleet collision freedom are NOT guaranteed.', flush=True)
        if args.run:
            return single.managed_run(output.resolve(), args.gui, args.timeout, args.gpu)
        return 0
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        print(f'ERROR: {error}', flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
