"""Isolated PX4/Gazebo spatial experiment: one aircraft or an opt-in fleet of four.

No arbitrary connection URL or flight-controller hardware interface is accepted. Flight commands
run inside an owned, network-isolated Docker container, not against host ports.
This tests spatial playback, not timed/offline swarm safety or onboard deployment.
"""

import argparse
import csv
from datetime import datetime
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import re
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET

import numpy as np

from .px4_reference import LocalOrigin, generate_reference, mission_items, polyline_distances
from .px4_scene import scene_tree
from .px4_termination import (end_behavior, with_end_behavior, flight_reference,
                              target_distance, remove_simulated_aircraft)
from .trajectory2d.obstacles import Obstacle, load_obstacles
from .px4_scenario import (default_scenario, validate_scenario, scenario_record,
                           load_scenario, loiter_record, derived_loiter, select_recovery,
                           loiter_observations, save_preview)


IMAGE = 'px4io/px4-sitl-gazebo@sha256:5ace9dbca657e385cc5c529432fd6d09db6165e482de9df5ab129a941a36aa54'
MODEL = 'gz_rc_cessna_lidar'
PX4_HASH = '61907962c9220437ec01a98fa26bfbf5cbb254b5'
ROOT = Path(__file__).resolve().parents[2]
GPU_RENDER_NODE = Path('/dev/dri/renderD128')
CAMERA_FOLLOW_TARGET = MODEL.removeprefix('gz_')+'_0'
CAMERA_FOLLOW_OFFSET = (-6., -3., 3.)


def save_json(path, value):
    # Artifacts only; never overwrite a prior run with a new experiment.
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def command(args, **kwargs):
    return subprocess.run(args, check=True, text=True, capture_output=True, timeout=30, **kwargs).stdout


def publish_airborne_release(output, mission_sha256):
    """One atomic own-model request; use a conservative post-publication cutoff."""
    pending=output/'airborne-release.pending.json'
    save_json(pending, {'mission_sha256':mission_sha256,'release':True})
    pending.rename(output/'airborne-release.json')
    return time.monotonic()


def source_hashes():
    files = [Path(__file__), ROOT/'src/swarm_sim/px4_reference.py', ROOT/'src/swarm_sim/px4_scene.py',
             ROOT/'src/swarm_sim/px4_scenario.py',
             ROOT/'src/swarm_sim/px4_fleet.py', ROOT/'examples/gazebo/start_fleet.sh',
             ROOT/'src/swarm_sim/px4_camera.py', ROOT/'src/swarm_sim/px4_termination.py',
             ROOT/'examples/gazebo/models/rc_cessna_lidar/model.sdf',
             ROOT/'examples/gazebo/start_sitl.sh',
             ROOT/'examples/gazebo/airborne_bootstrap.py',
             ROOT/'examples/gazebo/worlds/default.sdf',
             ROOT/'examples/gazebo/meshes/ground.dae',
             ROOT/'examples/gazebo/materials/textures/grass.ppm',
             *sorted((ROOT/'src/swarm_sim/trajectory2d').glob('*.py'))]
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}


def container_arguments(name, output, gui=False, gpu=False, airborne_start=False, fleet=False):
    """Only the project is mounted read-only; a fresh output directory is writable."""
    args = ['docker', 'run', '--detach', '--name', name, '--network', 'none',
            '--label', 'org.swarm-sim.px4-experiment=single-cessna',
            '--mount', f'type=bind,src={ROOT},dst=/workspace,readonly',
            '--mount', f'type=bind,src={output},dst=/runresults',
            '--env', 'PX4_SIM_MODEL='+MODEL,
            '--env', 'PX4_SYS_AUTOSTART=4003', '--env', 'SWARM_MANAGED_SIM=1',
            '--env', 'PX4_GZ_MODELS=/workspace/examples/gazebo/models',
            '--env', 'PX4_GZ_WORLDS=/workspace/examples/gazebo/worlds',
            '--env', 'PX4_GZ_NO_FOLLOW=1',
            '--env', 'GZ_PARTITION='+name]
    if airborne_start:
        args += ['--env', 'SWARM_AIRBORNE_START=1',
                 '--env', 'PX4_GZ_MODEL_NAME='+CAMERA_FOLLOW_TARGET,
                 '--env', 'PYTHONPATH=/workspace/examples/gazebo',
                 '--env', 'PYTHONDONTWRITEBYTECODE=1',
                 '--env', 'GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/examples/gazebo']
    if fleet:
        args += ['--env', 'SWARM_FLEET_SIZE=4']
    if gpu:
        if not GPU_RENDER_NODE.is_char_device() or not os.access(GPU_RENDER_NODE, os.R_OK | os.W_OK):
            raise ValueError(f'GPU mode requires a readable/writable render node: {GPU_RENDER_NODE}')
        # Explicit opt-in: only a render node, never a flight controller, card
        # control node, privileged container or the complete /dev directory.
        args += ['--device', f'{GPU_RENDER_NODE}:{GPU_RENDER_NODE}:rw',
                 '--env', 'LIBGL_ALWAYS_SOFTWARE=0']
    else:
        args += ['--env', 'LIBGL_ALWAYS_SOFTWARE=1']
    if gui:
        display, auth = os.environ.get('DISPLAY'), os.environ.get('XAUTHORITY')
        if not display or not auth or not Path(auth).is_file() or not Path('/tmp/.X11-unix').is_dir():
            raise ValueError('GUI requires a local X11/XWayland DISPLAY and a readable XAUTHORITY file; use headless otherwise')
        args += ['--env', 'DISPLAY='+display, '--env', 'XAUTHORITY=/tmp/swarm-xauth',
                 '--mount', 'type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly',
                 '--mount', f'type=bind,src={Path(auth).resolve()},dst=/tmp/swarm-xauth,readonly']
    else:
        args += ['--env', 'HEADLESS=1']
    script = 'start_fleet.sh' if fleet else 'start_sitl.sh'
    return args + ['--entrypoint', '/bin/sh', IMAGE, '/workspace/examples/gazebo/'+script]


def managed_run(output, gui, timeout_s, gpu=False):
    """Start/stop only our unique container; no system Gazebo/ROS installation."""
    importlib.metadata.version('mavsdk')  # Fail before starting a simulator if the optional SDK is absent.
    if importlib.metadata.version('mavsdk') != '4.1.0':
        raise ValueError('this adapter requires the verified MAVSDK Python 4.1.0 API')
    # Inspect locally first: downloading a pinned image is a separate explicit step.
    command(['docker', 'image', 'inspect', IMAGE])
    name = 'swarm-px4-'+uuid.uuid4().hex[:10]
    fleet = (output/'fleet.json').exists()
    reference = json.loads((output/('fleet.json' if fleet else 'reference.json')).read_text())
    args = container_arguments(name, output, gui, gpu, fleet or reference.get('airborne_start', False), fleet=fleet)
    started, success = False, False
    try:
        identifier = command(args).strip()
        started = True
        metadata = json.loads(command(['docker', 'inspect', name]))[0]
        if metadata['HostConfig']['NetworkMode'] != 'none':
            raise RuntimeError('refusing non-isolated simulator networking')
        save_json(output/'runtime.json', {'image': IMAGE, 'container_id': identifier,
            'model': MODEL, 'network': 'none', 'gui': gui, 'px4_expected_git_hash': PX4_HASH,
            'graphics_mode': 'dri' if gpu else 'software',
            'gpu_render_node': str(GPU_RENDER_NODE) if gpu else None,
            'mavsdk_python': importlib.metadata.version('mavsdk'), 'source_sha256': source_hashes(),
            'scene': json.loads((output/'scene.json').read_text()),
            'px4_release_status': 'development/prerelease, not stable v1.17'})
        worker = ['docker', 'exec',
            '--env', 'PYTHONPATH=/workspace/src', '--env', 'PYTHONDONTWRITEBYTECODE=1',
            '--env', 'MPLCONFIGDIR=/tmp/swarm-matplotlib', '--env', 'SWARM_SITL_WORKER=1',
            name, '/workspace/.venv/bin/python', '-m', 'swarm_sim.px4_fleet' if fleet else 'swarm_sim.px4_gazebo',
            '--worker', '/runresults', '--timeout', str(timeout_s)]
        process = subprocess.Popen(worker, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        with (output/'adapter.log').open('x', encoding='utf-8') as log:
            try:
                # communicate has a bounded wall timeout even if no MAVLink data arrives.
                stdout, _ = process.communicate(timeout=timeout_s+240)
                log.write(stdout)
                print(stdout, end='', flush=True)
            except BaseException:
                process.terminate()
                try:
                    stdout, _ = process.communicate(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    stdout, _ = process.communicate()
                log.write(stdout or '')
                raise
        success = process.returncode == 0
        if not (output/'report.json').exists():
            save_json(output/'report.json', {'schema': 'swarm-px4-gazebo-single-v1',
                'completed': False, 'backend_test_passed': False, 'spatial_tracking_passed': False,
                'timed_tracking_supported': False, 'swarm_separation_validated': False,
                'safety_certificate_transferred': False,
                'error': {'type': 'WorkerExit', 'returncode': process.returncode,
                          'message': 'worker exited without a report; see adapter.log'}})
        return process.returncode
    finally:
        if started:
            # Preserve diagnostics before stopping; no in-air disarm or forced landing.
            logs = subprocess.run(['docker', 'logs', name], capture_output=True, text=True, timeout=15)
            (output/'px4-gazebo.log').write_text(logs.stdout+logs.stderr, encoding='utf-8')
            # Installed PX4 defaults to HOME/.local/share/px4/rootfs. Retain ULogs,
            # including structured events not translated into STATUSTEXT.
            copied = subprocess.run(['docker', 'cp', name+':/root/.local/share/px4/rootfs/fs/log', str(output/'px4-logs')],
                                    capture_output=True, text=True, timeout=30)
            if copied.returncode == 0:
                subprocess.run(['docker', 'exec', name, 'chown', '-R', f'{os.getuid()}:{os.getgid()}',
                                '/runresults/px4-logs'], capture_output=True, timeout=15)
            subprocess.run(['docker', 'cp', name+':/root/.gz/auto_default.log', str(output/'gazebo.log')],
                           capture_output=True, timeout=15)
            subprocess.run(['docker', 'cp', name+':/root/.gz/rendering/ogre2.log', str(output/'ogre2.log')],
                           capture_output=True, timeout=15)
            subprocess.run(['docker', 'cp', name+':/opt/px4-gazebo/etc/extras/all_events.json.xz',
                            str(output/'px4-events.json.xz')], capture_output=True, timeout=15)
            # Native MAVSDK uses the image user's cache. Keep that user intact,
            # then restore ownership of this run's own generated regular files.
            owned = ['/runresults/'+p.name for p in output.iterdir() if p.is_file() and not p.is_symlink()]
            if owned:
                subprocess.run(['docker', 'exec', name, 'chown', f'{os.getuid()}:{os.getgid()}', *owned],
                               capture_output=True, timeout=15)
            if fleet:
                for index in range(4):
                    subprocess.run(['docker', 'exec', name, 'chown', '-R', f'{os.getuid()}:{os.getgid()}',
                                    f'/runresults/drone-{index}'], capture_output=True, timeout=15)
            stopped = subprocess.run(['docker', 'stop', '--timeout', '5', name], capture_output=True, timeout=20)
            removed = subprocess.run(['docker', 'rm', name], capture_output=True, timeout=15)
            save_json(output/'runtime_end.json', {'container_stopped': stopped.returncode == 0,
                'container_removed': removed.returncode == 0,
                'native_logs_copied': copied.returncode == 0,
                'worker_exited_successfully': success, 'runtime_ended_by': 'simulation shutdown; selected completion status is in report.json'})


class Measurements:
    """Observer-only data; actual flight control remains in PX4."""
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = {}
        self.received_at = {}
        self.rows = []
        self.status = []

    def callback(self, name):
        def receive(value, _):
            with self.lock:
                self.latest[name] = value
                self.received_at[name] = time.monotonic()
                if name == 'status_text':
                    self.status.append(str(value))
                if name == 'ground_truth':
                    if not all(math.isfinite(v) for v in (value.latitude_deg, value.longitude_deg, value.absolute_altitude_m)):
                        return
                    attitude, metrics, progress = (self.latest.get(k) for k in ('attitude_euler', 'fixedwing_metrics', 'mission_progress'))
                    self.rows.append({'sim_time_s': value.timestamp_us/1e6,
                        'latitude_deg': value.latitude_deg, 'longitude_deg': value.longitude_deg,
                        'absolute_altitude_m': value.absolute_altitude_m,
                        'mission_current': None if progress is None else progress.current,
                        'flight_mode': str(self.latest.get('flight_mode', 'unknown')),
                        'roll_deg': None if attitude is None else attitude.roll_deg,
                        'pitch_deg': None if attitude is None else attitude.pitch_deg,
                        'airspeed_m_s': None if metrics is None or not math.isfinite(metrics.airspeed_m_s) else metrics.airspeed_m_s})
        return receive


def airborne_estimate_check(measurements, desired, initial_time_s, yaw_rate, release_received_s, now_s=None):
    """Observer-only startup gate; preserve the original nominal-state bounds.

    MAVSDK PositionVelocityNed has no source timestamp. Receipt freshness is
    explicit but cannot prove source-time alignment or a stable full flight.
    """
    with measurements.lock:
        estimate = measurements.latest.get('position_velocity_ned')
        truth = measurements.latest.get('ground_truth')
        estimate_at = measurements.received_at.get('position_velocity_ned')
        truth_at = measurements.received_at.get('ground_truth')
        if estimate is None or truth is None or estimate_at is None or truth_at is None:
            return {'passed': False, 'reason': 'missing_samples'}
        height = -estimate.position.down_m
        velocity = np.array([estimate.velocity.east_m_s, estimate.velocity.north_m_s])
        sim_time = truth.timestamp_us/1e6
        now = time.monotonic() if now_s is None else now_s
    record = {'passed': False, 'height_m': height, 'velocity_east_m_s': float(velocity[0]),
        'velocity_north_m_s': float(velocity[1]), 'ground_truth_sim_time_s': sim_time,
        'estimate_receipt_s': estimate_at, 'ground_truth_receipt_s': truth_at,
        'estimate_source_timestamp_available': False,
        'height_tolerance_m': 3., 'velocity_component_tolerance_m_s': 3.,
        'maximum_receipt_age_s': 1.}
    if not np.all(np.isfinite([height,*velocity,sim_time,estimate_at,truth_at,now,release_received_s])):
        finite_record = {key: None if isinstance(value,float) and not math.isfinite(value) else value
                         for key,value in record.items()}
        return {**finite_record, 'reason': 'nonfinite_sample'}
    if min(estimate_at,truth_at) <= release_received_s or sim_time < initial_time_s:
        return {**record, 'reason': 'pre_release_sample'}
    if not (0 <= now-estimate_at <= 1. and 0 <= now-truth_at <= 1.):
        return {**record, 'reason': 'stale_sample'}
    elapsed = sim_time-initial_time_s
    yaw = desired[4]+yaw_rate*elapsed
    predicted = desired[3]*np.array([math.cos(yaw),math.sin(yaw)])
    height_error = abs(height-desired[2])
    velocity_error = np.abs(velocity-predicted)
    passed = bool(height_error < 3. and np.all(velocity_error < 3.))
    return {**record, 'passed': passed, 'reason': 'ready' if passed else 'nominal_state_mismatch',
        'elapsed_since_initial_state_s': elapsed, 'height_error_m': float(height_error),
        'velocity_component_errors_m_s': velocity_error.tolist(),
        'nominal_velocity_east_north_m_s': predicted.tolist()}


def wait_for(predicate, timeout, message, interval=.2):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(interval)
    raise TimeoutError(message)


def mission_is_ready(listener_output, expected_items):
    """The pinned PX4 upload ACK precedes asynchronous mission feasibility checks."""
    return (re.search(r'^\s+valid: True\s*$', listener_output, re.MULTILINE) is not None
            and re.search(r'^\s+seq_total: '+str(expected_items)+r'\s*$', listener_output, re.MULTILINE) is not None)


def reference_observation_rows(rows, item_count, release_time=None, first_sequence=3, trajectory_only=False):
    """Exclude preflight, never exclude post-release bootstrap transients."""
    return [row for row in rows if row['mission_current'] is not None
            and first_sequence <= row['mission_current'] < item_count-(0 if trajectory_only else 3)
            and (release_time is None or row['sim_time_s'] >= release_time)]


def camera_status_matches(output):
    """A service ACK alone does not establish an actual followed render target."""
    for line in output.splitlines():
        try:
            status = json.loads(line)
            if (status.get('trackMode') == 'FOLLOW'
                    and status.get('followTarget', {}).get('name') == CAMERA_FOLLOW_TARGET
                    and math.isclose(float(status.get('followPgain', 0)), 1., abs_tol=.001)
                    and all(math.isclose(float(status.get('followOffset', {}).get(axis, 0)), value, abs_tol=.01)
                            for axis, value in zip(('x', 'y', 'z'), CAMERA_FOLLOW_OFFSET))):
                return True
        except (ValueError, TypeError, AttributeError):
            continue
    return False


def camera_gz(args):
    """Bounded, GUI-only requests inside the managed Gazebo partition."""
    try:
        result = subprocess.run(['gz', *args], capture_output=True, text=True, timeout=2)
        return result.stdout if result.returncode == 0 else ''
    except (OSError, subprocess.TimeoutExpired):
        return ''


def initialize_camera_follow(timeout_s=30., before_arming=True):
    """One acknowledged startup attachment; no loop forcing user camera choices in flight."""
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError('positive camera initialization timeout required')
    deadline = time.monotonic()+timeout_s
    while time.monotonic() < deadline:
        follow = camera_gz(['service', '-s', '/gui/follow', '--reqtype', 'gz.msgs.StringMsg',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '1000',
            '--req', f'data: "{CAMERA_FOLLOW_TARGET}"'])
        if re.search(r'\bdata:\s*true\b', follow):
            offset = camera_gz(['service', '-s', '/gui/follow/offset', '--reqtype', 'gz.msgs.Vector3d',
                '--reptype', 'gz.msgs.Boolean', '--timeout', '1000',
                '--req', 'x: -6, y: -3, z: 3'])
            if re.search(r'\bdata:\s*true\b', offset):
                # The plugin's default gain (0.01/frame) lags far behind a
                # 15 m/s aircraft. Publish only after the acknowledged services
                # exist, then verify that this gain update actually arrived.
                camera_gz(['topic', '-t', '/gui/track', '-m', 'gz.msgs.CameraTrack', '-p',
                    f'track_mode: FOLLOW, follow_target: {{name: "{CAMERA_FOLLOW_TARGET}"}}, '
                    'follow_offset: {x: -6, y: -3, z: 3}, follow_pgain: 1.0, track_pgain: 1.0'])
                status = camera_gz(['topic', '-e', '-t', '/gui/currently_tracked', '-n', '1', '--json-output'])
                if camera_status_matches(status):
                    return {'target': CAMERA_FOLLOW_TARGET, 'mode': 'FOLLOW',
                            'offset_body_m': list(CAMERA_FOLLOW_OFFSET), 'pgain': 1., 'startup_status_confirmed': True}
        time.sleep(min(.2, max(0., deadline-time.monotonic())))
    context = 'before arming; no flight commands sent' if before_arming else 'after airborne Mission activation; experiment failed'
    raise TimeoutError('Gazebo GUI did not confirm camera follow '+context)


def native_command(module, *arguments):
    """Select a local native PX4 instance, never an external connection."""
    command = ['/opt/px4-gazebo/bin/px4-'+module]
    if os.environ.get('SWARM_FLEET_SIZE') == '4':
        index = int(os.environ['SWARM_FLEET_INDEX'])
        if index not in range(4):
            raise ValueError('Invalid fixed fleet instance.')
        command += ['--instance', str(index)]
    return command+list(arguments)


def execute_worker(output, timeout_s):
    """Private entry, reachable only in the managed, isolated SITL container."""
    if os.environ.get('SWARM_SITL_WORKER') != '1' or not Path('/.dockerenv').exists() or os.environ.get('PX4_SIM_MODEL') != MODEL:
        raise ValueError('worker is simulation-only; use the managed --run launcher')
    from mavsdk import Mavsdk, Configuration, ComponentType
    from mavsdk.plugins.telemetry import Telemetry
    from mavsdk.plugins.action import Action
    from mavsdk.plugins.param import Param
    from mavsdk.plugins.mission_raw import MissionRaw, MissionItem
    from mavsdk.plugins.info import Info
    measurements = Measurements()
    report = {'schema': 'swarm-px4-gazebo-single-v1', 'completed': False,
        'backend_test_passed': False, 'spatial_tracking_passed': False,
        'timed_tracking_supported': False, 'swarm_separation_validated': False,
        'safety_certificate_transferred': False,
        'reference_initialization': 'synthetic level flight after a separate PX4 takeoff/join',
        'scope': 'one aircraft; static scene geometry, no certified PX4 obstacle avoidance, live snapshot planning, swarm execution or real-hardware connections',
        'gazebo_obstacle_clearance_validated': False,
        'measurement_source': 'Gazebo ground truth forwarded by PX4 HIL_STATE_QUATERNION; attitude/airspeed from PX4 telemetry'}
    sdk = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
    origin, items, points = None, [], None
    world_offset = np.zeros(3)
    fleet = os.environ.get('SWARM_FLEET_SIZE') == '4'
    trajectory_only = False
    instance = int(os.environ['SWARM_FLEET_INDEX']) if fleet else 0
    if instance not in range(4) or (fleet and output != Path(f'/runresults/drone-{instance}')):
        raise ValueError('Fleet worker output/instance mismatch.')
    try:
        sdk.add_any_connection(f'udpin://127.0.0.1:{14540+instance}')
        system = sdk.first_autopilot(60.)
        if system is None:
            raise TimeoutError('no PX4 autopilot in the managed simulator')
        version_text = subprocess.run(native_command('ver', 'all'), check=True,
            capture_output=True, text=True, timeout=10).stdout
        if 'PX4_SITL' not in version_text or PX4_HASH not in version_text:
            raise ValueError('unverified PX4 SITL identity; no flight commands sent')
        report['px4_identity'] = version_text
        report['px4_version'] = str(Info(system).get_version())
        telemetry, mission, action, params = Telemetry(system), MissionRaw(system), Action(system), Param(system)
        for name in ('health', 'home', 'ground_truth', 'attitude_euler', 'fixedwing_metrics', 'status_text', 'in_air', 'armed', 'flight_mode', 'position_velocity_ned'):
            getattr(telemetry, 'subscribe_'+name)(measurements.callback(name))
        mission.subscribe_mission_progress(measurements.callback('mission_progress'))
        for method, rate in (('set_rate_ground_truth', 10.), ('set_rate_attitude_euler', 10.), ('set_rate_fixedwing_metrics', 5.)):
            getattr(telemetry, method)(rate)
        print('Waiting for PX4 readiness; no preflight checks are bypassed.', flush=True)
        def ready():
            health, home = measurements.latest.get('health'), measurements.latest.get('home')
            return (health is not None and home is not None and health.is_armable
                    and health.is_global_position_ok and health.is_home_position_ok
                    and len(measurements.rows) >= 5)
        wait_for(ready, 90., 'PX4 preflight/ground-truth readiness timeout')
        if params.get_param_int('SYS_AUTOSTART') != 4003:
            raise ValueError('unexpected airframe; no flight commands sent')
        if fleet and params.get_param_int('MAV_SYS_ID') != instance+1:
            raise ValueError('Unexpected MAVLink system identity; no arm command sent.')
        reference = json.loads((output/'reference.json').read_text())
        airborne = reference.get('airborne_start', False)
        if airborne != (os.environ.get('SWARM_AIRBORNE_START') == '1'):
            raise ValueError('airborne startup identity mismatch; no arm command sent')
        points = np.asarray(reference['positions_enu_m'])
        shared_scenario = reference.get('editor_scenario', False)
        report['shared_editor_scenario'] = shared_scenario
        loiter = reference.get('loiter')
        original = json.loads((output/'point_mass_mission.json').read_text())
        recorded_hash = original.pop('sha256')
        if (hashlib.sha256(json.dumps(original, sort_keys=True, allow_nan=False).encode()).hexdigest() != recorded_hash
                or reference['mission_sha256'] != recorded_hash or len(original['plans']) != 1
                or reference['obstacles'] != original['config']['obstacles'] or not np.array_equal(
                    points, np.asarray(original['plans'][0]['flight_states'])[:, :3])):
            raise ValueError('reference does not match its recorded single-aircraft mission')
        if shared_scenario and loiter != loiter_record(original):
            raise ValueError('loiter specification does not match the recorded mission')
        mode = end_behavior(original)
        if reference.get('end_behavior', 'land') != mode:
            raise ValueError('End behavior does not match the fingerprinted mission')
        trajectory_only = mode == 'remove-at-target'
        if trajectory_only and (not shared_scenario or not airborne):
            raise ValueError('Target removal requires a shared airborne editor scenario')
        points = flight_reference(original)
        report['end_behavior'] = mode
        if trajectory_only:
            report['landing_completed'] = False
        report['obstacle_count'] = len(original['config']['obstacles'])
        if fleet:
            from .px4_fleet import verify_worker_scene
            verify_worker_scene(output, original, instance)
            report['scope'] = 'one member of an experimental four-aircraft spatial mission; no timing/separation certificate'
            report['fleet_index'] = instance
        else:
            expected_scene = scene_tree(original, ROOT/'examples/gazebo/worlds/default.sdf', airborne_start=airborne)
            if ET.tostring(expected_scene.getroot()) != ET.tostring(ET.parse(output/'scene.sdf').getroot()):
                raise ValueError('Gazebo scene does not match the recorded mission; no arm command sent')
        report['scene'] = {'target_xy_m': original['config']['target'],
                           'obstacles': original['config']['obstacles'],
                           'target_annotations_collide': False,
                           'obstacles_have_collisions': True}
        home = measurements.latest['home']
        origin = LocalOrigin(home.latitude_deg, home.longitude_deg, home.absolute_altitude_m)
        if airborne:
            # Native home is the ground spawn below the reference's first XY
            # position. Keep world geometry unchanged; translate only the
            # derived geographic mission / observer's home-tangent coordinates.
            world_offset[:2] = points[0, :2]
            telemetry.set_rate_position_velocity_ned(50.)
            wait_for(lambda: (output/'airborne-bootstrap-ready.json').exists(), 10.,
                     'airborne physics plugin did not become ready; no arm command sent')
            report['reference_initialization'] = 'normal ground preflight/arm, then one-shot airborne physical impulse before reference execution; not a cold in-air PX4 boot'
        airspeed = float(original['snapshots'][0]['speed'])
        recovery = (select_recovery(points-world_offset, tuple(Obstacle(**o) for o in original['config']['obstacles']), world_offset)
                    if shared_scenario and not trajectory_only else None)
        items, indices = mission_items(points-world_offset, origin, airspeed=airspeed,
            loiter=derived_loiter(loiter, world_offset, original['config']['dt']),
            allow_vertical=shared_scenario, recovery=recovery, end_behavior=mode)
        report['loiter'] = loiter
        report['recovery'] = recovery
        report['world_offset_from_home_enu_m'] = world_offset.tolist()
        report['origin'] = vars(origin)
        report['px4_airspeed_parameters_m_s'] = {name: params.get_param_float(name)
            for name in ('FW_AIRSPD_MIN', 'FW_AIRSPD_TRIM', 'FW_AIRSPD_MAX')}
        report['px4_landing_parameters'] = {name: params.get_param_int(name)
            for name in ('FW_LND_USETER', 'FW_LND_ABORT')}
        report['simulation_fixture'] = 'official rc_cessna with downward lidar; 6 km flat ground tile; original PX4 checks enabled'
        if not report['px4_airspeed_parameters_m_s']['FW_AIRSPD_MIN'] <= airspeed <= report['px4_airspeed_parameters_m_s']['FW_AIRSPD_MAX']:
            raise ValueError('reference airspeed outside actual PX4 limits')
        save_json(output/'px4_mission.json', {'items': items, 'reference_sample_indices': indices,
            'acceptance_radius_m': 15., 'timing_preserved': False, 'end_behavior': mode})
        # None is the JSON representation of an unspecified MAVLink yaw, not NaN in artifacts.
        raw = [MissionItem(**{k: float('nan') if v is None else v for k, v in item.items()}) for item in items]
        if trajectory_only:
            # This SIMULATION mode explicitly has no landing item. Require a
            # takeoff item instead; retain normal mission-valid/readiness/arm
            # gates and every other PX4 parameter/failsafe. Never used on hardware.
            before = params.get_param_int('MIS_TKO_LAND_REQ')
            params.set_param_int('MIS_TKO_LAND_REQ', 1)
            if params.get_param_int('MIS_TKO_LAND_REQ') != 1:
                raise ValueError('Trajectory-only mission requirement was not applied')
            report['simulation_mission_requirement'] = {'parameter': 'MIS_TKO_LAND_REQ',
                'before': before, 'after': 1, 'meaning': 'require takeoff; no landing requirement',
                'mission_feasibility_check_retained': True, 'simulation_only': True}
        mission.upload_mission(raw)
        def mission_ready():
            state = subprocess.run(native_command('listener', 'mission_result', '-n', '1'),
                                   capture_output=True, text=True, timeout=5)
            return state.returncode == 0 and mission_is_ready(state.stdout, len(items))
        wait_for(mission_ready, 30., 'PX4 mission feasibility did not become valid; no arm command sent')
        if os.environ.get('HEADLESS') != '1' and not airborne:
            report['camera_follow'] = initialize_camera_follow()
            print(f'Gazebo camera follow confirmed: {CAMERA_FOLLOW_TARGET}.', flush=True)
        if fleet:
            from .px4_fleet import barrier
            barrier(output, 'ready-to-arm')
        action.arm()
        if airborne:
            # Keep the original valid takeoff/landing mission for feasibility,
            # but do not execute a runway takeoff or chase the point now behind
            # the plane. The actual airborne state must precede Mission mode.
            entry_sequence = 1 if loiter is not None else 3
            mission.set_current_mission_item(entry_sequence)
            report['airborne_first_mission_sequence'] = entry_sequence
            if fleet:
                barrier(output, 'armed-ready')
                from .px4_fleet import wait_bootstrap_turn
                report['fleet_bootstrap_order'] = wait_bootstrap_turn(
                    output, instance, original['parent_mission_sha256'])
                wait_for(lambda: measurements.latest.get('armed') is True, 3.,
                         'PX4 did not confirm armed state before its ordered warm initialization')
            release_received_s = publish_airborne_release(output, recorded_hash)
            wait_for(lambda: (output/'airborne-initial-state.json').exists(), 5.,
                     'airborne state was not observed after the release request')
            initialization = json.loads((output/'airborne-initial-state.json').read_text())
            snapshot = initialization['snapshot']
            desired = np.asarray(original['plans'][0]['flight_states'][0])
            desired_velocity = desired[3]*np.array([math.cos(desired[4]), math.sin(desired[4]), 0.])
            initialization['position_error_m'] = float(np.linalg.norm(np.array(snapshot['position_world_m'])-desired[:3]))
            initialization['velocity_error_m_s'] = float(np.linalg.norm(np.array(snapshot['velocity_world_m_s'])-desired_velocity))
            attitude_error = np.array(snapshot['rpy_rad'])-np.array([-desired[6], 0., desired[4]])
            attitude_error = (attitude_error+math.pi) % (2*math.pi)-math.pi
            initialization['attitude_error_max_deg'] = float(np.rad2deg(np.abs(attitude_error)).max())
            expected_angular = np.array([0., 0., original['snapshots'][0]['state'][3]])
            initialization['angular_velocity_error_rad_s'] = float(np.linalg.norm(
                np.asarray(snapshot['angular_velocity_rad_s'])-expected_angular))
            initialization['initial_state_passed'] = (initialization['position_error_m'] <= .5
                and initialization['velocity_error_m_s'] <= .5
                and initialization['attitude_error_max_deg'] <= 2.
                and initialization['angular_velocity_error_rad_s'] <= .1)
            report['airborne_initialization'] = initialization
            save_json(output/'airborne-check.json', initialization)
            if not initialization['initial_state_passed']:
                raise ValueError('physical airborne state does not match the requested start within 0.5 m / 0.5 m/s')
            print('One-shot airborne initialization confirmed; no continued simulator velocity/pose control.', flush=True)
            wait_for(lambda: measurements.latest.get('in_air') is True, 3.,
                     'PX4 did not detect airborne state before Mission mode')
            checks = []
            def estimated_airborne_state_ready():
                check = airborne_estimate_check(measurements, desired, snapshot['sim_time_s'],
                    original['snapshots'][0]['state'][3], release_received_s)
                checks.append(check)
                return check['passed']
            try:
                wait_for(estimated_airborne_state_ready, 3.,
                         'PX4 estimated airborne position/velocity did not converge before Mission mode', interval=.02)
            finally:
                report['airborne_estimator_acquisition'] = {'passed': bool(checks and checks[-1]['passed']),
                    'timeout_wall_s': 3., 'release_receipt_s': release_received_s,
                    'checks': checks, 'nominal_state_gate_retained': True,
                    'receipt_freshness_is_not_source_time_alignment': True}
                save_json(output/'airborne-estimator-check.json', report['airborne_estimator_acquisition'])
            report['airborne_estimated_state_before_mission'] = checks[-1]
            # This pinned prerelease controller keeps the ground-level TECS
            # altitude reference through an AUTO-mode estimator height reset.
            # Reinitialize that controller ONCE from the now-observed airborne
            # state. Simulator-only bootstrap, not a real-aircraft procedure or
            # a parameter/failsafe change. EKF/attitude/rate modules stay alive.
            for operation in ('stop', 'start'):
                subprocess.run(native_command('fw_lat_lon_control', operation),
                               check=True, capture_output=True, text=True, timeout=3)
            report['airborne_controller_initialization'] = 'one native fw_lat_lon_control stop/start after estimated-state acquisition; all original parameters retained'
            report['airborne_controller_reset_count'] = 1
            mission.start_mission()
            report['mission_started'] = True
            if fleet:
                from .px4_fleet import complete_bootstrap_turn
                complete_bootstrap_turn(output, instance, original['parent_mission_sha256'], report)
            if os.environ.get('HEADLESS') != '1':
                subprocess.Popen(['gz', 'sim', '-g'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                report['camera_follow'] = initialize_camera_follow(before_arming=False)
                print(f'Gazebo airborne camera follow confirmed: {CAMERA_FOLLOW_TARGET}.', flush=True)
        else:
            mission.start_mission()
            report['mission_started'] = True
        reference_end_seq = (3 if loiter is not None else 2)+len(indices)
        report['reference_end_sequence'] = reference_end_seq
        entry = 'airborne entry' if airborne else 'takeoff'
        ending = 'remove own model on observed target contact' if trajectory_only else 'return/landing pattern'
        print(f'PX4 mission started: {entry}, {len(indices)} generated reference waypoints, {ending}.', flush=True)
        deadline = time.monotonic()+timeout_s
        last_print = 0.
        seen_mission_mode = False
        observed_rows = 0
        while time.monotonic() < deadline:
            if trajectory_only:
                with measurements.lock:
                    pending = measurements.rows[observed_rows:]
                    observed_rows = len(measurements.rows)
                for row in pending:
                    if (row['mission_current'] is None or row['mission_current'] < (3 if loiter else 2)
                            or row['sim_time_s'] < report['airborne_initialization']['snapshot']['sim_time_s']):
                        continue
                    position = origin.to_enu(row['latitude_deg'], row['longitude_deg'], row['absolute_altitude_m'])+world_offset
                    distance = target_distance(position, original['config'])
                    if distance <= original['config']['target_radius']:
                        report['target_contact'] = {'sim_time_s': row['sim_time_s'],
                            'position_world_m': position.tolist(), 'distance_m': distance,
                            'radius_m': original['config']['target_radius'], 'source': 'own ground-truth sample'}
                        save_json(output/'target-contact.json', report['target_contact'])
                        report['removal'] = remove_simulated_aircraft(instance)
                        save_json(output/'target-removal.json', report['removal'])
                        break
                if report.get('removal', {}).get('absence_verified'):
                    break
            progress = measurements.latest.get('mission_progress')
            truth = measurements.latest.get('ground_truth')
            flight_mode = measurements.latest.get('flight_mode')
            if flight_mode is not None:
                seen_mission_mode |= flight_mode.name == 'MISSION'
                if seen_mission_mode and flight_mode.name == 'HOLD' and measurements.latest.get('in_air') is True:
                    raise RuntimeError('PX4 left Mission for Hold while airborne; target/reference not completed. '
                                       'Inspect px4-gazebo.log and px4-logs for the abort/failsafe cause.')
            if progress is not None and truth is not None:
                if time.monotonic()-last_print >= 10:
                    print(f'item {progress.current}/{progress.total}; simulation t={truth.timestamp_us/1e6:.1f}s', flush=True)
                    last_print = time.monotonic()
                if progress.current >= reference_end_seq:
                    report['reference_waypoints_completed'] = True
                if (not trajectory_only and report.get('reference_waypoints_completed')
                        and measurements.latest.get('in_air') is False
                        and measurements.latest.get('armed') is False):
                    report['landing_completed'] = True
                    if fleet:
                        # Keep ground-truth observation alive after own landing
                        # until the last aircraft lands; do not truncate the
                        # fleet observer at the first completed flight.
                        barrier(output, 'landed', timeout_s)
                    break
            time.sleep(.2)
        else:
            raise TimeoutError('PX4 did not reach the target and remove its model within the wall-time budget'
                               if trajectory_only else 'PX4 did not complete reference and landing within the wall-time test budget')
        report['completed'] = True
    except BaseException as error:
        report['error'] = {'type': type(error).__name__, 'message': str(error)}
        report['last_estimated_position_velocity_ned'] = str(measurements.latest.get('position_velocity_ned'))
        print(f'FAILED: {type(error).__name__}: {error}', flush=True)
    finally:
        sdk.destroy()
        rows = measurements.rows
        report['status_messages'] = measurements.status
        if origin is not None and points is not None and rows:
            for row in rows:
                xyz = origin.to_enu(row['latitude_deg'], row['longitude_deg'], row['absolute_altitude_m'])+world_offset
                row.update(zip(('east_m', 'north_m', 'up_m'), map(float, xyz)))
            if report.get('obstacle_count'):
                all_xyz = np.array([[r[k] for k in ('east_m', 'north_m', 'up_m')] for r in rows])
                clearance = min(float(np.min(Obstacle(**o).distance(all_xyz)))
                                for o in original['config']['obstacles'])
                report['minimum_sampled_obstacle_center_distance_m'] = clearance
                report['sampled_obstacle_center_penetration'] = clearance < 0
            # First waypoint completion is an acquisition marker, not a synchronized epoch.
            release_time = report.get('airborne_initialization', {}).get('snapshot', {}).get('sim_time_s')
            report['reference_observation_release_sim_time_s'] = release_time
            if report.get('loiter') is not None:
                report['loiter_observations'] = loiter_observations(rows, report['loiter'], release_time)
            selected = reference_observation_rows(rows, len(items), release_time, first_sequence=2 if report.get('loiter') else 3,
                                                  trajectory_only=trajectory_only)
            report['ground_truth_samples'] = len(rows)
            report['reference_following_samples'] = len(selected)
            if selected:
                xyz = np.array([[r[k] for k in ('east_m', 'north_m', 'up_m')] for r in selected])
                distances = polyline_distances(xyz, points)
                report.update(spatial_error_p95_m=float(np.percentile(distances, 95)),
                    spatial_error_max_m=float(distances.max()),
                    altitude_error_max_m=(float(np.abs(xyz[:, 2]-points[0, 2]).max())
                                          if np.ptp(points[:, 2]) < .1 else None),
                    spatial_quality_threshold_p95_m=25.,
                    spatial_tracking_passed=bool(len(selected) >= 50 and np.percentile(distances, 95) <= 25.))
                if report.get('shared_editor_scenario'):
                    config = original['config']
                    if config['target_altitude'] is None:
                        target_distances = np.linalg.norm(xyz[:, :2]-config['target'], axis=1)
                    else:
                        target_distances = np.linalg.norm(xyz-np.r_[config['target'], config['target_altitude']], axis=1)
                    hits = np.flatnonzero(target_distances <= config['target_radius'])
                    report['minimum_sampled_target_distance_m'] = float(target_distances.min())
                    report['target_reached_in_px4_samples'] = bool(len(hits))
                    report['first_sampled_arrival_sim_time_s'] = selected[int(hits[0])]['sim_time_s'] if len(hits) else None
                plot_reference(output, points, rows, original)
        if rows:
            with (output/'telemetry.csv').open('x', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        terminal_passed = (bool(report.get('target_contact') and report.get('removal', {}).get('absence_verified'))
                           if trajectory_only else bool(report.get('landing_completed') and report.get('reference_waypoints_completed')))
        report['backend_test_passed'] = bool(report['completed'] and terminal_passed
                                            and report.get('reference_following_samples', 0) >= 50
                                            and not report.get('sampled_obstacle_center_penetration', False))
        if report.get('loiter') is not None:
            report['backend_test_passed'] &= report.get('loiter_observations', {}).get('verified_in_px4_experiment', False)
        if report.get('shared_editor_scenario'):
            report['backend_test_passed'] &= report.get('target_reached_in_px4_samples', False)
        save_json(output/'report.json', report)
        print(json.dumps({k: report.get(k) for k in ('completed', 'backend_test_passed', 'spatial_tracking_passed',
            'spatial_error_p95_m', 'spatial_error_max_m', 'timed_tracking_supported', 'error')}, allow_nan=False), flush=True)
    return 0 if report['backend_test_passed'] and report['spatial_tracking_passed'] else 3


def plot_reference(output, points, rows, document=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    xyz = np.array([[r[k] for k in ('east_m', 'north_m', 'up_m')] for r in rows])
    figure = plt.figure(figsize=(11, 7), layout='constrained')
    axis = figure.add_subplot(111, projection='3d')
    axis.plot(*points.T, '--', label='Generated point-mass reference')
    axis.plot(*xyz.T, label='PX4 / Gazebo ground truth (includes takeoff/join)')
    if document is not None:
        from types import SimpleNamespace
        from .trajectory2d.obstacle_view import draw_obstacles_3d
        draw_obstacles_3d(axis, SimpleNamespace(obstacles=tuple(Obstacle(**o) for o in document['config']['obstacles'])))
        target = document['config']['target']
        target_z = document['config']['target_altitude']
        axis.scatter(*target, points[0, 2] if target_z is None else target_z,
                     color='gold', marker='X', s=80, label='Destination')
    axis.set(xlabel='East (m)', ylabel='North (m)', zlabel='Up from home (m)',
             title='Single fixed-wing spatial playback — timing/separation NOT validated')
    axis.legend()
    figure.savefig(output/'comparison.png', dpi=140)
    plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true', help='explicitly start isolated Docker SITL and fly the simulated mission')
    parser.add_argument('--num-drones', type=int, choices=[1, 4], default=1,
                        help='one aircraft or the experimental four-aircraft shared-world backend')
    parser.add_argument('--gui', action='store_true', help='show Gazebo through local X11/XWayland; requires --run')
    parser.add_argument('--gpu', action='store_true', help='explicitly expose only /dev/dri/renderD128 for Intel/AMD Mesa rendering; requires --run')
    parser.add_argument('--camera', choices=['free', 'follow'], help='four-aircraft GUI camera: free overview (default) or drone-0 follow')
    parser.add_argument('--seed', type=int, help='PRNG seed; default 7, or the saved scenario seed')
    parser.add_argument('--generator-mode', choices=['geometric', 'physical_mission'],
                        help='fresh editor: physical_mission; explicit saved selections and historical v1 geometry are preserved')
    parser.add_argument('--numerical-backend', choices=['python', 'cpp'],
                        help='fresh physical editor: cpp; saved selections are preserved; python selects the reference planner; PX4 control is unchanged')
    parser.add_argument('--workers', type=int, help='catalog processes; 0 selects available CPUs')
    parser.add_argument('--obstacles', type=Path, help='static box/cylinder JSON shared with Matplotlib; included in planning and Gazebo collisions, not certified PX4 avoidance')
    parser.add_argument('--airborne-start', action='store_true', help='experimental warm start: normal ground readiness/arming, one-shot physical initialization at the reference start; no cold in-air boot')
    parser.add_argument('--edit-scenario', action='store_true', help='edit aircraft, target and obstacles, validate the reference, then launch Gazebo with --run')
    parser.add_argument('--end-behavior', choices=['remove-at-target', 'land'],
                        help='editor/scenario default: remove-at-target; historical benchmark default: land')
    parser.add_argument('--scenario', type=Path, help='load single-aircraft or fleet-v1/v2/v3 JSON with --num-drones 4; implies airborne start')
    parser.add_argument('--timeout', type=float, default=600., help='wall seconds for startup, reference and selected completion, not the planning horizon')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--worker', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.num_drones == 4 and not args.worker:
        from .px4_fleet import main as fleet_main
        return fleet_main(argv)
    if args.camera is not None:
        parser.error('--camera currently applies to --num-drones 4')
    if (args.seed is not None and args.seed < 0) or not math.isfinite(args.timeout) or not 30 <= args.timeout <= 1800:
        parser.error('nonnegative seed and timeout within 30..1800 seconds required')
    if args.workers is not None and args.workers < 0:
        parser.error('workers must be nonnegative')
    if args.worker:
        return execute_worker(args.worker, args.timeout)
    if args.gui and not args.run:
        parser.error('gui requires run')
    if args.gpu and not args.run:
        parser.error('gpu requires run')
    if args.scenario and args.obstacles:
        parser.error('scenario already contains obstacles; edit them there or use --obstacles without --scenario')
    output = args.output or ROOT/'results/px4-gazebo'/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    if output.exists():
        parser.error('output must be a new directory')
    try:
        seed = 7 if args.seed is None else args.seed
        obstacles = load_obstacles(args.obstacles) if args.obstacles else ()
        scenario = load_scenario(args.scenario) if args.scenario else (
            default_scenario(seed, obstacles) if args.edit_scenario else None)
        termination = args.end_behavior or ('remove-at-target' if scenario is not None else 'land')
        if termination == 'remove-at-target' and scenario is None:
            raise ValueError('remove-at-target requires --scenario or --edit-scenario')
        if args.scenario and args.seed is not None:
            from dataclasses import replace
            scenario = (scenario[0], replace(scenario[1], seed=seed))
        if (args.generator_mode is not None or args.numerical_backend is not None or args.workers is not None) and scenario is None:
            raise ValueError('generator-mode/numerical-backend/workers require --scenario or --edit-scenario; historical bench is unchanged')
        if scenario is not None:
            from dataclasses import replace
            overrides = {}
            if args.generator_mode is not None:
                overrides['generator_mode'] = args.generator_mode
            if args.numerical_backend is not None:
                overrides['numerical_backend'] = args.numerical_backend
            if args.workers is not None:
                overrides['planning_workers'] = args.workers
            scenario = scenario[0], replace(scenario[1], **overrides)
        prepared = []
        def prepare_edited(snapshots, config):
            validate_scenario(snapshots, config)
            bundle = generate_reference(scenario=(snapshots, config))
            doc, _, xyz = bundle
            doc = with_end_behavior(doc, termination)
            bundle = doc, bundle[1], bundle[2]
            xyz = flight_reference(doc)
            if np.max(np.abs(xyz[:, :2])) > 2800:
                raise ValueError('Calculated reference leaves the laboratory ground footprint (±2800 m).')
            offset = np.r_[xyz[0, :2], 0.]
            recovery = select_recovery(xyz-offset, config.obstacles, offset) if termination == 'land' else None
            mission_items(xyz-offset, LocalOrigin(47.397971057728974, 8.546163739800146, 0.),
                airspeed=snapshots[0].speed, allow_vertical=True,
                loiter=derived_loiter(loiter_record(doc), offset, config.dt), recovery=recovery,
                end_behavior=termination)
            prepared[:] = [bundle]
        if args.edit_scenario:
            from .trajectory2d.scenario_editor import edit_scenario
            scenario = edit_scenario(scenario[0], scenario[1].target, scenario[1],
                                     launch_validator=prepare_edited, profile_loading=False)
            if scenario is None:
                print('Preparation cancelled; no simulator launched.', flush=True)
                return 0
        if scenario is not None:
            args.airborne_start = True
            if not prepared:
                prepare_edited(*scenario)
            document, times, positions = prepared[0]
        else:
            document, times, positions = generate_reference(seed, obstacles)
        loiter = loiter_record(document) if scenario is not None else None
        # Verify the adapter scope before starting Docker, including level flight.
        mission_items(flight_reference(document), LocalOrigin(47.397971057728974, 8.546163739800146, 0.),
            airspeed=document['snapshots'][0]['speed'], allow_vertical=scenario is not None,
            loiter=derived_loiter(loiter, np.zeros(3), document['config']['dt']), end_behavior=termination)
        output.mkdir(parents=True, exist_ok=False)
        save_json(output/'point_mass_mission.json', document)
        save_json(output/'reference.json', {'schema': 'swarm-px4-spatial-reference-v1',
            'positions_enu_m': positions.tolist(), 'nominal_times_s': times.tolist(),
            'seed': document['config']['seed'], 'mission_sha256': document['sha256'],
            'airborne_start': args.airborne_start,
            'editor_scenario': scenario is not None, 'loiter': loiter,
            'end_behavior': termination,
            'airspeed_m_s': document['snapshots'][0]['speed'], 'timing_preserved_by_px4_mission': False,
            'single_aircraft_only': True, 'obstacles': document['config']['obstacles'], 'source_sha256': source_hashes()})
        scene_tree(document, ROOT/'examples/gazebo/worlds/default.sdf', airborne_start=args.airborne_start).write(
            output/'scene.sdf', encoding='utf-8', xml_declaration=True)
        save_json(output/'scene.json', {'mission_sha256': document['sha256'],
            'scene_sha256': hashlib.sha256((output/'scene.sdf').read_bytes()).hexdigest(),
            'frame': 'Gazebo ENU metres; geographic home below the initial aircraft XY',
            'obstacle_count': len(document['config']['obstacles']), 'target_annotations_collide': False,
            'end_behavior': termination,
            'reference_display': {'arrival_color': 'cyan', 'onward_color': 'muted blue',
                                  'onward_visible': termination == 'land',
                                  'sample_simplification_tolerance_m': .5,
                                  'diameter_m': 1.2, 'collisions': False,
                                  'changes_flight_mission': False},
            'gazebo_obstacle_clearance_validated': False})
        if scenario is not None:
            save_json(output/'scenario.json', scenario_record(*scenario))
            save_preview(output/'planning-preview.png', document)
        if document['config']['obstacles']:
            print('WARNING: obstacles have physical collisions. Only the point-mass reference was checked; '
                  'PX4 waypoint rounding, takeoff/join and landing clearance are NOT validated.', flush=True)
        print(f'Reference generated; results: {output}', flush=True)
        if not args.run:
            print('Offline only: no simulator started and no flight commands sent. Add --run for managed SITL.', flush=True)
            return 0
        result = managed_run(output.resolve(), args.gui, args.timeout, args.gpu)
        print(f'Artifacts: {output}', flush=True)
        return result
    except (ValueError, OSError, RuntimeError, subprocess.SubprocessError, importlib.metadata.PackageNotFoundError) as error:
        print(f'ERROR: {error}', flush=True)
        if output.exists() and not (output/'launch_error.json').exists():
            save_json(output/'launch_error.json', {'type': type(error).__name__, 'message': str(error)})
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
