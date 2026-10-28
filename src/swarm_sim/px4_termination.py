"""Simulation-only target removal; never an aircraft landing/fault procedure."""

import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time

import numpy as np


def end_behavior(document):
    mode = document.get('simulation_end_behavior', 'land')
    if mode not in ('land', 'remove-at-target'):
        raise ValueError('Unknown simulation end behavior')
    return mode


def with_end_behavior(document, mode):
    result = {k: v for k, v in document.items() if k != 'sha256'}
    result['simulation_end_behavior'] = mode
    end_behavior(result)
    result['sha256'] = hashlib.sha256(json.dumps(result, sort_keys=True, allow_nan=False).encode()).hexdigest()
    return result


def flight_reference(document):
    """Derived PX4 prefix only; the original full-horizon plan stays immutable.

    Five seconds after nominal contact allow spatial fly-through acquisition.
    They are not an arrival tolerance, a landing leg or permission to miss.
    """
    plan = document['plans'][0]
    points = np.asarray(plan['flight_states'], dtype=float)[:, :3]
    if end_behavior(document) == 'remove-at-target':
        stop = min(len(points), math.ceil((plan['arrival_time']+5.)/plan['dt'])+1)
        points = points[:stop]
    return points.copy()


def target_distance(position, config):
    position = np.asarray(position, dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError('Finite own XYZ observation required')
    goal = config['target']
    if config['target_altitude'] is None:
        return float(np.linalg.norm(position[:2]-goal))
    return float(np.linalg.norm(position-np.r_[goal, config['target_altitude']]))


def remove_simulated_aircraft(index):
    """Remove exactly the caller's isolated Gazebo model, then verify absence.

    No disarm, termination, pose, actuator or real-hardware operation is sent.
    The other simulated aircraft and all scene geometry remain untouched.
    """
    fleet = os.environ.get('SWARM_FLEET_SIZE') == '4'
    expected = int(os.environ.get('SWARM_FLEET_INDEX', '0')) if fleet else 0
    if (os.environ.get('SWARM_MANAGED_SIM') != '1'
            or os.environ.get('SWARM_SITL_WORKER') != '1'
            or not Path('/.dockerenv').exists() or type(index) is not int
            or index not in range(4) or index != expected):
        raise ValueError('Model removal is restricted to the own managed SITL instance')
    name = f'rc_cessna_lidar_{index}'

    def service(topic, request_type, reply_type, request):
        result = subprocess.run(['gz', 'service', '-s', topic, '--reqtype', request_type,
            '--reptype', reply_type, '--timeout', '3000', '--req', request],
            check=True, capture_output=True, text=True, timeout=5)
        return result.stdout

    def present():
        scene = service('/world/default/scene/info', 'gz.msgs.Empty', 'gz.msgs.Scene', '')
        # Harmonic's Scene message has no root world name. Require the pinned
        # fixture's ground model in a valid top-level model list, never treat
        # an empty reply/transport error as proof that the aircraft vanished.
        models = re.findall(r'^model\s*\{\s*name:\s*"([^"]+)"', scene, re.MULTILINE)
        if 'ground_plane' not in models:
            raise RuntimeError('Cannot verify the isolated Gazebo scene')
        return name in models

    if not present():
        raise RuntimeError('Own aircraft was absent before removal; not a successful arrival')
    reply = service('/world/default/remove', 'gz.msgs.Entity', 'gz.msgs.Boolean',
                    f'name: "{name}", type: MODEL')
    if re.search(r'\bdata:\s*true\b', reply) is None:
        raise RuntimeError('Gazebo rejected own-aircraft removal')
    deadline = time.monotonic()+5
    while present():
        if time.monotonic() >= deadline:
            raise TimeoutError('Gazebo acknowledged removal but own model is still present')
        time.sleep(.05)
    return {'model': name, 'acknowledged': True, 'absence_verified': True,
            'simulation_only': True, 'landing_performed': False}
