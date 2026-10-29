"""Explicit optional C++ construction; Python dynamics/validators stay independent.

The binding copies validated float64 input buffers before releasing the GIL.
Returned arrays are views of immutable bytes, never views of reusable scratch.
"""
from functools import lru_cache
import hashlib
from importlib import import_module
from pathlib import Path
import numpy as np


@lru_cache(maxsize=1)
def require_native():
    try:
        module = import_module('swarm_sim._trajectory_native')
    except ImportError as error:
        raise RuntimeError('C++ trajectory backend is unavailable; run ./scripts/build_native.sh or install a wheel built with SWARM_BUILD_NATIVE=1') from error
    if module.build_info().get('abi') != 1:
        raise RuntimeError('Unsupported C++ trajectory ABI; rebuild the extension')
    return module


def identity():
    module = require_native()
    return {**module.build_info(), 'binary_sha256': hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()}


def parameters(config):
    if config.aircraft_profiles:
        raise ValueError('native kernel requires resolved own aircraft physics')
    f, lim = config.flight, config.limits
    return tuple(map(float, (f.stall_speed, f.stall_margin, f.max_load_factor, f.min_load_factor,
        np.deg2rad(f.max_flight_path_deg), np.deg2rad(f.max_flight_path_rate_deg_s),
        f.max_thrust_acceleration, f.max_brake_acceleration, f.parasitic_drag, f.induced_drag,
        lim.min_speed, lim.max_speed, np.deg2rad(lim.max_bank_deg), np.deg2rad(lim.max_roll_rate_deg_s),
        config.min_altitude, config.max_altitude, config.max_climb_rate, config.descent_rate_limit,
        config.max_vertical_acceleration, config.dt)))


def _array(value):
    # Explicit wrapper conversion; low-level binding itself never force-casts.
    return np.ascontiguousarray(value, dtype=np.float64)


def evaluate(operation, state, controls, config, **options):
    return require_native().evaluate(operation, _array(state), _array(controls), parameters(config), options)


def _path(result):
    states, controls, phases, reason, arrival = result
    if reason:
        return None, reason
    return (np.frombuffer(states, dtype=np.float64).reshape(-1, 7),
            np.frombuffer(controls, dtype=np.float64).reshape(-1, 4),
            tuple(phases), arrival), None


def propagate(state, control, duration, config, substeps=2, wind=(0., 0., 0.)):
    result = evaluate('propagate', state, control, config, duration=float(duration),
                      substeps=substeps, wind=_array(wind))
    return np.frombuffer(result, dtype=np.float64)


def checked_step(state, control, config):
    result = evaluate('checked', state, control, config)
    return None if result is None else np.frombuffer(result, dtype=np.float64)


def rollout(state, controls, config, *, check=True, substeps=2, wind=(0., 0., 0.)):
    """Whole held-command history in one native call; rejected prefixes are not plans."""
    return _path(evaluate('rollout', state, controls, config, check=int(check),
                         substeps=substeps, wind=_array(wind)))


def fixture(speed, config, cruise=None):
    target_z = (cruise if config.target_altitude is None else config.target_altitude)
    state = [0., 0., config.max_altitude if cruise is None else cruise, speed, 0., 0., 0.]
    return evaluate('headroom' if cruise is None else 'terminal', state, (), config,
                    speed=float(speed), cruise=float(cruise or 0),
                    steps=round(config.horizon/config.dt), angle=float(-np.deg2rad(config.descent_angle_deg)),
                    target_z=float(target_z or 0), target_radius=float(config.target_radius),
                    position_error=float(config.position_error))


def construct(snapshot, config, *, cruise, release, levels, minimum_hold, speed_reference,
              speed_derivative, speed_end, heading, gate, direction, normal, waypoints,
              candidate_id, diagnostics):
    from .catalog import reject
    from .flight3d import from_snapshot
    from .obstacles import avoidance_waypoints
    recipe = dict(steps=round(config.horizon/config.dt), speed=float(snapshot.speed),
        yaw_rate=float(snapshot.state[3]), cruise=float(cruise), release=float(release),
        minimum_hold=float(minimum_hold), speed_end=float(speed_end), heading=float(heading),
        angle=float(-np.deg2rad(config.descent_angle_deg)),
        target_z=float(cruise if config.target_altitude is None else config.target_altitude),
        target_radius=float(config.target_radius), position_error=float(config.position_error),
        latest_arrival=float(config.latest_nominal_arrival),
        rich=int(config.cruise_levels > 1 or config.speed_profile != 'constant'),
        levels=_array(levels), speed_reference=_array(speed_reference),
        speed_derivative=_array(speed_derivative), gate=_array(gate), direction=_array(direction),
        normal=_array(normal), target=_array(config.target), waypoints=_array(waypoints))
    prefix, reason = _path(evaluate('prefix', from_snapshot(snapshot), (), config, **recipe))
    if reason:
        return reject(diagnostics, reason)
    states, controls, phases, arrival = prefix
    first = len(controls)
    exit_waypoints = []
    if config.obstacles:
        exit_point = states[-1, :2] + (recipe['steps']-first)*config.dt*snapshot.speed*direction
        exit_waypoints = avoidance_waypoints(states[-1, :2], [exit_point], config, snapshot.speed, candidate_id)
    suffix, reason = _path(evaluate('onward', states[-1], _array(exit_waypoints).reshape(-1, 2),
        config, **recipe, first_step=first))
    if reason:
        return reject(diagnostics, reason)
    tail, commands, tail_phases, _ = suffix
    return np.concatenate((states, tail[1:])), np.concatenate((controls, commands)), phases+tail_phases, arrival
