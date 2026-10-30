"""Explicit native kernels; Python dynamics/validation remain reference oracles.

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


def require_execution_native():
    module = require_native()
    if module.build_info().get('execution_api') != 1:
        raise RuntimeError('C++ execution kernel is unavailable; rebuild with ./scripts/build_native.sh')
    return module


def require_validation_native():
    module = require_native()
    if module.build_info().get('validation_api') != 1:
        raise RuntimeError('C++ validation kernel is unavailable; rebuild with ./scripts/build_native.sh')
    return module


def validation_options(config):
    return dict(steps=round(config.horizon/config.dt), activation=float(config.activation_time),
        min_delay=float(config.min_extra_loiter), max_delay=float(config.max_extra_loiter),
        latest_arrival=float(config.latest_nominal_arrival), target=_array(config.target),
        target_altitude=config.target_altitude, target_radius=float(config.target_radius),
        position_error=float(config.position_error), descent_degrees=float(config.descent_angle_deg),
        spread_degrees=float(config.approach_spread_deg),
        cruise_range=None if config.cruise_altitude_range is None else _array(config.cruise_altitude_range),
        cruise_levels=config.cruise_levels, smooth_speed=config.speed_profile != 'constant',
        speed_range=_array(config.speed_profile_range if config.speed_profile_range is not None else (0., 0.)),
        speed_acceleration=float(config.speed_profile_acceleration), clock_error=float(config.clock_error),
        aircraft_radius=float(config.aircraft_radius), obstacle_clearance=float(config.obstacle_clearance),
        obstacles=_array([[int(o.kind == 'cylinder'), *o.center, *o.half_extents]
                          for o in config.obstacles]).reshape(-1, 7))


def validate_candidate(plan, snapshot, config, diagnostics=None):
    """Native independent replay after Python input/profile/family binding checks."""
    from .catalog import invalid
    from .flight3d import from_snapshot
    count = round(config.horizon/config.dt)
    if (plan.flight_states is None or plan.flight_controls is None
            or plan.flight_states.shape != (count+1, 7) or plan.flight_controls.shape != (count, 4)
            or plan.states.shape != (count+1, 4) or plan.accelerations.shape != (count,)
            or plan.vertical_states is None or plan.vertical_states.shape != (count+1, 2)
            or plan.vertical_accelerations is None or plan.vertical_accelerations.shape != (count,)
            or plan.flight_parameters != config.flight or plan.airspeed_limit != config.limits.max_speed):
        return invalid(diagnostics, 'validation_metadata')
    record = dict(drone_id=plan.drone_id, candidate_id=plan.candidate_id, speed=plan.speed,
        dt=plan.dt, release=plan.release_time, arrival=plan.arrival_time, cruise=plan.cruise_altitude, airspeed_limit=plan.airspeed_limit,
        approach=plan.approach_angle, states=_array(plan.flight_states), controls=_array(plan.flight_controls),
        projected=_array(np.column_stack((plan.states, plan.vertical_states))),
        acceleration=_array(plan.accelerations), vertical_acceleration=_array(plan.vertical_accelerations),
        levels=_array(plan.cruise_levels_m),
        speed_reference=None if plan.speed_reference is None else _array(plan.speed_reference))
    reason, contact = require_validation_native().validate_candidate(record, validation_options(config),
        parameters(config), _array(from_snapshot(snapshot)), float(snapshot.state[3]), snapshot.cruise_altitude)
    if contact is not None and diagnostics is not None:
        diagnostics['first_contact_time_s'] = contact
    return invalid(diagnostics, reason) if reason else True


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


def propagate_samples(state, control, durations, config, *, substeps=4, disturbance=(0., 0., 0.)):
    """Simulator-only independent integrations; disturbance is never a controller input."""
    require_execution_native()
    result = evaluate('propagate_samples', state, control, config, durations=_array(durations),
                      substeps=substeps, wind=_array(disturbance))
    return np.frombuffer(result, dtype=np.float64).reshape(-1, 7)


def protect_control(state, requested, config, hold_dt):
    require_execution_native()
    result, intervened, feasible = evaluate('protect', state, requested, config, hold_dt=float(hold_dt))
    return np.frombuffer(result, dtype=np.float64), bool(intervened), bool(feasible)


def local_control_update(measured, reference, base, config, measurement_dt, hold_dt, *,
                         estimated_wind, previous_measured=None, previous_control=None):
    """Own measurement/reference/history only; no true wind, peers or observer state."""
    require_execution_native()
    options = dict(reference=_array(reference), estimated_wind=_array(estimated_wind),
                   measurement_dt=float(measurement_dt), hold_dt=float(hold_dt))
    if (previous_measured is None) != (previous_control is None):
        raise ValueError('both own previous measurement and control are required')
    if previous_measured is not None:
        options.update(previous_measured=_array(previous_measured), previous_control=_array(previous_control))
    command, estimate, intervened, feasible = evaluate('execution_control', measured, base, config, **options)
    return (np.frombuffer(command, dtype=np.float64), np.frombuffer(estimate, dtype=np.float64),
            bool(intervened), bool(feasible))


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
