"""Bounded physical mission construction without the legacy vertical template.

XY gates are routing proposals only. Every stored state is obtained by applying
held physical controls to its actual predecessor. No state is snapped onto a
reference, no speed is imposed after integration, and no mission limit changes.
Single/multiple cruise levels and constant/smooth airspeed references are
integrated directly, including their actual holds and speed transitions.
It is deliberately separate from future control-space trajectory optimization.
"""

from functools import lru_cache, wraps

import numpy as np

from .catalog import reject
from .dynamics import G, wrap
from .flight3d import (air_velocity, envelope_ok, from_snapshot, guidance,
                       project, propagate, trim_control)
from .kinodynamic_recovery import pullout_from_state


def _scalar_proposal_cache(function):
    """Cache only immutable proposal scalars, keyed by the complete own config.

    Mutable/unhashable caller inputs retain the uncached path. Never cache
    aircraft states, commands, candidate acceptance or joint safety checks.
    """
    cached = lru_cache(maxsize=256)(function)

    @wraps(function)
    def proposal(*args):
        try:
            hash(args)
        except TypeError:
            return function(*args)
        return cached(*args)

    proposal.cache_clear = cached.cache_clear
    proposal.cache_info = cached.cache_info
    return proposal


def _step(state, control, config):
    """Check both switch sides and held-control interiors before accepting a step."""
    following = propagate(state, control, config.dt, config.flight)
    checked = propagate(state, control, np.array([0., .25, .5, .75, 1.]) * config.dt,
                        config.flight, substeps=4)
    # One batch retains the refined samples AND the nominal RK4 endpoint. They
    # are distinct integrations; neither may stand in for the other.
    if not envelope_ok(np.concatenate((checked, following[None, :])), control, config):
        return None
    return following


def _control(state, speed, gamma_rate, config, *, zero_bank=False):
    """Physical force request, with speed feedback and no kinematic state reset."""
    flight = config.flight
    rate = min(np.deg2rad(flight.max_flight_path_rate_deg_s),
               .9 * config.max_vertical_acceleration / state[3])
    gamma_rate = float(np.clip(gamma_rate, -rate, rate))
    roll = (float(np.clip(-state[6] / config.dt,
                         -np.deg2rad(config.limits.max_roll_rate_deg_s),
                         np.deg2rad(config.limits.max_roll_rate_deg_s))) if zero_bank else 0.)
    mid_bank = state[6] + roll * config.dt / 2
    load = (np.cos(state[5]) + state[3] * gamma_rate / G) / np.cos(mid_bank)
    load = float(np.clip(load, flight.min_load_factor, flight.load_ceiling(state[3])))
    axial = (flight.drag(state[3], load) + G * np.sin(state[5])
             + 1.5 * (speed - state[3]))
    return np.array([np.clip(axial, 0, flight.max_thrust_acceleration),
                     np.clip(-axial, 0, flight.max_brake_acceleration), load, roll])


def _pitch(state, speed, angle, config, steps):
    states, controls = [state], []
    for _ in range(steps):
        if abs(state[5] - angle) <= 1e-4:
            return states, controls
        command = _control(state, speed, (angle - state[5]) / config.dt, config)
        state = _step(state, command, config)
        if state is None:
            return None
        states.append(state)
        controls.append(command)
    return (states, controls) if abs(state[5] - angle) <= 1e-4 else None


def _requested_vz(state, altitude, config):
    """Anticipate vertical braking instead of saturating on altitude error alone."""
    error = altitude - state[2]
    rate = config.max_climb_rate if error >= 0 else config.descent_rate_limit
    speed = min(.6 * abs(error), np.sqrt(1.2 * config.max_vertical_acceleration * abs(error)),
                .85 * rate, state[3] * np.sin(.95 * np.deg2rad(config.flight.max_flight_path_deg)))
    return float(np.copysign(speed, error))


def _altitude_reference(state, speed, altitude, heading, config):
    reference = state.copy()
    reference[3:7] = [speed, heading,
                      np.arcsin(np.clip(_requested_vz(state, altitude, config) / speed, -.95, .95)), 0.]
    return reference


def _altitude_control(state, speed, altitude, config):
    requested_vz = _requested_vz(state, altitude, config)
    gamma = np.arcsin(np.clip(requested_vz / state[3], -.95, .95))
    return _control(state, speed, 1.5 * (gamma - state[5]), config)


def _settled(state, altitude):
    return abs(state[2] - altitude) <= .02 and abs(air_velocity(state)[2]) <= .01


@_scalar_proposal_cache
def _terminal_distance(speed, cruise, config):
    """Measure the approach length by physical rollout, never by a Z trapezoid."""
    if config.numerical_backend == 'cpp':
        from .native_backend import fixture
        return fixture(speed, config, cruise)
    target_z = cruise if config.target_altitude is None else config.target_altitude
    angle = -np.deg2rad(config.descent_angle_deg)
    state = np.array([0., 0., cruise, speed, 0., 0., 0.])
    state = _step(state, _control(state, speed, 0., config), config)
    if state is None:
        return None
    budget = round(config.horizon / config.dt)
    if angle < 0:
        entry = _pitch(state, speed, angle, config, budget)
        if entry is None:
            return None
        state = entry[0][-1]
        # The first entry into the target sphere must occur on the settled slope.
        if state[2] - target_z < (config.target_radius + config.position_error) * np.sin(-angle):
            return None
        for _ in range(budget):
            command = _control(state, speed, 1.5 * (angle - state[5]), config)
            following = _step(state, command, config)
            if following is None:
                return None
            if following[2] <= target_z:
                # Constant-slope interpolation is proposal sizing only. Arrival is
                # independently localized by reintegrated first-contact checks.
                fraction = (state[2] - target_z) / (state[2] - following[2])
                return float(state[0] + fraction * (following[0] - state[0]))
            state = following
    else:
        for _ in range(budget):
            if _settled(state, target_z):
                return float(state[0] + max(2 * config.target_radius, speed))
            state = _step(state, _altitude_control(state, speed, target_z, config), config)
            if state is None:
                return None
    return None


@_scalar_proposal_cache
def _approach_headroom(speed, config):
    """Proposal-only pitch-down loss from a fixed physical boundary fixture."""
    if config.numerical_backend == 'cpp':
        from .native_backend import fixture
        return fixture(speed, config)
    state = np.array([0., 0., config.max_altitude, speed, 0., 0., 0.])
    entry = _pitch(state, speed, -np.deg2rad(config.descent_angle_deg),
                   config, round(config.horizon / config.dt))
    return None if entry is None else float(state[2] - entry[0][-1][2])


def _cruise(snapshot, config, candidate_id, rng, repair_hint, repair_mode):
    if config.cruise_altitude_range is None:
        return snapshot.altitude if snapshot.cruise_altitude is None else snapshot.cruise_altitude
    low, high = config.cruise_altitude_range
    if config.descent_angle_deg > 0:
        # Size headroom from the physical pitch-down primitive in an admissible
        # boundary fixture; this does not validate the actual approach.
        drop = _approach_headroom(snapshot.speed, config)
        if drop is None:
            return None
        low = max(low, config.target_altitude + drop
                  + (2 * config.target_radius + snapshot.speed * config.dt)
                  * np.sin(np.deg2rad(config.descent_angle_deg)))
    if low > high:
        return None
    if repair_hint is not None and repair_mode == 'altitude':
        return repair_hint.cruise_level(low, high)
    if config.obstacles and config.obstacle_routing == 'spatial' and candidate_id % 3:
        from .obstacles import nominal_clearance_required
        clearance = (nominal_clearance_required(config.limits.max_speed, config)
                     + config.limits.max_speed * config.dt / 2 + 3.)
        above = candidate_id % 3 == 1
        choices = []
        for obstacle in config.obstacles:
            boundary = obstacle.center[2] + (obstacle.half_extents[2] + clearance) * (1 if above else -1)
            level = max(low, boundary) if above else min(high, boundary)
            if low <= level <= high:
                choices.append(float(level))
        if choices:
            choices = sorted(set(choices), key=lambda z: (abs(z - snapshot.altitude), z))
            return choices[(candidate_id // 3) % len(choices)]
    # Interleave low-recovery-cost proposals with the original layer and range
    # exploration. A high initial layer is not a mandatory recovery altitude
    # when the mission explicitly permits automatic level selection.
    family = candidate_id % 4
    return float(low if family == 0 else np.clip(snapshot.altitude, low, high)
                 if family == 1 else rng.uniform(low, high))


def _levels(cruise, config, rng):
    """Choose distinct in-range levels; actual attainment is independently checked."""
    levels = [float(cruise)]
    if config.cruise_levels == 1:
        return tuple(levels)
    low, high = config.cruise_altitude_range
    for _ in range(config.cruise_levels - 1):
        choices = [float(v) for v in (levels[-1] - rng.uniform(5., 9.),
                                     levels[-1] + rng.uniform(5., 9.), low, high)
                   if low <= v <= high and all(abs(v - z) >= 5. for z in levels)]
        if not choices:
            return None
        levels.append(choices[int(rng.integers(len(choices)))])
    return tuple(levels)


def _speed_bump(snapshot, config, candidate_id, release, times):
    """C1 reference on the real mission clock; no geometric retiming."""
    reference = np.full(len(times), snapshot.speed)
    derivative = np.zeros(len(times))
    if config.speed_profile == 'constant':
        return reference, derivative, release
    low, high = config.speed_profile_range
    start, end = release + 4., release + 32.
    rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence(
        [config.seed, snapshot.drone_id, candidate_id, 7301])))
    sign = -1 if rng.integers(2) == 0 else 1
    capacity = snapshot.speed - low if sign < 0 else high - snapshot.speed
    if capacity < .3:
        sign *= -1
        capacity = snapshot.speed - low if sign < 0 else high - snapshot.speed
    amplitude = sign * min(capacity * rng.uniform(.6, .95),
                           .8 * config.speed_profile_acceleration * (end - start) / np.pi)
    if abs(amplitude) < .2 or end >= config.latest_nominal_arrival:
        return None
    active = (times > start) & (times < end)
    phase = (times[active] - start) / (end - start)
    reference[active] += amplitude * np.sin(np.pi * phase)**2
    derivative[active] = amplitude * np.pi / (end - start) * np.sin(2 * np.pi * phase)
    return reference, derivative, end


def physical_candidate(snapshot, config, candidate_id, rng, approach_angle=None, diagnostics=None,
                       repair_hint=None, repair_mode=None, departure_override=None,
                       level_hold_override=None, routing_extra_margin=0.):
    """Construct an own-state-only full-horizon plan; callers must validate it."""
    from .planning import Trajectory
    if config.aircraft_profiles:
        raise ValueError('physical construction requires resolved own aircraft physics')
    cruise = _cruise(snapshot, config, candidate_id, rng, repair_hint, repair_mode)
    if cruise is None:
        return reject(diagnostics, 'physical_approach_headroom')
    distance = _terminal_distance(snapshot.speed, cruise, config)
    if distance is None:
        return reject(diagnostics, 'physical_terminal_construction')
    release = config.activation_time + rng.uniform(config.min_extra_loiter, config.max_extra_loiter)
    if candidate_id % 8 == 0:
        # Include the earliest legal boundary instead of relying on a random
        # draw to leave enough time for full recovery in short missions.
        release = config.activation_time + config.min_extra_loiter
    if repair_hint is not None and repair_mode == 'departure':
        release = repair_hint.release_time(config)
    if departure_override is not None:
        if not config.activation_time + config.min_extra_loiter <= departure_override <= config.activation_time + config.max_extra_loiter:
            return reject(diagnostics, 'interval_departure_out_of_bounds')
        release = float(departure_override)
    steps = round(config.horizon / config.dt)
    levels = _levels(cruise, config, rng)
    if levels is None:
        return reject(diagnostics, 'physical_cruise_levels_range')
    schedule = _speed_bump(snapshot, config, candidate_id, release,
                           np.arange(steps + 1) * config.dt)
    if schedule is None:
        return reject(diagnostics, 'physical_speed_profile_range_or_time')
    speed_reference, speed_derivative, speed_end = schedule
    heading = float(wrap(approach_angle + np.pi) if approach_angle is not None else
                    np.arctan2(config.target[1] - snapshot.state[1], config.target[0] - snapshot.state[0]))
    if config.obstacles and approach_angle is None and candidate_id >= 5:
        # A free approach need not keep the designation-to-target heading. A
        # long level transition can otherwise put every terminal join inside
        # the same solid. Preserve initial proposals and assigned sectors;
        # explore a bounded deterministic set only where approach is free.
        offsets = (-30., 30., -60., 60., -90., 90., 180.)
        heading = float(wrap(heading + np.deg2rad(offsets[(candidate_id - 5) % len(offsets)])))
    direction = np.array([np.cos(heading), np.sin(heading)])
    normal = np.array([-direction[1], direction[0]])
    gate = np.asarray(config.target) - distance * direction
    lead = snapshot.speed * (6 + 3 * (candidate_id % 5))
    join = gate - lead * direction
    if repair_hint is not None and repair_mode == 'lateral':
        points = [join.copy(), join.copy()]
        repair_hint.lateral_waypoints(np.asarray(snapshot.state[:2]), join - snapshot.state[:2],
                                     normal, points, config.detour)
        join = points[0]
    waypoints = [join]
    routing_mode = 'lateral'
    if config.obstacles:
        from dataclasses import replace
        from .obstacles import avoidance_waypoints, nominal_clearance_required
        routing = replace(config, obstacle_clearance=config.obstacle_clearance + routing_extra_margin)
        if config.obstacle_routing == 'spatial' and candidate_id % 3:
            clearance = (nominal_clearance_required(config.limits.max_speed, routing)
                         + config.limits.max_speed * config.dt / 2 + 3.)
            blocking = tuple(o for o in config.obstacles
                             if abs(cruise - o.center[2]) - o.half_extents[2] < clearance)
            if len(blocking) < len(config.obstacles):
                routing = replace(routing, obstacles=blocking)
                routing_mode = 'overflight' if candidate_id % 3 == 1 else 'underflight'
        waypoints = avoidance_waypoints(np.asarray(snapshot.state[:2]), waypoints, routing,
                                        snapshot.speed, candidate_id)
    if config.numerical_backend == 'cpp':
        from .native_backend import construct
        result = construct(snapshot, config, cruise=cruise, release=release, levels=levels,
            minimum_hold=config.cruise_level_hold if level_hold_override is None else level_hold_override,
            speed_reference=speed_reference, speed_derivative=speed_derivative, speed_end=speed_end,
            heading=heading, gate=gate, direction=direction, normal=normal, waypoints=waypoints,
            candidate_id=candidate_id, diagnostics=diagnostics)
        if result is None:
            return None
        full, controls, phases, arrival = result
        projected = project(full, np.vstack((controls, controls[-1])), config.flight)
        return Trajectory(snapshot.drone_id, candidate_id, snapshot.speed, config.dt,
            projected[:, :4], np.diff(projected[:, 3])/config.dt, arrival, release,
            approach_angle, projected[:, 4:], np.diff(projected[:, 5])/config.dt, cruise,
            full, controls, config.flight, config.limits.max_speed, routing_mode=routing_mode,
            cruise_levels_m=levels if len(levels)>1 else (),
            speed_reference=speed_reference if config.speed_profile != 'constant' else None,
            generation_phases=phases)
    states, commands, phases = [from_snapshot(snapshot)], [], []

    def append(command):
        if len(commands) >= steps:
            return False
        following = _step(states[-1], command, config)
        if following is None:
            return False
        commands.append(command)
        states.append(following)
        return True

    def phase(name, start):
        if len(commands) > start:
            phases.append((name, start, len(commands)))

    while len(commands) * config.dt < release - 1e-10:
        if not append(trim_control(states[-1], config.flight)):
            return reject(diagnostics, 'physical_loiter')
    phase('loiter', 0)
    minimum_hold = config.cruise_level_hold if level_hold_override is None else level_hold_override
    if config.cruise_levels > 1 or config.speed_profile != 'constant':
        # Physically fly the requested level sequence before committing to the
        # terminal line. Extra maneuvers use the same horizon/departure budget.
        # No instantaneous state assignment or metadata-only level declaration.
        program = (*levels, cruise) if len(levels) > 1 else levels
        for index, altitude in enumerate(program):
            start, held = len(commands), 0
            while len(commands) < steps:
                state = states[-1]
                held = held + 1 if _settled(state, altitude) else 0
                if held * config.dt >= minimum_hold and (
                        index + 1 < len(program) or len(commands) * config.dt >= speed_end + 3.):
                    break
                k = len(commands)
                reference = _altitude_reference(state, speed_reference[k], altitude, state[4], config)
                yaw = np.sign(snapshot.state[3]) * min(.15, .6 * config.limits.max_rate(state[3]))
                command = guidance(state, reference, (yaw, 0.), config, config.dt,
                                   speed_feedforward=float(speed_derivative[k]))
                if not append(command):
                    return reject(diagnostics, 'physical_profile_envelope')
            else:
                return reject(diagnostics, 'physical_profile_horizon')
            phase('cruise_level_' + str(index), start)
    transit_start = len(commands)
    waypoint, on_line = 0, False
    settled_steps = 0
    while len(commands) < steps:
        state = states[-1]
        settled_steps = settled_steps + 1 if _settled(state, cruise) else 0
        if not on_line and np.linalg.norm(state[:2] - waypoints[waypoint]) < 2 * snapshot.speed:
            if waypoint + 1 < len(waypoints):
                waypoint += 1
            else:
                on_line = True
        if on_line:
            cross = float(np.dot(state[:2] - gate, normal))
            along = float(np.dot(state[:2] - gate, direction))
            desired_heading = heading - np.arctan2(cross, 1.5 * snapshot.speed)
            if along >= 0:
                # Choose the nearest existing grid boundary. Truncation is an
                # offline proposal operation, before any validation/selection.
                previous = float(np.dot(states[-2][:2] - gate, direction)) if len(commands) > transit_start else along
                if abs(previous) < abs(along):
                    states.pop()
                    commands.pop()
                    state = states[-1]
                    cross = float(np.dot(state[:2] - gate, normal))
                if (abs(wrap(state[4] - heading)) > np.deg2rad(.4) or abs(cross) > .5
                        or not _settled(state, cruise) or abs(state[3] - snapshot.speed) > .05
                        or settled_steps * config.dt < minimum_hold):
                    return reject(diagnostics, 'physical_approach_alignment')
                break
        else:
            delta = waypoints[waypoint] - state[:2]
            desired_heading = np.arctan2(delta[1], delta[0])
        reference = _altitude_reference(state, snapshot.speed, cruise, desired_heading, config)
        command = guidance(state, reference, (0., 0.), config, config.dt)
        if not append(command):
            return reject(diagnostics, 'physical_transit_envelope')
    else:
        return reject(diagnostics, 'physical_transit_horizon')
    phase('transit', transit_start)
    start = len(commands)
    command = _control(states[-1], snapshot.speed, -states[-1][5] / config.dt, config, zero_bank=True)
    if not append(command) or abs(states[-1][6]) > 1e-12:
        return reject(diagnostics, 'physical_alignment_envelope')
    phase('alignment', start)
    angle = -np.deg2rad(config.descent_angle_deg)
    target_z = cruise if config.target_altitude is None else config.target_altitude
    start = len(commands)
    if angle < 0:
        entry = _pitch(states[-1], snapshot.speed, angle, config, steps - len(commands))
        if entry is None:
            return reject(diagnostics, 'physical_descent_entry')
        # These controls are reintegrated from the actual predecessor, including
        # the alignment's real speed, slope, bank, heading and altitude.
        for command in entry[1]:
            if not append(command):
                return reject(diagnostics, 'physical_descent_entry')
    else:
        while not _settled(states[-1], target_z):
            if not append(_altitude_control(states[-1], snapshot.speed, target_z, config)):
                return reject(diagnostics, 'physical_target_altitude')
    phase('descent_entry' if angle < 0 else 'target_altitude', start)
    start = len(commands)
    target = np.r_[config.target, target_z]
    while len(commands) < steps:
        state = states[-1]
        if np.dot(state[:3] - target, air_velocity(state)) >= 0:
            break
        command = (_control(state, snapshot.speed, 1.5 * (angle - state[5]), config)
                   if angle < 0 else _altitude_control(state, snapshot.speed, target_z, config))
        if not append(command):
            return reject(diagnostics, 'physical_passage_envelope')
    phase('passage', start)
    distances = np.linalg.norm(np.asarray(states)[:, :3] - target, axis=1)
    arrival_index = int(np.argmin(distances))
    arrival = arrival_index * config.dt
    if distances[arrival_index] > config.target_radius - config.position_error:
        return reject(diagnostics, 'physical_target_missed')
    if arrival > config.latest_nominal_arrival + 1e-9:
        return reject(diagnostics, 'physical_arrival_timing')
    start = len(commands)
    if angle < 0:
        recovery = pullout_from_state(states[-1], config,
                                      max_duration_s=(steps - len(commands)) * config.dt)
        if recovery is None:
            return reject(diagnostics, 'physical_pullout')
        for command, endpoint in zip(recovery.controls, recovery.states[1:]):
            if not append(command) or not np.array_equal(states[-1], endpoint):
                return reject(diagnostics, 'physical_pullout_boundary')
    phase('pullout', start)
    start = len(commands)
    recovery_end = None
    exit_waypoints, exit_index = None, 0
    if config.obstacles:
        # Continue flying around known solids after passage; a target contact
        # does not end the planning horizon or remove the aircraft.
        exit_point = states[-1][:2] + (steps - len(commands)) * config.dt * snapshot.speed * direction
        exit_waypoints = avoidance_waypoints(states[-1][:2], [exit_point], config,
                                             snapshot.speed, candidate_id)
    while len(commands) < steps:
        if recovery_end is None and _settled(states[-1], cruise):
            recovery_end = len(commands)
        command = _altitude_control(states[-1], snapshot.speed, cruise, config)
        if exit_waypoints:
            state = states[-1]
            if (exit_index + 1 < len(exit_waypoints)
                    and np.linalg.norm(state[:2] - exit_waypoints[exit_index]) < 2 * snapshot.speed):
                exit_index += 1
            delta = exit_waypoints[exit_index] - state[:2]
            reference = _altitude_reference(state, snapshot.speed, cruise,
                                            np.arctan2(delta[1], delta[0]), config)
            command = guidance(state, reference, (0., 0.), config, config.dt)
        if not append(command):
            return reject(diagnostics, 'physical_cruise_recovery')
    if not _settled(states[-1], cruise):
        return reject(diagnostics, 'physical_recovery_horizon')
    if recovery_end is None:
        recovery_end = steps
    if recovery_end > start:
        phases.append(('cruise_recovery', start, recovery_end))
    if steps > recovery_end:
        phases.append(('onward', recovery_end, steps))
    full, controls = np.asarray(states), np.asarray(commands)
    projected = project(full, np.vstack((controls, controls[-1])), config.flight)
    return Trajectory(snapshot.drone_id, candidate_id, snapshot.speed, config.dt,
        projected[:, :4], np.diff(projected[:, 3]) / config.dt, arrival, release,
        approach_angle, projected[:, 4:], np.diff(projected[:, 5]) / config.dt, cruise,
        full, controls, config.flight, config.limits.max_speed,
        routing_mode=routing_mode,
        cruise_levels_m=levels if len(levels) > 1 else (),
        speed_reference=speed_reference if config.speed_profile != 'constant' else None,
        generation_phases=tuple(phases))
