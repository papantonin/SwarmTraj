"""Geometric vertical proposals, and exact independent-Z legacy primitives.

This is a 2.5D kinematic model: speed denotes horizontal speed; total speed
increases during climbs. Coupled flight uses these only as geometric proposals,
retimes them and rolls out the actual point-mass dynamics before validation.
"""

import numpy as np


def descent_geometry(speed, config):
    """Final constant-slope passage followed by a bounded vertical recovery."""
    angle = np.deg2rad(config.descent_angle_deg)
    rate = speed * (np.sin(angle) if config.coupled_flight else np.tan(angle))
    ramp_steps = max(1, int(np.ceil(rate / (config.max_vertical_acceleration * config.dt))))
    level_steps = int(np.ceil(max(1.0, config.target_radius / speed + config.clock_error + config.dt) / config.dt))
    drop = rate * (0.5 * ramp_steps + level_steps) * config.dt
    recovery_drop = rate * ramp_steps * config.dt / 2
    return rate, ramp_steps, level_steps, drop, recovery_drop


def descent_error(snapshot, config):
    if config.descent_angle_deg == 0:
        return None
    rate, _, _, drop, recovery_drop = descent_geometry(snapshot.speed, config)
    if rate > config.descent_rate_limit + 1e-9:
        maximum = np.rad2deg(np.arcsin(min(1, config.descent_rate_limit / snapshot.speed))
                            if config.coupled_flight else np.arctan2(config.descent_rate_limit, snapshot.speed))
        return f"Drone {snapshot.drone_id}: descent angle limited to {maximum:.1f}° at {snapshot.speed:g} m/s."
    if config.coupled_flight:
        if config.descent_angle_deg > config.flight.max_flight_path_deg:
            return f"Drone {snapshot.drone_id}: flight-path angle exceeds the flight-model limit."
        angle = np.deg2rad(config.descent_angle_deg)
        # Necessary steady straight-flight trim, not a proof of a feasible approach.
        load = np.cos(angle)
        axial = config.flight.drag(snapshot.speed, load) - 9.81 * np.sin(angle)
        if axial < -config.flight.max_brake_acceleration - 1e-9:
            return f"Drone {snapshot.drone_id}: Insufficient drag/airbrake authority to maintain airspeed on this slope."
    if config.target_altitude - recovery_drop < config.min_altitude - 1e-8:
        return f"Drone {snapshot.drone_id}: target altitude too low for the post-passage pull-out."
    ceiling = (config.cruise_altitude_range[1] if config.cruise_altitude_range is not None else
               snapshot.altitude if snapshot.cruise_altitude is None else snapshot.cruise_altitude)
    if ceiling < config.target_altitude + drop - 1e-8:
        return f"Drone {snapshot.drone_id}: cruise altitude too low to establish the requested descent angle."
    return None


def transition(displacement, dt, max_rate, max_acceleration):
    """Rest-to-rest trapezoid, snapped to the grid without exceeding bounds."""
    distance = abs(displacement)
    if distance < 1e-10:
        return np.zeros(0)
    ramp_steps = max(1, min(int(np.ceil(np.sqrt(distance / max_acceleration) / dt)),
                            int(np.ceil(max_rate / (max_acceleration * dt)))))
    ramp_time = ramp_steps * dt
    flat_steps = max(0, int(np.ceil((distance / max_rate - ramp_time) / dt)))
    acceleration = displacement / (ramp_time * (ramp_time + flat_steps * dt))
    return np.r_[np.full(ramp_steps, acceleration), np.zeros(flat_steps),
                 np.full(ramp_steps, -acceleration)]


def bounded_transition(displacement, config):
    limit = config.descent_rate_limit if displacement < 0 else config.max_climb_rate
    return transition(displacement, config.dt, limit, config.max_vertical_acceleration)


def build_profile(snapshot, config, release, arrival, rng, diagnostics=None, *, profile_info=None):
    """Hold, reach cruise, optionally merge at target altitude and return to cruise.

    Reject profiles that cannot fit, rather than teleporting or extending delays.
    """
    count = round(config.horizon / config.dt)
    from .catalog import reject
    cruise = snapshot.altitude if snapshot.cruise_altitude is None else snapshot.cruise_altitude
    acceleration = np.zeros(count)
    climb = bounded_transition(cruise - snapshot.altitude, config)
    start = int(np.ceil(release / config.dt - 1e-10))
    if start + len(climb) > min(count, round(arrival / config.dt)):
        return reject(diagnostics, "vertical_profile", "climb_cannot_finish_before_arrival")
    acceleration[start:start + len(climb)] = climb
    free_end = round(arrival / config.dt)
    if config.descent_angle_deg > 0:
        rate, ramp_steps, level_steps, drop, recovery_drop = descent_geometry(snapshot.speed, config)
        if descent_error(snapshot, config) is not None or cruise < config.target_altitude + drop - 1e-8:
            return reject(diagnostics, "vertical_profile", "descent_geometry_not_feasible")
        stage = bounded_transition(config.target_altitude + drop - cruise, config)
        slope = np.r_[np.full(ramp_steps, -rate / (ramp_steps * config.dt)), np.zeros(level_steps)]
        descent = np.r_[stage, slope]
        arrival_step = round(arrival / config.dt)
        descent_start = arrival_step - len(descent)
        free_end = descent_start
        recovery = np.full(ramp_steps, rate / (ramp_steps * config.dt))
        return_to_cruise = bounded_transition(cruise - (config.target_altitude - recovery_drop), config)
        end = arrival_step + len(recovery) + len(return_to_cruise)
        if descent_start < start + len(climb):
            return reject(diagnostics, "vertical_profile", "insufficient_time_before_descent")
        if end > count:
            if diagnostics is not None:
                diagnostics["required_horizon_s"] = end * config.dt
            return reject(diagnostics, "vertical_profile", "recovery_exceeds_horizon")
        acceleration[descent_start:arrival_step] = descent
        acceleration[arrival_step:arrival_step + len(recovery)] = recovery
        acceleration[arrival_step + len(recovery):end] = return_to_cruise
    elif config.target_altitude is not None:
        merge = bounded_transition(config.target_altitude - cruise, config)
        return_to_cruise = bounded_transition(cruise - config.target_altitude, config)
        # Vary merge timing without changing the assigned departure time.
        lead = int(np.ceil(rng.uniform(1, 4) / config.dt))
        arrival_step = round(arrival / config.dt)
        merge_start = arrival_step - lead - len(merge)
        free_end = merge_start
        # In automatic mode, separate vertically immediately after the passage;
        # a fixed level-flight hold at the common altitude creates avoidable conflicts.
        exit_hold = 0 if config.cruise_altitude_range is not None else 2
        return_start = arrival_step + int(np.ceil(exit_hold / config.dt))
        if merge_start < start + len(climb):
            return reject(diagnostics, "vertical_profile", "insufficient_time_before_target_altitude")
        if return_start + len(return_to_cruise) > count:
            if diagnostics is not None:
                diagnostics["required_horizon_s"] = (return_start + len(return_to_cruise)) * config.dt
            return reject(diagnostics, "vertical_profile", "recovery_exceeds_horizon")
        acceleration[merge_start:merge_start + len(merge)] = merge
        acceleration[return_start:return_start + len(return_to_cruise)] = return_to_cruise
    if config.cruise_levels > 1:
        from .rich_profiles import insert_levels
        levels = insert_levels(acceleration, snapshot, config, cruise, start + len(climb), free_end, rng)
        if levels is None:
            return reject(diagnostics, "vertical_profile", "insufficient_time_or_range_for_cruise_levels")
        if profile_info is not None:
            profile_info["levels"] = levels
    velocity = np.r_[0.0, np.cumsum(acceleration) * config.dt]
    altitude = np.r_[snapshot.altitude, snapshot.altitude + np.cumsum(
        velocity[:-1] * config.dt + 0.5 * acceleration * config.dt**2)]
    return np.column_stack((altitude, velocity)), acceleration
