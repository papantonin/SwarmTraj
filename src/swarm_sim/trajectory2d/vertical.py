"""Independent analytical descent checks and archived vertical-profile math.

Production candidates are constructed in C++; these helpers validate inputs or
inspect historical records and cannot generate an executable mission.
"""

import numpy as np


def descent_geometry(speed, config):
    """Final constant-slope passage followed by a bounded vertical recovery."""
    speed=config.terminal_speed(speed)
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
    speed=config.terminal_speed(snapshot.speed)
    if rate > config.descent_rate_limit + 1e-9:
        maximum = np.rad2deg(np.arcsin(min(1, config.descent_rate_limit / speed))
                            if config.coupled_flight else np.arctan2(config.descent_rate_limit, speed))
        return f"Drone {snapshot.drone_id}: descent angle limited to {maximum:.1f}° at {speed:g} m/s."
    if config.coupled_flight:
        if config.descent_angle_deg > config.flight.max_flight_path_deg:
            return f"Drone {snapshot.drone_id}: flight-path angle exceeds the flight-model limit."
        angle = np.deg2rad(config.descent_angle_deg)
        # Necessary steady straight-flight trim, not a proof of a feasible approach.
        load = np.cos(angle)
        axial = config.flight.drag(speed, load) - 9.81 * np.sin(angle)
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
