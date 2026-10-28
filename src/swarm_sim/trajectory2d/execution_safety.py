"""Observer-only envelope diagnostics and a shared execution acceptance gate.

These checks report numerical evidence, not real-time certified monitors. No
observer events or other aircraft's state feed back into a local controller.
"""

import numpy as np

from .flight3d import air_velocity, derivative


def envelope_margins(states, controls, config):
    s, u = np.asarray(states), np.asarray(controls)
    f, limits = config.flight, config.limits
    d = derivative(s, u, f)
    az = d[..., 3] * np.sin(s[..., 5]) + s[..., 3] * np.cos(s[..., 5]) * d[..., 5]
    margins = {
        "minimum_airspeed": s[..., 3] - limits.min_speed,
        "maximum_airspeed": limits.max_speed - s[..., 3],
        "bank": np.deg2rad(limits.max_bank_deg) - np.abs(s[..., 6]),
        "flight_path_angle": np.deg2rad(f.max_flight_path_deg) - np.abs(s[..., 5]),
        "minimum_altitude": s[..., 2] - config.min_altitude,
        "maximum_altitude": config.max_altitude - s[..., 2],
        "climb_rate": config.max_climb_rate - air_velocity(s)[..., 2],
        "descent_rate": config.descent_rate_limit + air_velocity(s)[..., 2],
        "vertical_acceleration": config.max_vertical_acceleration - np.abs(az),
        "flight_path_rate": np.deg2rad(f.max_flight_path_rate_deg_s) - np.abs(d[..., 5]),
        "minimum_thrust": u[..., 0] + np.zeros_like(s[..., 3]),
        "maximum_thrust": f.max_thrust_acceleration - u[..., 0] + np.zeros_like(s[..., 3]),
        "minimum_brake": u[..., 1] + np.zeros_like(s[..., 3]),
        "maximum_brake": f.max_brake_acceleration - u[..., 1] + np.zeros_like(s[..., 3]),
        "minimum_load": u[..., 2] - f.min_load_factor + np.zeros_like(s[..., 3]),
        "loaded_stall_or_structure": f.load_ceiling(s[..., 3]) - u[..., 2],
        "roll_rate": np.deg2rad(limits.max_roll_rate_deg_s) - np.abs(u[..., 3]) + np.zeros_like(s[..., 3]),
    }
    return margins


def flight_diagnostics(states, controls, times, plans, config, boundaries):
    configs = [config.for_drone(p.drone_id) for p in plans]
    per_drone = [envelope_margins(states[:, k], controls[:, k], c) for k, c in enumerate(configs)]
    outgoing = {name: np.stack([m[name] for m in per_drone], axis=1) for name in per_drone[0]}
    incoming = [envelope_margins(s, u, c) for (_, s, u), c in zip(boundaries, configs)]
    events = []
    minima = {}
    for name, values in outgoing.items():
        minima[name] = [min(float(values[:, k].min()), float(incoming[k][name].min())) for k in range(len(plans))]
        for k, plan in enumerate(plans):
            tolerance = 1e-8 if name in ('minimum_altitude', 'maximum_altitude', 'climb_rate', 'descent_rate') else 1e-5
            bad = np.flatnonzero(values[:, k] < -tolerance)
            bad_in = np.flatnonzero(incoming[k][name] < -tolerance)
            candidates = ([float(times[bad[0]])] if len(bad) else []) + (
                [float(boundaries[k][0][bad_in[0]])] if len(bad_in) else [])
            if candidates:
                events.append({"kind": "physical_envelope", "constraint": name,
                               "drone_id": plan.drone_id, "first_observed_time_s": min(candidates),
                               "minimum_margin": minima[name][k],
                               "margin_unit": "rad" if name in ("bank", "flight_path_angle") else
                                              "rad/s" if name in ("flight_path_rate", "roll_rate") else
                                              "m" if "altitude" in name else
                                              "m/s" if name in ("minimum_airspeed", "maximum_airspeed", "climb_rate", "descent_rate") else
                                              "g" if name in ("minimum_load", "loaded_stall_or_structure") else "m/s²"})
    return {"minimum_flight_envelope_margins_per_drone": minima,
            "maximum_absolute_vertical_acceleration_m_s2": [
                c.max_vertical_acceleration - value for c, value in zip(configs, minima['vertical_acceleration'])]}, events


def first_bound_event(kind, values, limit, times, **labels):
    bad = np.flatnonzero(values > limit)
    if not len(bad):
        return None
    return {"kind": kind, **labels, "first_observed_time_s": float(times[bad[0]]),
            "maximum_observed_or_bounded_value": float(np.max(values)), "limit": float(limit)}


def execution_failures(report):
    keys = ["all_arrived", "tube_conditions_verified", "separation_verified_in_execution_model",
            "arrival_deadline_verified_in_execution", "post_arrival_time_verified_in_execution",
            "approach_sectors_verified_in_execution", "vertical_limits_verified",
            "descent_angle_verified_in_execution", "obstacle_clearance_verified_in_execution_model",
            "controller_protection_verified"]
    if report['flight_model'] == 'point_mass':
        keys.append('physical_limits_verified')
        if 'contact_events_verified' in report:
            keys.append('contact_events_verified')
    if report.get('reservation_policy') == 'grid':
        keys.append('reservation_adherence_verified')
    return [key for key in keys if not report[key]]
