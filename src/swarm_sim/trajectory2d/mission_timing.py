"""Separate arrival deadlines, post-passage observation and diagnostic advice.

Horizon suggestions are heuristics, not feasibility certificates. They never
change a mission parameter or extend an explicit arrival deadline.
"""

import math

import numpy as np


def recovery_completion(plan):
    """First grid time after passage with cruise settled through the endpoint.

    This is descriptive metadata on already validated plans, not a new safety
    checker. Coupled validation has its own refined flight-envelope checks.
    """
    if plan.vertical_states is None or plan.cruise_altitude is None:
        return plan.arrival_time
    times = np.arange(len(plan.states)) * plan.dt
    settled = ((np.abs(plan.vertical_states[:, 0] - plan.cruise_altitude) <= 0.2)
               & (np.abs(plan.vertical_states[:, 1]) <= 0.1))
    settled_through_end = np.logical_and.accumulate(settled[::-1])[::-1]
    indices = np.flatnonzero((times >= plan.arrival_time) & settled_through_end)
    return float(times[indices[0]]) if len(indices) else None


def schedule(plans, config):
    return [{"drone_id": p.drone_id, "departure_s": p.release_time,
             "arrival_s": p.arrival_time, "recovery_completed_s": recovery_completion(p),
             "post_arrival_available_s": config.horizon - p.arrival_time,
             "routing_mode": p.routing_mode, "cruise_altitude_m": p.cruise_altitude}
            for p in plans]


def timing_diagnostics(config, catalogs, snapshots, plans=()):
    details = {}
    for catalog in catalogs:
        for reason, count in catalog.rejection_details.items():
            details[reason] = details.get(reason, 0) + count
    temporal = any(details.get(key, 0) for key in (
        "target_not_reached_within_horizon", "post_arrival_window_exceeds_horizon",
        "recovery_exceeds_horizon", "validation_arrival_horizon", "validation_cruise_recovery"))
    suggested = None
    advice = []
    template = []
    if config.coupled_flight and config.descent_angle_deg > 0:
        from .vertical import descent_geometry
        for snapshot in snapshots:
            local = config.for_drone(snapshot.drone_id)
            rate = snapshot.speed * np.tan(np.deg2rad(config.descent_angle_deg))
            steps = max(1, int(np.ceil(rate / (.9 * local.max_vertical_acceleration * config.dt))))
            geometric_drop = rate * steps * config.dt / 2
            physical_drop = descent_geometry(snapshot.speed, local)[4]
            available = config.target_altitude - config.min_altitude
            limitation = bool(geometric_drop > available + 1e-8 and physical_drop <= available + 1e-8)
            template.append({"drone_id": snapshot.drone_id, "geometric_recovery_drop_m": float(geometric_drop),
                "coupled_preflight_recovery_drop_m": float(physical_drop), "available_height_m": float(available),
                "conservative_template_limitation": limitation})
            if limitation:
                advice.append(f"Drone {snapshot.drone_id}: the conservative geometric recovery template exceeds "
                    "available height although the coupled preflight check does not. This is a generator limitation, "
                    "not physical infeasibility; review explicit speed, slope or target-height settings.")
    if not plans and temporal:
        hints = [c.minimum_required_horizon_s for c in catalogs if c.minimum_required_horizon_s is not None]
        # Grid-aligned, deliberately heuristic; geometry and timings change on retry.
        value = max([1.5 * config.horizon, *hints])
        suggested = math.ceil(math.ceil(value / 10) * 10 / config.dt) * config.dt
        advice.append("Try a longer horizon to cover routing, target passage and recovery; this does not guarantee a solution.")
    if not plans and (details.get("arrival_deadline_exceeded", 0) or config.arrival_deadline is not None):
        advice.append("An explicit arrival deadline remains binding: increasing horizon alone cannot relax it.")
    if not plans and any(details.get(key, 0) for key in (
            "climb_cannot_finish_before_arrival", "insufficient_time_before_descent",
            "insufficient_time_before_target_altitude")):
        advice.append("Some proposals leave insufficient time for altitude transitions before arrival; review cruise range or route geometry.")
    if not plans and details.get("obstacle_clearance", 0):
        advice.append("Integrated paths violate robust obstacle clearance; review routing alternatives, not safety margins.")
    return {"horizon_s": config.horizon, "arrival_deadline_s": config.arrival_deadline,
            "minimum_post_arrival_s": config.post_arrival_time,
            "latest_nominal_arrival_s": config.latest_nominal_arrival,
            "suggested_horizon_s": suggested, "suggestion_is_feasibility_guarantee": False,
            "rejection_details": dict(sorted(details.items())), "advice": advice,
            "geometric_template": template,
            "per_drone": [{"drone_id": s.drone_id,
                           "minimum_observed_required_horizon_s": c.minimum_required_horizon_s}
                          for s, c in zip(snapshots, catalogs)],
            "schedule": schedule(plans, config)}
