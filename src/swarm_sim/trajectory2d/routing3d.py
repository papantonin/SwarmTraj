"""Deterministic altitude-level proposals combined with lateral visibility paths.

One cruise level is explored per candidate, not arbitrary 3D free-space search.
All transitions, target approach and onward flight still require independent
integrated XYZ validation. A level-clear obstacle may still block the climb.
"""

from dataclasses import replace

import numpy as np

from .obstacles import (_segment_hits_rectangle, avoidance_waypoints,
                        nominal_clearance_required)
from .vertical import descent_geometry


def spatial_cruise_proposal(snapshot, config, speed, candidate_id, start, waypoints):
    """Return (mode, forced level); infeasible alternatives fall back to lateral.

    Families interleave lateral / above / below and vary obstacle-derived levels.
    No extra RNG draws are used, so serial/resumed/spawn catalogs agree.
    """
    if config.dimensions != 3 or config.obstacle_routing != "spatial" or not config.obstacles:
        return "lateral", None
    family = candidate_id % 3
    if family == 0:
        return "lateral", None
    radius = speed / config.limits.max_rate(speed)
    clearance = nominal_clearance_required(config.limits.max_speed, config) + config.limits.max_speed * config.dt / 2 + 3
    padding = clearance + 2 * radius + 8
    points = [np.asarray(start), *map(np.asarray, waypoints)]
    relevant = [o for o in config.obstacles if any(
        _segment_hits_rectangle(a, b, np.array(o.center[:2]) - o.half_extents[:2] - padding,
                                 np.array(o.center[:2]) + o.half_extents[:2] + padding)
        for a, b in zip(points[:-1], points[1:]))]
    if not relevant:
        return "lateral", None
    fixed = snapshot.altitude if snapshot.cruise_altitude is None else snapshot.cruise_altitude
    low, high = config.cruise_altitude_range or (fixed, fixed)
    if config.descent_angle_deg > 0:
        low = max(low, config.target_altitude + descent_geometry(speed, config)[3])
    if low > high:
        return "lateral", None
    levels = []
    for obstacle in relevant:
        boundary = (obstacle.center[2] + obstacle.half_extents[2] + clearance if family == 1 else
                    obstacle.center[2] - obstacle.half_extents[2] - clearance)
        lower = max(low, boundary) if family == 1 else low
        upper = high if family == 1 else min(high, boundary)
        if lower <= upper:
            levels.append(float(np.clip(snapshot.altitude, lower, upper)))
    levels = sorted(set(levels), key=lambda z: (abs(z - snapshot.altitude), z))
    if not levels:
        return "lateral", None
    return ("overflight" if family == 1 else "underflight"), levels[(candidate_id // 3) % len(levels)]


def spatial_waypoints(start, waypoints, config, speed, candidate_id, cruise):
    """Ignore vertically cleared solids only on the cruise-route prefix.

    The last approach gate(s) and destination use the full lateral map, as does
    post-arrival routing. Climb/cruise feasibility is never inferred from this
    visibility heuristic; final whole-horizon XYZ checks remain authoritative.
    """
    clearance = nominal_clearance_required(config.limits.max_speed, config) + config.limits.max_speed * config.dt / 2 + 3
    blocking = tuple(o for o in config.obstacles
                     if abs(cruise - o.center[2]) - o.half_extents[2] < clearance - 1e-9)
    prefix_count = max(0, len(waypoints) - 2)
    prefix = avoidance_waypoints(start, waypoints[:prefix_count], replace(config, obstacles=blocking),
                                 speed, candidate_id, optional_count=2)
    last = prefix[-1] if prefix else start
    return [*prefix, *avoidance_waypoints(last, waypoints[prefix_count:], config, speed, candidate_id)]
