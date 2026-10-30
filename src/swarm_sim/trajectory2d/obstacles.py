"""Known static solids, conservative clearance checks and proposal-only routing.

Signed distances are exact for the configured ideal solids. Trajectory bounds
remain conditional on the numerical motion, speed envelope and tracking tubes.
The XY visibility graph is a heuristic, never the safety validator.
"""

from dataclasses import dataclass
import json

import numpy as np


@dataclass(frozen=True)
class Obstacle:
    kind: str
    center: tuple[float, float, float]
    half_extents: tuple[float, float, float]

    def __post_init__(self):
        if self.kind not in ("box", "cylinder"):
            raise ValueError("Obstacle kind must be box or cylinder")
        if (np.shape(self.center) != (3,) or np.shape(self.half_extents) != (3,)
                or not np.all(np.isfinite([self.center, self.half_extents]))
                or min(self.half_extents) <= 0):
            raise ValueError("Obstacles require finite XYZ center and positive half-extents")
        if self.kind == "cylinder" and self.half_extents[0] != self.half_extents[1]:
            raise ValueError("Cylinder X/Y half-extents must equal its radius")
        object.__setattr__(self, "center", tuple(map(float, self.center)))
        object.__setattr__(self, "half_extents", tuple(map(float, self.half_extents)))

    @classmethod
    def cylinder(cls, x, y, radius, zmin, zmax):
        return cls("cylinder", (x, y, (zmin + zmax) / 2), (radius, radius, (zmax - zmin) / 2))

    @classmethod
    def box(cls, xmin, ymin, zmin, xmax, ymax, zmax):
        low, high = np.array([xmin, ymin, zmin]), np.array([xmax, ymax, zmax])
        return cls("box", tuple((low + high) / 2), tuple((high - low) / 2))

    def distance(self, positions):
        """Signed distance to solid in XYZ, or to its footprint for XY inputs."""
        points = np.asarray(positions, dtype=float)
        if points.shape[-1] not in (2, 3):
            raise ValueError("Obstacle distances require XY or XYZ positions")
        delta = points - self.center[:points.shape[-1]]
        if self.kind == "box":
            q = np.abs(delta) - self.half_extents[:points.shape[-1]]
        else:
            radial = np.linalg.norm(delta[..., :2], axis=-1) - self.half_extents[0]
            if points.shape[-1] == 2:
                return radial
            q = np.stack((radial, np.abs(delta[..., 2]) - self.half_extents[2]), axis=-1)
        return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(np.max(q, axis=-1), 0)


def load_obstacles(path):
    """A JSON list using the same fields as the mission's obstacle records."""
    with open(path, encoding="utf-8") as stream:
        records = json.load(stream)
    if not isinstance(records, list):
        raise ValueError("Obstacle JSON must be a list")
    try:
        return tuple(Obstacle(**record) for record in records)
    except (TypeError, KeyError) as error:
        raise ValueError("Each obstacle needs kind, center and half_extents") from error


def nominal_clearance_required(speed_bound, config):
    return config.aircraft_radius + config.obstacle_clearance + config.position_error + speed_bound * config.clock_error


def plan_clearance(plan, config):
    """Full-horizon lower bound using the 1-Lipschitz signed-distance function."""
    if not config.obstacles:
        return None
    values = [float(np.min(obstacle.distance(plan.positions))) for obstacle in config.obstacles]
    return min(values) - plan.speed_bound * plan.dt / 2


def obstacle_preflight(snapshots, config):
    for snapshot in snapshots:
        local = config.for_drone(snapshot.drone_id)
        point = snapshot.position3d if config.dimensions == 3 else snapshot.state[:2]
        bound = (local.limits.max_speed if config.coupled_flight else
                 np.hypot(snapshot.speed, config.vertical_speed_limit) if config.dimensions == 3 else snapshot.speed)
        for index, obstacle in enumerate(config.obstacles):
            distance = float(obstacle.distance(point))
            if distance < config.obstacle_clearance + local.aircraft_radius:
                return "initial_obstacle_violation", f"Drone {snapshot.drone_id} starts inside obstacle {index}'s clearance zone."
            if distance < nominal_clearance_required(bound, local):
                return "initial_obstacle_margin_not_certified", f"Drone {snapshot.drone_id}: initial robust margin to obstacle {index} is insufficient."
    target = (np.r_[config.target, config.target_altitude] if config.target_altitude is not None
              else config.target if config.dimensions == 2 else None)
    if target is not None:
        for index, obstacle in enumerate(config.obstacles):
            # 1-Lipschitz upper bound: the entire nominal arrival region is
            # forbidden. Partial overlaps are left to actual trajectory checks.
            if float(obstacle.distance(target)) + config.target_radius - config.position_error < config.obstacle_clearance + max(config.for_drone(s.drone_id).aircraft_radius for s in snapshots):
                return "target_region_obstructed", f"The entire nominal arrival region violates clearance to obstacle {index}."
    return None


def execution_clearance(actual_positions, plans, config, sample_dt, wind, speed_bound_valid=True):
    if not config.obstacles:
        return {"obstacle_count": 0, "obstacle_clearance_m": config.obstacle_clearance,
                "minimum_sampled_obstacle_distance_m": None, "minimum_obstacle_distance_lower_bound_m": None,
                "obstacle_clearance_verified_in_execution_model": True, "sampled_obstacle_collision": False}
    sampled = min(float(np.min(o.distance(actual_positions))) for o in config.obstacles)
    sampled_surface = min(float(np.min(o.distance(actual_positions[:, k]))) - config.for_drone(p.drone_id).aircraft_radius
                          for k, p in enumerate(plans) for o in config.obstacles)
    bounds = [config.for_drone(p.drone_id).limits.max_speed if config.coupled_flight else
              float(np.hypot(p.speed, config.vertical_speed_limit)) if config.dimensions == 3 else p.speed
              for p in plans]
    lower = sampled - (max(bounds) + wind) * sample_dt / 2 if speed_bound_valid else None
    surface_lower = min((float(np.min(o.distance(actual_positions[:, k])))
                         - config.for_drone(p.drone_id).aircraft_radius
                         - (bounds[k] + wind) * sample_dt / 2)
                        for k, p in enumerate(plans) for o in config.obstacles) if speed_bound_valid else None
    return {"obstacle_count": len(config.obstacles), "obstacle_clearance_m": config.obstacle_clearance,
            "minimum_sampled_obstacle_distance_m": sampled,
            "minimum_sampled_obstacle_surface_clearance_m": sampled_surface,
            "minimum_obstacle_distance_lower_bound_m": lower,
            "minimum_obstacle_surface_clearance_lower_bound_m": surface_lower,
            "obstacle_clearance_verified_in_execution_model": bool(surface_lower is not None and surface_lower >= config.obstacle_clearance),
            "sampled_obstacle_collision": bool(sampled_surface < 0)}
