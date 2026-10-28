"""Offline proposal hints from sampled pair conflicts, never safety certificates."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ConflictWitness:
    drone_ids: tuple[int, int]
    candidate_ids: tuple[int, int]
    sample_time_s: float
    positions: tuple[tuple[float, ...], tuple[float, ...]]
    lower_bound_m: float
    required_m: float

    @property
    def slack(self):
        return self.lower_bound_m - self.required_m


def witness(first, second, lower, required):
    a, b = sorted((first, second), key=lambda p: p.drone_id)
    index = int(np.argmin(np.linalg.norm(a.positions - b.positions, axis=1)))
    return ConflictWitness((a.drone_id, b.drone_id), (a.candidate_id, b.candidate_id),
                           float(index * a.dt),
                           (tuple(map(float, a.positions[index])), tuple(map(float, b.positions[index]))),
                           float(lower), float(required))


@dataclass(frozen=True)
class RepairHint:
    conflict: ConflictWitness
    drone_id: int

    @property
    def side(self):
        return self.conflict.drone_ids.index(self.drone_id)

    @property
    def own_position(self):
        return np.asarray(self.conflict.positions[self.side])

    @property
    def other_position(self):
        return np.asarray(self.conflict.positions[1 - self.side])

    def modes(self, config):
        modes = []
        if config.dimensions == 3 and config.cruise_altitude_range is not None:
            modes.append("altitude")
        if config.detour > 0:
            modes.append("lateral")
        if config.max_extra_loiter > config.min_extra_loiter:
            modes.append("departure")
        return modes

    def lateral_waypoints(self, start, difference, normal, waypoints, detour):
        """Bias one optional waypoint near the conflict's projected abscissa.

        Mandatory approach gates and the destination are never changed. The
        lateral displacement stays within the existing proposal detour scale.
        """
        fraction = float(np.clip(np.dot(self.own_position[:2] - start, difference)
                                 / max(float(np.dot(difference, difference)), 1e-9), .15, .85))
        away = float(np.dot(self.own_position[:2] - self.other_position[:2], normal))
        sign = np.sign(away) if abs(away) > 1e-9 else (-1 if self.side == 0 else 1)
        index = 0 if fraction < .475 else 1
        waypoints[index] = start + fraction * difference + sign * detour * normal

    def cruise_level(self, low, high):
        # Separate pair endpoints while preserving their witness altitude order.
        # Coincident altitudes use opposite ends, deterministically by ID.
        own, other = self.own_position[2], self.other_position[2]
        above = own > other if abs(own - other) > 1e-9 else self.side == 1
        return float(high if above else low)

    def release_time(self, config):
        delay = config.min_extra_loiter if self.side == 0 else config.max_extra_loiter
        return config.activation_time + delay
