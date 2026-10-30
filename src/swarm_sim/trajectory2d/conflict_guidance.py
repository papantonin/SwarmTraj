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
