"""Conservative full-horizon cell/time resource allocation over flyable catalogs.

Not SIPP or a complete kinodynamic planner. Reservations are additional hard
constraints; they never replace independent integrated separation checks.
"""

from dataclasses import dataclass
from itertools import product
import numpy as np


class ReservationBudgetError(RuntimeError):
    pass


@dataclass(frozen=True)
class Occupancy:
    drone_id: int
    candidate_id: int
    cells: dict
    inflation_m: float

    def to_dict(self):
        return {"drone_id": self.drone_id, "candidate_id": self.candidate_id,
                "inflation_m": self.inflation_m,
                "cells": [{"index": list(cell), "intervals_s": [list(i) for i in intervals]}
                          for cell, intervals in sorted(self.cells.items())]}


def occupancy(plan, config):
    # Sample n represents its temporal Voronoi interval. Its inflated box covers
    # physical motion dt/2, tracking error and bounded clock displacement, plus
    # half the center-to-center safety distance. Boxes overapproximate balls.
    radius = (config.separation / 2 + config.for_drone(plan.drone_id).aircraft_radius + config.position_error
              + plan.speed_bound * (config.clock_error + plan.dt / 2))
    cells = {}
    size = config.reservation_cell_size
    if not np.isfinite(radius) or max(1., 2 * radius / size) > config.reservation_max_entries ** (1 / plan.positions.shape[1]):
        raise ReservationBudgetError("Inflated reservation exceeds the cell/time construction budget.")
    entries = 0
    for n, point in enumerate(plan.positions):
        if np.max(np.abs(point / size)) + radius / size >= np.iinfo(np.int64).max / 2:
            raise ReservationBudgetError("Reservation coordinates exceed the supported integer grid range.")
        lower = np.floor((point - radius) / size).astype(int)
        upper = np.floor((point + radius) / size).astype(int)
        entries += int(np.prod(upper.astype(float) - lower + 1))
        if entries > config.reservation_max_entries:
            raise ReservationBudgetError("Per-candidate cell/time construction budget exhausted; no infeasibility claim.")
        begin, end = max(0., (n - .5) * plan.dt), min(plan.horizon, (n + .5) * plan.dt)
        for cell in product(*(range(int(a), int(b) + 1) for a, b in zip(lower, upper))):
            intervals = cells.setdefault(cell, [])
            if intervals and begin <= intervals[-1][1] + 1e-10:
                intervals[-1] = (intervals[-1][0], end)
            else:
                intervals.append((begin, end))
    return Occupancy(plan.drone_id, plan.candidate_id,
                     {cell: tuple(intervals) for cell, intervals in cells.items()}, radius)


def first_conflict(first, second):
    """Closed interval intersections, including exact simultaneous boundaries."""
    best = None
    for cell in sorted(first.cells.keys() & second.cells.keys()):
        a, b = first.cells[cell], second.cells[cell]
        i = j = 0
        while i < len(a) and j < len(b):
            begin, end = max(a[i][0], b[j][0]), min(a[i][1], b[j][1])
            if begin <= end + 1e-10:
                item = {"cell": list(cell), "start_s": begin, "end_s": max(begin, end),
                        "drone_ids": [first.drone_id, second.drone_id]}
                if best is None or (begin, cell) < (best["start_s"], tuple(best["cell"])):
                    best = item
                break
            if a[i][1] < b[j][1]:
                i += 1
            else:
                j += 1
    return best


def safe_intervals(occupied, horizon):
    """Complement of occupied closed intervals with nextafter-excluded endpoints.

    Diagnostic/search building block only: aircraft cannot hover until a window
    opens. The current solver selects complete dynamically valid trajectories.
    """
    if not np.isfinite(horizon) or horizon <= 0:
        raise ValueError("horizon must be finite and positive")
    merged = []
    for start, end in sorted(occupied):
        if not 0 <= start <= end <= horizon:
            raise ValueError("invalid occupied interval")
        if merged and start <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(end, merged[-1][1]))
        else:
            merged.append((start, end))
    result, cursor = [], 0.
    for start, end in merged:
        if cursor < start:
            result.append((cursor, float(np.nextafter(start, -np.inf))))
        cursor = float(np.nextafter(end, np.inf))
    if cursor < horizon:
        result.append((cursor, horizon))
    return result
