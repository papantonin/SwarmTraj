"""Free departure intervals predicted from offline reference reservations.

Only a proposal forecast: moving the release changes loiter phase and geometry.
Every generated plan still needs independent physics and full joint rechecks.
"""

from dataclasses import dataclass
import numpy as np

from .reservations import safe_intervals


@dataclass(frozen=True)
class DepartureWindows:
    parent_candidate_id: int
    reference_drone_ids: tuple[int, ...]
    intervals_s: tuple[tuple[float, float], ...]
    constraints: int
    budget_exhausted: bool = False

    def choose(self, rng):
        interval = self.intervals_s[int(rng.integers(len(self.intervals_s)))]
        return float(rng.uniform(*interval)), interval


def departure_windows(parent, own, others, config):
    """Subtract forbidden releases induced by intersections of resource intervals.

    Own moving interval [a,b] relative to parent release r intersects a blocker
    [c,d] after a release shift iff new_release belongs to [r+c-b,r+d-a].
    The known holding prefix is not shifted and is always rechecked on rollout.
    """
    low = config.activation_time + config.min_extra_loiter
    high = config.activation_time + config.max_extra_loiter
    forbidden = []
    count = 0
    guard = 1e-6  # Exclude closed boundaries plus pair-check floating tolerance.
    for other in sorted(others, key=lambda p: p.drone_id):
        for cell in sorted(own.cells.keys() & other.cells.keys()):
            for a, b in own.cells[cell]:
                if b < parent.release_time:
                    continue
                a = max(a, parent.release_time)
                for c, d in other.cells[cell]:
                    count += 1
                    if count > config.interval_max_constraints:
                        return DepartureWindows(parent.candidate_id, tuple(sorted(p.drone_id for p in others)), (), count, True)
                    start = max(low, parent.release_time + c - b - guard)
                    end = min(high, parent.release_time + d - a + guard)
                    if start <= end:
                        forbidden.append((start, end))
    if low == high:
        free = () if forbidden else ((low, high),)
    else:
        free = tuple((max(a, low), min(b, high)) for a, b in safe_intervals(forbidden, config.horizon)
                     if max(a, low) <= min(b, high))
    return DepartureWindows(parent.candidate_id, tuple(sorted(p.drone_id for p in others)), free, count)
