"""Immutable transport of native departure-window forecasts; no Python planner."""

from dataclasses import dataclass



@dataclass(frozen=True)
class DepartureWindows:
    parent_candidate_id: int
    reference_drone_ids: tuple[int, ...]
    intervals_s: tuple[tuple[float, float], ...]
    constraints: int
    budget_exhausted: bool = False
