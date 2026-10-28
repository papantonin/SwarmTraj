"""Offline intermediate-cell timing forecasts; every rollout is revalidated."""

from dataclasses import dataclass
import numpy as np
from scipy.integrate import cumulative_trapezoid

from .reservations import occupancy, safe_intervals


@dataclass(frozen=True)
class PassageWindows:
    parent_candidate_id: int
    reference_drone_ids: tuple[int, ...]
    cell: tuple[int, ...] | None
    anchor: tuple[float, ...]
    nominal_time_s: float
    transit_duration_s: float
    intervals_s: tuple[tuple[float, float], ...]
    constraints: int
    budget_exhausted: bool = False


def passage_windows(parent, own, others, config):
    """Choose one conflicting pre-terminal cell and complement its blockers.

    References are provisional offline alternatives, never fleet commitments.
    The occupancy inflation already accounts for the configured tube/clock
    assumptions; all closed blocker endpoints get an additional numerical guard.
    """
    ids = tuple(sorted(o.drone_id for o in others))
    count, choices = 0, []
    midpoint = (parent.release_time + parent.arrival_time) / 2
    shared = own.cells.keys() & {cell for o in others for cell in o.cells}
    for cell in sorted(shared):
        blocked = [interval for o in sorted(others, key=lambda o: o.drone_id)
                   for interval in o.cells.get(cell, ())]
        count += len(blocked) * len(own.cells[cell])
        if count > config.interval_max_constraints:
            return PassageWindows(parent.candidate_id, ids, None, (), 0., 0., (), count, True)
        eligible = [(a, b) for a, b in own.cells[cell]
                    if a >= parent.release_time + 16 and b <= parent.arrival_time - 20
                    and any(max(a, c) <= min(b, d) for c, d in blocked)]
        if not eligible:
            continue
        guarded = [(max(0., a - 1e-6), min(config.horizon, b + 1e-6)) for a, b in blocked]
        free = tuple(safe_intervals(guarded, config.horizon))
        for a, b in eligible:
            center = (a + b) / 2
            choices.append(((abs(center - midpoint), cell, a), cell, center, b - a, free))
    if not choices:
        return None
    _, cell, center, duration, free = min(choices, key=lambda item: item[0])
    times = np.arange(len(parent.positions)) * parent.dt
    anchor = tuple(float(np.interp(center, times, parent.positions[:, j]))
                   for j in range(parent.positions.shape[1]))
    return PassageWindows(parent.candidate_id, ids, cell, anchor, float(center), float(duration), free, count)


def speed_schedule_for_window(proposal, config, fine, geometric_speed, windows, diagnostics=None):
    """Fit a bounded C1 local bump BEFORE physical rollout, never retime a plan.

    A 40-step monotone bisection targets a reachable window center on the
    geometric-reference clock. It does not certify the integrated cell visit.
    Holding, final approach and recovery retain their initial speed setpoint.
    """
    if config.speed_profile != "smooth" or not windows.intervals_s or windows.cell is None:
        return None
    points = proposal.sample(fine)
    xyz = np.column_stack((points[:, :2], points[:, 4]))
    allowed = (fine >= proposal.release_time + 12) & (fine <= proposal.arrival_time - 18)
    if not np.any(allowed):
        return None
    distance = np.linalg.norm(xyz - windows.anchor, axis=1)
    critical = float(fine[np.argmin(np.where(allowed, distance, np.inf))])
    start, end = proposal.release_time + 4., min(critical - 2., proposal.arrival_time - 18.)
    if end - start < 8:
        return None
    low, high = config.speed_profile_range
    cap = config.speed_profile_acceleration * (end - start) * proposal.speed / (np.pi * high)
    amin, amax = -min(proposal.speed - low, cap), min(high - proposal.speed, cap)
    u = (fine - start) / (end - start)
    active = (u > 0) & (u < 1)
    shape, slope = np.zeros_like(fine), np.zeros_like(fine)
    shape[active] = np.sin(np.pi * u[active])**2
    slope[active] = np.pi / (end - start) * np.sin(2 * np.pi * u[active])

    def at(amplitude):
        clock = cumulative_trapezoid(geometric_speed / (proposal.speed + amplitude * shape), fine, initial=0)
        return float(np.interp(critical, fine, clock))

    earliest, latest = at(amax), at(amin)
    # Forecast only: physical footprint dwell changes with speed. Actual
    # inflated occupancy must independently fit the chosen window afterward.
    half_span = windows.transit_duration_s / 2 * max(1., proposal.speed / low) + 2 * config.dt
    choices = []
    for a, b in windows.intervals_s:
        left, right = max(a + half_span, earliest), min(b - half_span, latest)
        if left <= right:
            desired = (left + right) / 2
            choices.append((abs(desired - windows.nominal_time_s), a, b, desired))
    if not choices:
        return None
    _, a, b, desired = min(choices)
    left, right = amin, amax
    for _ in range(40):
        middle = (left + right) / 2
        if at(middle) > desired:
            left = middle
        else:
            right = middle
    amplitude = (left + right) / 2
    if abs(amplitude) < .2:
        return None
    if diagnostics is not None:
        diagnostics["passage_fit"] = {"selected_window_s": (a, b), "reference_passage_s": at(amplitude),
            "geometric_passage_s": critical, "speed_amplitude_m_s": amplitude,
            "bump_interval_s": (start, end), "reachable_reference_times_s": (earliest, latest)}
    return proposal.speed + amplitude * shape, amplitude * slope


def verify_passage(plan, windows, selected_window, config):
    """Rebuild inflated occupancy; all visits must fit this one selected window."""
    visits = occupancy(plan, config).cells.get(windows.cell, ())
    a, b = selected_window
    valid = bool(visits) and all(start >= a and end <= b for start, end in visits)
    return valid, visits
