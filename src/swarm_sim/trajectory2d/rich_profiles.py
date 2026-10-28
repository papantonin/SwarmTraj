"""Smooth proposal families, never substitutes for coupled physical rollout."""

import numpy as np


def speed_schedule(proposal, config, fine):
    """A C1 speed bump before final approach, on the geometric path parameter.

    Bound dV/dt = dV/dtau * V / geometric_speed conservatively using geometric
    speed >= proposal.speed. Holding and terminal/recovery setpoints stay intact.
    """
    base = np.full_like(fine, proposal.speed, dtype=float)
    derivative = np.zeros_like(base)
    if config.speed_profile == "constant":
        return base, derivative
    low, high = config.speed_profile_range
    start, end = proposal.release_time + 4., proposal.arrival_time - 18.
    if end - start < 8:
        return None
    rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence(
        [config.seed, proposal.drone_id, proposal.candidate_id, 7301])))
    direction = -1 if rng.integers(2) == 0 else 1
    capacity = proposal.speed - low if direction < 0 else high - proposal.speed
    if capacity < .3:
        direction *= -1
        capacity = proposal.speed - low if direction < 0 else high - proposal.speed
    amplitude = direction * min(capacity * rng.uniform(.6, .95),
        config.speed_profile_acceleration * (end - start) * proposal.speed / (np.pi * high))
    if abs(amplitude) < .2:
        return None
    u = (fine - start) / (end - start)
    active = (u > 0) & (u < 1)
    base[active] += amplitude * np.sin(np.pi * u[active])**2
    derivative[active] = amplitude * np.pi / (end - start) * np.sin(2 * np.pi * u[active])
    return base, derivative


def insert_levels(acceleration, snapshot, config, cruise, start, free_end, rng):
    """Insert rest-to-rest bounded transitions and holds before final descent."""
    from .vertical import bounded_transition
    if config.cruise_levels == 1:
        return ()
    low, high = config.cruise_altitude_range
    levels = [float(cruise)]
    hold = max(1, int(np.ceil(config.cruise_level_hold / config.dt)))
    commands = [np.zeros(hold)]
    current = cruise
    for _ in range(config.cruise_levels - 1):
        # Avoid repeatedly asking for large climbs. These are proposal scales,
        # not extra physical limits or automatic range changes.
        possibilities = [v for v in (current - rng.uniform(5, 15), current + rng.uniform(5, 15))
                         if low <= v <= high and all(abs(v - p) >= 5 for p in levels)]
        if not possibilities:
            return None
        following = float(possibilities[int(rng.integers(len(possibilities)))])
        commands.extend((bounded_transition(following - current, config), np.zeros(hold)))
        levels.append(following)
        current = following
    commands.extend((bounded_transition(cruise - current, config), np.zeros(hold)))
    joined = np.concatenate(commands)
    if start + len(joined) > free_end:
        return None
    acceleration[start:start + len(joined)] = joined
    return tuple(levels)
