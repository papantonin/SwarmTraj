"""Physically integrated pull-out primitives, separate from proposal geometry.

This is a local physical primitive, never a mission certificate. It integrates the same
held-control 7-state point-mass model as the existing planner and refuses any
sampled envelope violation. It models only a straight, unbanked post-passage
pull-out and does NOT claim that the preceding approach or the complete mission
is feasible. The existing full-horizon independent validator remains mandatory.

The scalar seed below is retained for historical laboratory comparisons only.
Mission generation uses the actual boundary state and complete held controls.
"""

from dataclasses import dataclass
from functools import lru_cache
from types import SimpleNamespace

import numpy as np

from .dynamics import G
from .flight3d import envelope_ok, propagate


@dataclass(frozen=True)
class Pullout:
    """A sampled, integrated unbanked recovery; arrays are immutable."""
    states: np.ndarray
    controls: np.ndarray
    duration_s: float
    altitude_loss_m: float

    def __post_init__(self):
        for name in ("states", "controls"):
            array = np.array(getattr(self, name), dtype=float, copy=True)
            array.setflags(write=False)
            object.__setattr__(self, name, array)

    def __reduce__(self):
        return type(self), (self.states, self.controls, self.duration_s, self.altitude_loss_m)

    @property
    def entry_state(self):
        return self.states[0]

    @property
    def exit_state(self):
        return self.states[-1]

    @property
    def dt(self):
        return self.duration_s / len(self.controls)


def validate_pullout(primitive, config):
    """Independently reintegrate a primitive under the supplied own physics.

    This sampled local check establishes no approach, obstacle or fleet claim.
    Mission candidates still require the full existing independent validator.
    """
    states, controls = primitive.states, primitive.controls
    n = len(controls)
    if (not config.coupled_flight or getattr(config, 'aircraft_profiles', ()) or n == 0
            or controls.shape != (n, 4) or states.shape != (n + 1, 7)
            or not np.isclose(primitive.duration_s, n * config.dt, rtol=0, atol=1e-10)
            or abs(states[-1, 5]) > 1e-4 or np.abs(states[:, 6]).max() > 1e-12
            or states[0, 5] >= 0
            or not np.isclose(primitive.altitude_loss_m, states[0, 2] - states[:, 2].min(), rtol=0, atol=1e-9)):
        return False
    rolled = propagate(states[:-1], controls, config.dt, config.flight)
    if not np.allclose(rolled, states[1:], atol=1e-9, rtol=0):
        return False
    return all(envelope_ok(propagate(states[:-1], controls, fraction * config.dt,
                                     config.flight, substeps=4), controls, config)
               for fraction in (0., .25, .5, .75, 1.))


def pullout_from_state(initial_state, config, *, max_duration_s=None):
    """Recover from the actual seven-state boundary without resetting any state.

    This first primitive supports unbanked descending boundaries only. A banked
    entry is rejected, never silently flattened. The caller must resolve own
    aircraft physics before calling. Returned controls are held for config.dt.
    """
    state = np.array(initial_state, dtype=float, copy=True)
    if (not config.coupled_flight or state.shape != (7,) or not np.all(np.isfinite(state))
            or state[3] <= 0 or not -np.pi / 2 < state[5] < 0 or abs(state[6]) > 1e-12
            or getattr(config, 'aircraft_profiles', ())):
        return None
    if max_duration_s is not None and (not np.isfinite(max_duration_s) or max_duration_s <= 0):
        return None
    return _integrate_pullout(state, config, max_duration_s)


def simulate_pullout(speed: float, config, *, angle_deg: float | None = None):
    """Integrate an unbanked pull-out beginning at the requested passage slope.

    Return None when it violates the actual flight envelope or the minimum
    altitude. There is no attempt to 'fix' failures by raising safety limits.
    This laboratory boundary condition is NOT an initial aircraft snapshot.
    """
    angle = config.descent_angle_deg if angle_deg is None else angle_deg
    if not np.isfinite(speed) or speed <= 0 or not np.isfinite(angle) or not 0 < angle < 90:
        return None
    if not config.coupled_flight or config.target_altitude is None:
        return None
    target_gamma = -np.deg2rad(angle)
    state = np.array([0.0, 0.0, float(config.target_altitude), float(speed),
                      0.0, target_gamma, 0.0])
    return pullout_from_state(state, config)


def _integrate_pullout(state, config, max_duration_s):
    """Native laboratory primitive with the original bounded step allowance."""
    from .native_backend import evaluate, _path
    rate = min(np.deg2rad(config.flight.max_flight_path_rate_deg_s),
               .97 * config.max_vertical_acceleration / state[3])
    if rate <= 0:
        return None
    steps = int(np.ceil((abs(state[5]) / rate + 2.) / config.dt))
    if max_duration_s is not None:
        steps = min(steps, int(np.floor(max_duration_s / config.dt + 1e-9)))
    path, reason = _path(evaluate('pullout', state, (), config, steps=steps))
    if reason:
        return None
    states, controls, _, _ = path
    return Pullout(states, controls, len(controls) * config.dt,
                   float(states[0, 2] - states[:, 2].min()))


def recovery_seed_acceleration(speed: float, config):
    """Historical scalar laboratory diagnostic, never used to generate missions.

    Repeated candidates for one airframe reuse the same deterministic scalar
    result, without caching trajectories or mutable mission objects.
    """
    if not config.coupled_flight or config.target_altitude is None or config.descent_angle_deg <= 0:
        return None
    return _cached_seed(
        float(speed), float(config.dt), float(config.descent_angle_deg),
        float(config.target_altitude), float(config.min_altitude), float(config.max_altitude),
        float(config.max_climb_rate), float(config.descent_rate_limit),
        float(config.max_vertical_acceleration), config.flight, config.limits,
    )


@lru_cache(maxsize=256)
def _cached_seed(speed, dt, angle, target_altitude, min_altitude, max_altitude,
                 max_climb_rate, descent_rate_limit, max_vertical_acceleration, flight, limits):
    """Only immutable physical inputs enter the cache key."""
    config = SimpleNamespace(
        coupled_flight=True, aircraft_profiles=(), dt=dt, descent_angle_deg=angle,
        target_altitude=target_altitude, min_altitude=min_altitude,
        max_altitude=max_altitude, max_climb_rate=max_climb_rate,
        descent_rate_limit=descent_rate_limit,
        max_vertical_acceleration=max_vertical_acceleration,
        flight=flight, limits=limits,
    )
    trial = simulate_pullout(speed, config)
    if trial is None or trial.altitude_loss_m <= 0:
        return None
    # The 2.5D template encodes the desired passage angle with horizontal speed
    # V and vertical speed V*tan(theta), unlike the 3D total-airspeed model.
    geom_rate = float(speed * np.tan(np.deg2rad(angle)))
    padded_loss = trial.altitude_loss_m + max(0.5, geom_rate * dt / 2)
    accel = geom_rate**2 / (2 * padded_loss)
    return float(accel) if np.isfinite(accel) and accel > 0 else None
