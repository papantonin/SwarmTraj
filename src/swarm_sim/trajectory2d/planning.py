"""Offline candidate generation and bounded joint compatibility search."""

from dataclasses import asdict, dataclass, field, fields, replace
from itertools import combinations
import hashlib
from time import perf_counter

import numpy as np

from .dynamics import Limits, Snapshot, propagate, wrap
from .vertical import descent_error
from .flight3d import FlightLimits
from .obstacles import Obstacle


@dataclass(frozen=True)
class Config:
    seed: int = 7
    target: tuple[float, float] = (140.0, 50.0)
    horizon: float = 90.0
    dt: float = 0.2
    activation_time: float = 3.0
    target_radius: float = 6.0
    candidates: int = 28
    attempts: int = 90
    max_search_nodes: int = 30000
    separation: float = 8.0
    position_error: float = 2.0
    clock_error: float = 0.15
    max_extra_loiter: float = 22.0
    min_extra_loiter: float = 0.0
    detour: float = 45.0
    approach_spread_deg: float = 0.0
    limits: Limits = field(default_factory=Limits)
    dimensions: int = 3
    target_altitude: float | None = None
    min_altitude: float = 0.0
    max_altitude: float = 200.0
    max_climb_rate: float = 3.0
    max_vertical_acceleration: float = 1.0
    cruise_altitude_range: tuple[float, float] | None = None
    descent_angle_deg: float = 0.0
    max_descent_rate: float | None = None
    flight_model: str = "point_mass"
    flight: FlightLimits = field(default_factory=FlightLimits)
    approach_assignment: str = "geometric"
    approach_routing: str = "pursuit"  # Historical reference or explicit curvature-bounded tangent proposals.
    planning_workers: int = 1  # Safe serial default; explicit 0 selects automatic spawn workers.
    rollout_check_interval: int = 16  # Same endpoint checks, batched; 1 reproduces the reference path.
    planning_strategy: str = "fixed"  # Library compatibility; CLI defaults to adaptive.
    initial_candidates: int = 4
    search_order: str = "id"
    obstacles: tuple[Obstacle, ...] = ()
    obstacle_clearance: float = 5.0
    selection_policy: str = "first"  # Library compatibility; CLI prefers robust margins.
    efficiency_slack_cap_m: float = 20.  # Ranking saturation only; never changes required separation.
    margin_search_nodes: int = 2000
    margin_expansion_rounds: int = 1
    arrival_deadline: float | None = None  # Absolute mission time; None adds no separate deadline.
    post_arrival_time: float = 5.0  # Minimum observation time, not a fixed recovery duration.
    obstacle_routing: str = "lateral"  # Library compatibility; CLI explores spatial alternatives.
    conflict_guidance: str = "none"  # Opt-in proposal repair; does not alter validation.
    reservation_policy: str = "none"  # Opt-in conservative cell/time allocation.
    reservation_cell_size: float = 20.0
    reservation_max_entries: int = 1000000
    interval_guidance: str = "none"
    interval_max_constraints: int = 50000
    cruise_levels: int = 1
    cruise_level_hold: float = 3.0
    speed_profile: str = "constant"
    speed_profile_range: tuple[float, float] | None = None
    speed_profile_acceleration: float = .3
    profile_policy: str = "strict"  # Mixed families are an explicit opt-in, not a fallback.
    passage_guidance: str = "none"  # Offline intermediate-cell speed fitting, not departure retiming.
    candidate_guidance: str = "none"  # Own-rejection reference adjustments; never weaker validators.
    generator_mode: str | None = None  # Omitted: physical in coupled 3D, geometric in historical modes.
    numerical_backend: str | None = None  # C++ production; explicit historical producer metadata remains readable.
    aircraft_profiles: tuple = ()  # Complete explicit (drone_id, AircraftProfile) bindings; offline only.
    aircraft_radius: float = 0.0  # Enclosing sphere; zero preserves the legacy point-aircraft model.

    def for_drone(self, drone_id):
        """Return own physics and shared mission only, never other airframes."""
        if not self.aircraft_profiles:
            return self
        profile = self.profile_for(drone_id)
        return replace(self, aircraft_profiles=(), **profile.physical_config())

    def profile_for(self, drone_id):
        for identifier, profile in self.aircraft_profiles:
            if identifier == drone_id:
                return profile
        if self.aircraft_profiles:
            raise ValueError(f'no aircraft profile assigned to drone {drone_id}')
        return None

    def require_aircraft_ids(self, ids):
        if self.aircraft_profiles and set(ids) != {i for i, _ in self.aircraft_profiles}:
            raise ValueError('aircraft profile IDs must exactly match the mission IDs')

    @property
    def latest_nominal_arrival(self):
        return min(self.horizon - self.post_arrival_time,
                   self.horizon if self.arrival_deadline is None else self.arrival_deadline) - self.clock_error

    @property
    def coupled_flight(self):
        return self.dimensions == 3 and self.flight_model == "point_mass"

    @property
    def descent_rate_limit(self):
        return self.max_climb_rate if self.max_descent_rate is None else self.max_descent_rate

    @property
    def vertical_speed_limit(self):
        return max(self.max_climb_rate, self.descent_rate_limit)

    def __post_init__(self):
        # Resolve omitted choices once. Persisted/replaced configurations retain
        # their concrete selections; archived records must supply historical defaults.
        if self.generator_mode is None:
            object.__setattr__(self, 'generator_mode', 'physical_mission' if self.coupled_flight else 'geometric')
        if self.numerical_backend is None:
            object.__setattr__(self, 'numerical_backend',
                               'cpp' if self.coupled_flight and self.generator_mode != 'geometric' else 'python')
        from .aircraft_profiles import AircraftProfile
        if (not isinstance(self.aircraft_profiles, tuple)
                or any(not isinstance(pair, tuple) or len(pair) != 2 or type(pair[0]) is not int
                       or pair[0] < 0 or not isinstance(pair[1], AircraftProfile) for pair in self.aircraft_profiles)):
            raise ValueError('aircraft_profiles must contain validated (nonnegative ID, profile) tuples')
        if len({i for i, _ in self.aircraft_profiles}) != len(self.aircraft_profiles):
            raise ValueError('duplicate aircraft profile IDs')
        object.__setattr__(self, 'aircraft_profiles', tuple(sorted(self.aircraft_profiles)))
        if self.aircraft_profiles and not self.coupled_flight:
            raise ValueError('per-drone profiles require coupled 3D flight')
        if type(self.aircraft_radius) not in (int, float) or not np.isfinite(self.aircraft_radius) or self.aircraft_radius < 0:
            raise ValueError('aircraft_radius must be finite and nonnegative')
        positive = [self.horizon, self.dt, self.target_radius, self.separation]
        nonnegative = [self.activation_time, self.position_error, self.clock_error,
                       self.min_extra_loiter, self.max_extra_loiter, self.detour]
        if not np.all(np.isfinite(positive + nonnegative)):
            raise ValueError("configuration must be finite")
        if min(positive) <= 0 or min(nonnegative) < 0:
            raise ValueError("invalid configuration bounds")
        if self.min_extra_loiter > self.max_extra_loiter:
            raise ValueError("departure delay minimum must not exceed maximum")
        if self.activation_time >= self.horizon or self.dt > self.horizon:
            raise ValueError("activation and dt must precede the horizon")
        if not np.isclose(self.horizon / self.dt, round(self.horizon / self.dt)):
            raise ValueError("horizon must be an integer multiple of dt")
        if any(not isinstance(v, int) or v < 1 for v in
               (self.candidates, self.attempts, self.max_search_nodes)) or self.seed < 0:
            raise ValueError("invalid search budgets/seed")
        if np.shape(self.target) != (2,) or not np.all(np.isfinite(self.target)):
            raise ValueError("target must be a finite XY pair")
        if self.position_error >= self.target_radius:
            raise ValueError("position_error must be smaller than target_radius")
        if not np.isfinite(self.approach_spread_deg) or not 0 <= self.approach_spread_deg <= 360:
            raise ValueError("approach_spread_deg must be between 0 and 360 degrees")
        if self.approach_assignment not in ("geometric", "id"):
            raise ValueError("approach_assignment must be geometric or id")
        if self.approach_routing not in ('pursuit', 'tangent'):
            raise ValueError('approach_routing must be pursuit or tangent')
        if type(self.planning_workers) is not int or self.planning_workers < 0:
            raise ValueError("planning_workers must be a nonnegative integer (0 = auto)")
        if type(self.rollout_check_interval) is not int or self.rollout_check_interval < 1:
            raise ValueError("rollout_check_interval must be a positive integer")
        if self.planning_strategy not in ("fixed", "adaptive") or self.search_order not in ("id", "mrv"):
            raise ValueError("invalid planning strategy or search order")
        if type(self.initial_candidates) is not int or self.initial_candidates < 1:
            raise ValueError("initial_candidates must be a positive integer")
        if not isinstance(self.obstacles, tuple) or any(not isinstance(o, Obstacle) for o in self.obstacles):
            raise ValueError("obstacles must be a tuple of validated Obstacle objects")
        if not np.isfinite(self.obstacle_clearance) or self.obstacle_clearance < 0:
            raise ValueError("obstacle_clearance must be finite and nonnegative")
        if self.selection_policy not in ("first", "max_margin", "efficient"):
            raise ValueError("selection_policy must be first, max_margin or efficient")
        if (type(self.efficiency_slack_cap_m) not in (int,float) or not np.isfinite(self.efficiency_slack_cap_m)
                or self.efficiency_slack_cap_m<0):
            raise ValueError('efficiency_slack_cap_m must be finite and nonnegative')
        if type(self.margin_search_nodes) is not int or self.margin_search_nodes < 1:
            raise ValueError("margin_search_nodes must be a positive integer")
        if type(self.margin_expansion_rounds) is not int or self.margin_expansion_rounds < 0:
            raise ValueError("margin_expansion_rounds must be a nonnegative integer")
        if not np.isfinite(self.post_arrival_time) or self.post_arrival_time < 0:
            raise ValueError("post_arrival_time must be finite and nonnegative")
        if self.arrival_deadline is not None and (not np.isfinite(self.arrival_deadline)
                                                  or not 0 < self.arrival_deadline <= self.horizon):
            raise ValueError("arrival_deadline must be positive and no later than horizon")
        if self.obstacle_routing not in ("lateral", "spatial"):
            raise ValueError("obstacle_routing must be lateral or spatial")
        if self.conflict_guidance not in ("none", "witness"):
            raise ValueError("conflict_guidance must be none or witness")
        if self.candidate_guidance not in ("none", "rejections"):
            raise ValueError("candidate_guidance must be none or rejections")
        if self.candidate_guidance != "none" and not self.coupled_flight:
            raise ValueError("candidate guidance requires coupled 3D flight")
        if self.generator_mode not in ("geometric", "physical_recovery_seed", "physical_mission"):
            raise ValueError("generator_mode must be geometric, physical_mission or physical_recovery_seed")
        if self.numerical_backend not in ("python", "cpp"):
            raise ValueError("numerical_backend must be python or cpp")
        if self.numerical_backend == "cpp" and (not self.coupled_flight or self.generator_mode == "geometric"):
            raise ValueError("cpp numerical_backend requires the coupled 3D physical generator")
        if self.generator_mode != "geometric" and not self.coupled_flight:
            raise ValueError("physical generator_mode requires coupled 3D flight")
        if self.generator_mode != "geometric" and self.passage_guidance != "none":
            raise ValueError("physical generator_mode does not support geometric passage-window fitting")
        if self.reservation_policy not in ("none", "grid"):
            raise ValueError("reservation_policy must be none or grid")
        if not np.isfinite(self.reservation_cell_size) or self.reservation_cell_size < 5:
            raise ValueError("reservation_cell_size must be finite and at least 5 m")
        if type(self.reservation_max_entries) is not int or self.reservation_max_entries < 1:
            raise ValueError("reservation_max_entries must be a positive integer")
        if self.interval_guidance not in ("none", "safe_windows"):
            raise ValueError("interval_guidance must be none or safe_windows")
        if self.interval_guidance != "none" and self.reservation_policy != "grid":
            raise ValueError("interval guidance requires grid reservations")
        if type(self.interval_max_constraints) is not int or self.interval_max_constraints < 1:
            raise ValueError("interval_max_constraints must be a positive integer")
        if self.profile_policy not in ("strict", "adaptive_mixed"):
            raise ValueError("profile_policy must be strict or adaptive_mixed")
        if self.profile_policy == "adaptive_mixed" and (not self.coupled_flight or self.planning_strategy != "adaptive"):
            raise ValueError("adaptive_mixed profiles require adaptive coupled 3D planning")
        if type(self.cruise_levels) is not int or not 1 <= self.cruise_levels <= 3:
            raise ValueError("cruise_levels must be between 1 and 3")
        if self.cruise_levels > 1 and (self.dimensions != 3 or self.cruise_altitude_range is None):
            raise ValueError("multiple cruise levels require a 3D automatic cruise range")
        if not np.isfinite(self.cruise_level_hold) or self.cruise_level_hold < 2:
            raise ValueError("cruise_level_hold must be finite and at least 2 seconds")
        if self.speed_profile not in ("constant", "smooth"):
            raise ValueError("speed_profile must be constant or smooth")
        if self.speed_profile != "constant" and not self.coupled_flight:
            raise ValueError("speed profiles require coupled point-mass 3D flight")
        if self.speed_profile_range is not None:
            bounds = self.speed_profile_range
            envelopes = [p.limits for _, p in self.aircraft_profiles] or [self.limits]
            if (np.shape(bounds) != (2,) or not np.all(np.isfinite(bounds))
                    or any(not lim.min_speed <= bounds[0] <= bounds[1] <= lim.max_speed for lim in envelopes)):
                raise ValueError("speed_profile_range must lie within physical airspeed limits")
        if self.speed_profile != "constant" and (self.speed_profile_range is None or
                                                  self.speed_profile_range[0] == self.speed_profile_range[1]):
            raise ValueError("smooth speed profiles require a nonzero speed_profile_range")
        if self.passage_guidance not in ("none", "safe_windows"):
            raise ValueError("passage_guidance must be none or safe_windows")
        if self.passage_guidance != "none" and (not self.coupled_flight or self.reservation_policy != "grid"
                                                or self.speed_profile != "smooth"):
            raise ValueError("passage guidance requires coupled 3D, grid reservations and smooth speed permission")
        if not np.isfinite(self.speed_profile_acceleration) or self.speed_profile_acceleration <= 0:
            raise ValueError("speed_profile_acceleration must be finite and positive")
        if self.dimensions not in (2, 3):
            raise ValueError("dimensions must be 2 or 3")
        if self.flight_model not in ("point_mass", "legacy"):
            raise ValueError("flight_model must be point_mass or legacy")
        if not isinstance(self.flight, FlightLimits) or not isinstance(self.limits, Limits):
            raise ValueError("flight and limits must be validated parameter objects")
        if not np.all(np.isfinite([self.min_altitude, self.max_altitude,
                                   self.max_climb_rate, self.max_vertical_acceleration])):
            raise ValueError("vertical limits must be finite")
        if not 0 <= self.min_altitude < self.max_altitude or min(self.max_climb_rate, self.max_vertical_acceleration) <= 0:
            raise ValueError("invalid altitude/climb limits")
        if self.target_altitude is not None and (self.dimensions != 3 or
                not self.min_altitude <= self.target_altitude <= self.max_altitude):
            raise ValueError("target altitude must be within the 3D altitude limits")
        if self.cruise_altitude_range is not None:
            bounds = self.cruise_altitude_range
            if (self.dimensions != 3 or np.shape(bounds) != (2,) or not np.all(np.isfinite(bounds))
                    or not self.min_altitude <= bounds[0] <= bounds[1] <= self.max_altitude):
                raise ValueError("Cruise range must respect the altitude limits and min ≤ max (3D mode).")
        if not np.isfinite(self.descent_angle_deg) or not 0 <= self.descent_angle_deg < 90:
            raise ValueError("Descent angle must be at least 0° and less than 90°.")
        if self.descent_angle_deg > 0 and (self.dimensions != 3 or self.target_altitude is None):
            raise ValueError("A descent angle requires a common XYZ destination in 3D.")
        if self.max_descent_rate is not None and (not np.isfinite(self.max_descent_rate) or self.max_descent_rate <= 0):
            raise ValueError("Maximum descent rate must be finite and positive.")


@dataclass(frozen=True)
class Trajectory:
    drone_id: int
    candidate_id: int
    speed: float
    dt: float
    states: np.ndarray
    accelerations: np.ndarray
    arrival_time: float
    release_time: float
    approach_angle: float | None = None  # Outward bearing of the assigned entry side (radians).
    vertical_states: np.ndarray | None = None  # [z, vertical_speed] on the common grid.
    vertical_accelerations: np.ndarray | None = None
    cruise_altitude: float | None = None  # Selected level; may differ between candidates.
    flight_states: np.ndarray | None = None  # [x,y,z,V,heading,gamma,bank]
    flight_controls: np.ndarray | None = None  # [thrust/m,extra_drag/m,load,roll_rate]
    flight_parameters: FlightLimits | None = None
    airspeed_limit: float | None = None
    routing_mode: str = "lateral"
    cruise_levels_m: tuple[float, ...] = ()
    speed_reference: np.ndarray | None = None
    profile_family: str | None = None  # Only explicit mixed-policy plans carry this declaration.
    aircraft_profile: object | None = None  # Own immutable profile only, never a fleet map.
    generation_phases: tuple = ()  # Descriptive own-plan (name, first_step, last_step) intervals.

    def __post_init__(self):
        # Isolate plans from caller-owned buffers and make them read-only.
        object.__setattr__(self, "cruise_levels_m", tuple(self.cruise_levels_m))
        object.__setattr__(self, "generation_phases", tuple(tuple(p) for p in self.generation_phases))
        for name in ("states", "accelerations", "vertical_states", "vertical_accelerations", "flight_states", "flight_controls", "speed_reference"):
            if getattr(self, name) is None:
                continue
            array = np.array(getattr(self, name), dtype=float, copy=True)
            array.setflags(write=False)
            object.__setattr__(self, name, array)

    def __reduce__(self):
        # NumPy's default pickle restores writable arrays. Reconstruct through
        # the constructor so IPC cannot remove the plan's read-only contract.
        return type(self), tuple(getattr(self, item.name) for item in fields(self))

    @property
    def horizon(self):
        return len(self.accelerations) * self.dt

    @property
    def positions(self):
        return (self.states[:, :2] if self.vertical_states is None else
                np.column_stack((self.states[:, :2], self.vertical_states[:, 0])))

    @property
    def speed_bound(self):
        if self.flight_states is not None:
            return self.airspeed_limit
        return (self.speed if self.vertical_states is None else
                float(np.hypot(self.speed, np.max(np.abs(self.vertical_states[:, 1])))))

    def sample(self, time):
        """Continuous interpolation; pre/post horizon use constant endpoint turn rate.

        Endpoint extension supports bounded clock offsets without stopping or
        teleporting an aircraft. Only [0, horizon] is jointly certified.
        """
        if self.flight_states is not None:
            from .flight3d import project
            full = self.sample_flight(time)
            index = np.clip(np.floor(np.asarray(time) / self.dt + 1e-9).astype(int), 0, len(self.flight_controls) - 1)
            return project(full, self.flight_controls[index], self.flight_parameters)
        t = np.asarray(time, dtype=float)
        index = np.floor(np.maximum(t, 0) / self.dt).astype(int)
        index = np.clip(index, 0, len(self.accelerations) - 1)
        base = self.states[index].copy()
        tau = t - index * self.dt
        alpha = self.accelerations[index].copy()
        before, after = t < 0, t >= self.horizon
        base = np.where(before[..., None], self.states[0], base)
        base = np.where(after[..., None], self.states[-1], base)
        tau = np.where(before, t, np.where(after, t - self.horizon, tau))
        alpha = np.where(before | after, 0.0, alpha)
        horizontal = propagate(base, self.speed, alpha, tau)
        if self.vertical_states is None:
            return horizontal
        vertical = self.vertical_states[index].copy()
        vertical = np.where(before[..., None], self.vertical_states[0], vertical)
        vertical = np.where(after[..., None], self.vertical_states[-1], vertical)
        az = np.where(before | after, 0.0, self.vertical_accelerations[index])
        z = vertical[..., 0] + vertical[..., 1] * tau + 0.5 * az * tau**2
        vz = vertical[..., 1] + az * tau
        return np.concatenate((horizontal, np.stack((z, vz), axis=-1)), axis=-1)

    def sample_flight(self, time):
        """Reintegrate the held physical controls; preserve full 3D state."""
        from .flight3d import propagate as fly, trim_control
        if self.flight_states is None:
            raise ValueError("This plan has no coupled flight state")
        t = np.asarray(time, dtype=float)
        index = np.clip(np.floor(np.maximum(t, 0) / self.dt + 1e-9).astype(int), 0, len(self.flight_controls) - 1)
        state = self.flight_states[index].copy()
        controls = self.flight_controls[index].copy()
        before, after = t < 0, t >= self.horizon
        state = np.where(before[..., None], self.flight_states[0], state)
        state = np.where(after[..., None], self.flight_states[-1], state)
        controls = np.where(before[..., None], trim_control(self.flight_states[0], self.flight_parameters), controls)
        controls = np.where(after[..., None], trim_control(self.flight_states[-1], self.flight_parameters), controls)
        tau = np.where(before, t, np.where(after, t - self.horizon, t - index * self.dt))
        return fly(state, controls, tau, self.flight_parameters)

    def to_dict(self):
        profiles = {}
        if self.generation_phases:
            profiles['generation_phases'] = [list(p) for p in self.generation_phases]
        if self.aircraft_profile is not None:
            profiles['aircraft_profile'] = self.aircraft_profile.record()
        if self.cruise_levels_m:
            profiles["cruise_levels_m"] = list(self.cruise_levels_m)
        if self.profile_family is not None:
            profiles["profile_family"] = self.profile_family
        if self.speed_reference is not None:
            profiles["speed_reference_m_s"] = self.speed_reference.tolist()
        return {**profiles, "drone_id": self.drone_id, "candidate_id": self.candidate_id,
                "speed": self.speed, "dt": self.dt, "arrival_time": self.arrival_time,
                "release_time": self.release_time, "states": self.states.tolist(),
                "accelerations": self.accelerations.tolist(), "approach_angle_rad": self.approach_angle,
                "vertical_states": None if self.vertical_states is None else self.vertical_states.tolist(),
                "vertical_accelerations": None if self.vertical_accelerations is None else self.vertical_accelerations.tolist(),
                "cruise_altitude_m": self.cruise_altitude,
                "flight_model": "point_mass" if self.flight_states is not None else "legacy",
                "flight_states": None if self.flight_states is None else self.flight_states.tolist(),
                "flight_controls": None if self.flight_controls is None else self.flight_controls.tolist(),
                "flight_state_columns": ["x_m", "y_m", "z_m", "airspeed_m_s", "air_heading_rad", "flight_path_rad", "bank_rad"],
                "flight_control_columns": ["thrust_m_s2", "extra_drag_m_s2", "load_factor", "roll_rate_rad_s"],
                "flight_parameters": None if self.flight_parameters is None else asdict(self.flight_parameters),
                "airspeed_limit_m_s": self.airspeed_limit, "routing_mode": self.routing_mode}


@dataclass(frozen=True)
class Result:
    status: str
    reason: str
    plans: tuple[Trajectory, ...] = ()
    minimum_distance_lower_bound: float | None = None
    required_distance: float | None = None
    counts: tuple[int, ...] = ()
    search_nodes: int = 0
    diagnostics: dict = field(default_factory=dict)
    timings: dict = field(default_factory=dict, compare=False, repr=False)

    @property
    def validated(self):
        return self.status == "validated"


def continuous_distance_lower_bound(first, second):
    """Lipschitz certificate between grid instants, NOT just a sampled minimum.

    Spatial speeds are bounded by V1/V2, so distance is (V1+V2)-Lipschitz.
    Every instant lies at most dt/2 from a sampled endpoint. Assumes common grid.
    """
    if first.dt != second.dt or first.states.shape != second.states.shape:
        raise ValueError("certification requires a common time grid")
    if first.positions.shape != second.positions.shape:
        raise ValueError("cannot mix 2D and 3D trajectories")
    sampled = np.linalg.norm(first.positions - second.positions, axis=1)
    return float(sampled.min() - (first.speed_bound + second.speed_bound) * first.dt / 2)


def required_distance(first, second, config):
    return (config.separation + config.for_drone(first.drone_id).aircraft_radius
            + config.for_drone(second.drone_id).aircraft_radius) + 2 * config.position_error + (
        first.speed_bound + second.speed_bound) * config.clock_error


def approach_angles(snapshots, config):
    """Deterministic native sector assignment; a proposal, never a safety proof."""
    from .native_planning import approach_angles as native_angles
    return native_angles(snapshots, config)


def approach_is_valid(plan, config):
    """Check the first nominal contact with the destination disk, not just gates."""
    if plan.approach_angle is None:
        return config.approach_spread_deg == 0
    positions = plan.states[:, :2] - config.target
    contacts = np.flatnonzero(np.linalg.norm(positions, axis=1) <= config.target_radius)
    if not len(contacts):
        return False
    index = contacts[0]
    angle = np.arctan2(positions[index, 1], positions[index, 0])
    heading = plan.states[index, 2]
    tolerance = np.deg2rad(15)
    return bool(index * plan.dt >= plan.release_time
                and abs(wrap(angle - plan.approach_angle)) <= tolerance
                and abs(wrap(heading - plan.approach_angle - np.pi)) <= tolerance)


def _candidate(snapshot, config, candidate_id, rng, approach_angle=None, diagnostics=None,
               repair_hint=None, repair_mode=None, departure_override=None,
               level_hold_override=None, routing_extra_margin=0.):
    """Compatibility adapter to the native proposal API; validate before selection."""
    from .native_planning import propose_candidate
    return propose_candidate(snapshot, config, candidate_id, rng, approach_angle, diagnostics,
        repair_hint, repair_mode, departure_override, level_hold_override, routing_extra_margin)


def _validate_individual(plan, snapshot, config, diagnostics=None, *, backend='python'):
    """Validate the stored primitives independently of candidate construction."""
    if config.coupled_flight:
        if backend == 'cpp':
            from .native_backend import validate_candidate
            return validate_candidate(plan, snapshot, config, diagnostics)
        from .coupled_planning import validate_coupled
        return validate_coupled(plan, snapshot, config, diagnostics)
    if plan.speed_reference is not None or plan.cruise_levels_m:
        return False
    if plan.flight_states is not None:
        return False
    if not np.all(np.isfinite(plan.states)) or not np.all(np.isfinite(plan.accelerations)):
        return False
    if not config.limits.min_speed <= plan.speed <= config.limits.max_speed:
        return False
    if not (config.activation_time + config.min_extra_loiter - 1e-9 <= plan.release_time
            <= config.activation_time + config.max_extra_loiter + 1e-9):
        return False
    if not np.allclose(plan.states[0], snapshot.state, atol=1e-10, rtol=0):
        return False
    if np.max(np.abs(plan.states[:, 3])) > config.limits.max_rate(plan.speed) + 1e-9:
        return False
    if np.max(np.abs(plan.accelerations)) > config.limits.max_acceleration(plan.speed) + 1e-9:
        return False
    propagated = propagate(plan.states[:-1], plan.speed, plan.accelerations, plan.dt)
    if not np.allclose(propagated, plan.states[1:], rtol=0, atol=1e-8):
        return False
    holding = np.arange(len(plan.accelerations)) * plan.dt < plan.release_time
    if np.any(plan.accelerations[holding] != 0):
        return False
    if config.dimensions == 3:
        if plan.vertical_states is None or plan.vertical_accelerations is None:
            return False
        z = plan.vertical_states
        az = plan.vertical_accelerations
        if z.shape != (len(plan.states), 2) or az.shape != plan.accelerations.shape:
            return False
        if not np.all(np.isfinite(z)) or not np.all(np.isfinite(az)):
            return False
        if not np.allclose(z[0], [snapshot.altitude, 0], atol=1e-9, rtol=0):
            return False
        if np.max(np.abs(az)) > config.max_vertical_acceleration + 1e-9 or np.any(az[holding] != 0):
            return False
        if z[:, 1].max() > config.max_climb_rate + 1e-9 or z[:, 1].min() < -config.descent_rate_limit - 1e-9:
            return False
        expected = np.column_stack((z[:-1, 0] + z[:-1, 1] * plan.dt + 0.5 * az * plan.dt**2,
                                    z[:-1, 1] + az * plan.dt))
        if not np.allclose(expected, z[1:], atol=1e-8, rtol=0):
            return False
        if config.cruise_altitude_range is not None:
            low, high = config.cruise_altitude_range
            if plan.cruise_altitude is None or not low <= plan.cruise_altitude <= high:
                return False
            cruise = plan.cruise_altitude
        else:
            cruise = snapshot.altitude if snapshot.cruise_altitude is None else snapshot.cruise_altitude
            if plan.cruise_altitude is not None and not np.isclose(plan.cruise_altitude, cruise, atol=1e-8):
                return False
        if not np.allclose(z[-1], [cruise, 0], atol=1e-8, rtol=0):
            return False
        flight = (np.arange(len(z)) * plan.dt >= plan.release_time) & (
            np.arange(len(z)) * plan.dt <= plan.arrival_time + 1e-9)
        if not np.any(flight & (np.abs(z[:, 0] - cruise) <= 1e-8) & (np.abs(z[:, 1]) <= 1e-8)):
            return False
        # Check internal stationary points too; altitude extrema may lie between samples.
        tau = np.divide(-z[:-1, 1], az, out=np.zeros_like(az), where=np.abs(az) > 1e-12)
        tau = np.clip(tau, 0, plan.dt)
        interior = z[:-1, 0] + z[:-1, 1] * tau + 0.5 * az * tau**2
        if min(z[:, 0].min(), interior.min()) < config.min_altitude - 1e-8 or max(z[:, 0].max(), interior.max()) > config.max_altitude + 1e-8:
            return False
        if config.target_altitude is not None:
            arrival_state = plan.sample(plan.arrival_time)
            if np.linalg.norm(np.r_[arrival_state[:2] - config.target,
                                    arrival_state[4] - config.target_altitude]) > config.target_radius - config.position_error + 1e-8:
                return False
            if config.descent_angle_deg > 0:
                angle = np.rad2deg(np.arctan2(-arrival_state[5], plan.speed))
                if abs(angle - config.descent_angle_deg) > 1e-6:
                    return False
                contacts = np.flatnonzero(np.linalg.norm(plan.positions - np.r_[config.target, config.target_altitude], axis=1)
                                          <= config.target_radius)
                if not len(contacts):
                    return False
                first_angle = np.rad2deg(np.arctan2(-z[contacts[0], 1], plan.speed))
                if abs(first_angle - config.descent_angle_deg) > 0.5:
                    return False
    elif plan.vertical_states is not None:
        return False
    return bool(np.linalg.norm(plan.sample(plan.arrival_time)[:2] - config.target)
                <= config.target_radius - config.position_error + 1e-8
                and approach_is_valid(plan, config))


def validate_individual(plan, snapshot, config, diagnostics=None, *, backend=None):
    """Independent audit replay; Python here is a validator, never a planner."""
    backend = 'python' if backend is None else backend
    if backend not in ('python', 'cpp'):
        raise ValueError('validation backend must be python or cpp')
    if backend == 'cpp' and not config.coupled_flight:
        raise ValueError('C++ validation requires coupled 3D flight')
    from .catalog import invalid
    from .profile_policy import validation_config
    if plan.drone_id != snapshot.drone_id:
        return invalid(diagnostics, 'validation_drone_id')
    assigned = config.profile_for(plan.drone_id)
    if assigned is not None and plan.aircraft_profile != assigned:
        return invalid(diagnostics, 'validation_aircraft_profile')
    config = config.for_drone(plan.drone_id)
    if plan.aircraft_profile is not None:
        try:
            plan.aircraft_profile.require_matches(config)
        except (AttributeError, ValueError):
            return invalid(diagnostics, 'validation_aircraft_profile')
    try:
        config = validation_config(plan, config)
    except ValueError:
        return invalid(diagnostics, "validation_profile_family")
    if (not np.isfinite(plan.arrival_time) or plan.arrival_time < plan.release_time
            or plan.arrival_time > config.latest_nominal_arrival + 1e-9):
        detail = ("arrival_deadline_exceeded" if config.arrival_deadline is not None
                  and plan.arrival_time + config.clock_error > config.arrival_deadline
                  else "post_arrival_window_exceeds_horizon")
        return invalid(diagnostics, "arrival_timing", detail)
    if (plan.dt != config.dt or len(plan.states) != round(config.horizon / config.dt) + 1
            or len(plan.accelerations) != round(config.horizon / config.dt)):
        return invalid(diagnostics, "plan_horizon")
    if not _validate_individual(plan, snapshot, config, diagnostics, backend=backend):
        return False
    if config.obstacles and backend != 'cpp':
        from .obstacles import plan_clearance, nominal_clearance_required
        if plan.dt != config.dt or len(plan.states) != round(config.horizon / config.dt) + 1:
            return invalid(diagnostics, "obstacle_plan_horizon")
        lower = plan_clearance(plan, config)
        if not np.isfinite(lower) or lower < nominal_clearance_required(plan.speed_bound, config) + 1e-7:
            return invalid(diagnostics, "obstacle_clearance")
    return True


def _generate_catalog(task):
    """Independent per-ID PRNG stream; top-level callable for spawn workers."""
    from .catalog import Catalog, extend_catalog
    snapshot, config, approach_angle = task
    return extend_catalog((snapshot, config, approach_angle, Catalog(), config.candidates)).plans


def _plan_mission(snapshots: tuple[Snapshot, ...], config: Config):
    """Spawn orchestration only; proposals, catalogs and search execute in C++."""
    from .production import require_production
    require_production(config)
    if not snapshots:
        raise ValueError("at least one snapshot is required")
    if len({s.drone_id for s in snapshots}) != len(snapshots):
        raise ValueError("drone IDs must be unique")
    snapshots = tuple(sorted(snapshots, key=lambda s: s.drone_id))
    config.require_aircraft_ids(s.drone_id for s in snapshots)
    # Validate shared requested speed ranges against every individual envelope
    # before starting workers; never clip a mission request to fit a profile.
    for s in snapshots:
        config.for_drone(s.drone_id)
    if config.cruise_levels > 1 and not config.coupled_flight:
        return Result("unsupported_profile_model", "Multiple cruise levels currently require coupled 3D flight.")
    for s in snapshots:
        local = config.for_drone(s.drone_id)
        if s.drone_id < 0 or np.shape(s.state) != (4,) or not np.all(np.isfinite(s.state)):
            raise ValueError("invalid snapshot")
        if not local.limits.min_speed <= s.speed <= local.limits.max_speed:
            return Result("invalid_initial_state", "Initial airspeed violates the model limits.")
        if config.speed_profile != "constant" and not config.speed_profile_range[0] <= s.speed <= config.speed_profile_range[1]:
            return Result("invalid_initial_state", "Initial airspeed must lie within the requested speed profile range.")
        if abs(s.state[3]) > local.limits.max_rate(s.speed):
            return Result("invalid_initial_state", "Initial turn rate violates the model limits.")
        if config.dimensions == 3:
            altitudes = [s.altitude]
            if config.cruise_altitude_range is None:
                altitudes.append(s.altitude if s.cruise_altitude is None else s.cruise_altitude)
            if not all(np.isfinite(a) and config.min_altitude <= a <= config.max_altitude for a in altitudes):
                return Result("invalid_initial_state", "Initial/cruise altitude violates the altitude limits.")
            error = descent_error(s, local)
            if error is not None:
                return Result("descent_not_feasible", error)
            if config.coupled_flight:
                from .flight3d import from_snapshot, trim_control, envelope_ok
                initial = from_snapshot(s)
                if not envelope_ok(initial[None], trim_control(initial, local.flight)[None], local):
                    return Result("invalid_initial_state", "Initial loiter violates the coupled flight/stall/propulsion envelope.")
    from .obstacles import obstacle_preflight
    obstacle_error = obstacle_preflight(snapshots, config)
    if obstacle_error:
        return Result(*obstacle_error)
    for a, b in combinations(snapshots, 2):
        distance = np.linalg.norm(a.position3d - b.position3d) if config.dimensions == 3 else np.linalg.norm(np.subtract(a.state[:2], b.state[:2]))
        required_centers = config.separation + config.for_drone(a.drone_id).aircraft_radius + config.for_drone(b.drone_id).aircraft_radius
        if distance < required_centers:
            return Result("initial_separation_violation", "Separation is already violated at t=0.")
        speed_sum = (config.for_drone(a.drone_id).limits.max_speed + config.for_drone(b.drone_id).limits.max_speed if config.coupled_flight else
                     np.hypot(a.speed, config.vertical_speed_limit) + np.hypot(b.speed, config.vertical_speed_limit)
                     if config.dimensions == 3 else a.speed + b.speed)
        robust = required_centers + 2 * config.position_error + speed_sum * config.clock_error
        if distance < robust:
            return Result("initial_margin_not_certified", "Initial separation cannot satisfy the chosen tube criterion.")

    for s in snapshots:
        maximum_speed = config.for_drone(s.drone_id).limits.max_speed if config.coupled_flight else s.speed
        displacement = np.linalg.norm(np.r_[np.subtract(s.state[:2], config.target),
                         s.altitude - config.target_altitude] if config.coupled_flight and config.target_altitude is not None
                         else np.subtract(s.state[:2], config.target))
        if displacement > maximum_speed * config.horizon + config.target_radius:
            return Result("destination_unreachable_within_horizon",
                          "Even a straight path at the model's allowed speed bound cannot reach the target before the horizon.")

    from .catalog import Catalog, extend_catalog
    from .parallel import generate_catalogs
    from .native_planning import NativeJointSearch as JointSearch

    assignments = approach_angles(snapshots, config)
    catalogs = [Catalog() for _ in snapshots]
    search = JointSearch(config)
    rounds = []
    generation_s = search_s = 0.0
    indices = list(range(len(snapshots)))
    goal = config.initial_candidates if config.planning_strategy == "adaptive" else config.candidates
    while True:
        start = perf_counter()
        hints = search.repair_hints([snapshots[i].drone_id for i in indices])
        temporal = search.interval_hints(catalogs, indices)
        passages = search.passage_hints(catalogs, indices)
        tasks = [(snapshots[i], config, assignments[snapshots[i].drone_id], catalogs[i],
                  min(config.candidates, goal if not rounds else len(catalogs[i].plans) + config.initial_candidates),
                  hints.get(snapshots[i].drone_id), temporal.get(snapshots[i].drone_id),
                  "conflict" if rounds else "initial", passages.get(snapshots[i].drone_id))
                 for i in indices]
        for i, catalog in zip(indices, generate_catalogs(extend_catalog, tasks, config.planning_workers)):
            catalogs[i] = catalog
        generation_s += perf_counter() - start
        counts = tuple(len(c.plans) for c in catalogs)
        start = perf_counter()
        plans = search.solve([c.plans for c in catalogs])
        search_s += perf_counter() - start
        if plans is None and search.nodes >= config.max_search_nodes:
            search.exhausted = True  # Do not generate another batch with no search budget left.
        rounds.append({"expanded_drone_ids": [snapshots[i].drone_id for i in indices],
                       "counts": counts, "attempts": [c.attempted for c in catalogs],
                       "search_nodes": search.nodes, "solved": plans is not None})
        if plans is not None or search.exhausted or config.planning_strategy == "fixed" or not all(counts):
            break
        indices = search.expansion_indices(catalogs, snapshots)
        if not indices:
            break

    optimization_s = 0.0
    selection = {"policy": config.selection_policy}
    if plans is not None and config.selection_policy in ("max_margin", "efficient"):
        initial_plans = plans
        initial_score = search.score(plans)
        steps = []
        expanded = []
        for quality_round in range(config.margin_expansion_rounds + 1):
            start = perf_counter()
            plans, quality = search.refine([c.plans for c in catalogs], plans)
            elapsed = perf_counter() - start
            search_s += elapsed
            optimization_s += elapsed
            steps.append({"expanded_drone_ids": expanded, "counts": tuple(len(c.plans) for c in catalogs),
                          "attempts": [c.attempted for c in catalogs], **quality})
            if (quality_round == config.margin_expansion_rounds or not quality["catalog_optimal"]
                    or not search.optimization_available() or len(plans) < 2):
                break
            indices = search.margin_expansion_indices(catalogs, snapshots, plans)
            if not indices:
                break
            start = perf_counter()
            hints = search.repair_hints([snapshots[i].drone_id for i in indices], plans)
            temporal = search.interval_hints(catalogs, indices, plans)
            passages = search.passage_hints(catalogs, indices, plans)
            tasks = [(snapshots[i], config, assignments[snapshots[i].drone_id], catalogs[i],
                      min(config.candidates, len(catalogs[i].plans) + config.initial_candidates),
                      hints.get(snapshots[i].drone_id), temporal.get(snapshots[i].drone_id), "margin",
                      passages.get(snapshots[i].drone_id)) for i in indices]
            for i, catalog in zip(indices, generate_catalogs(extend_catalog, tasks, config.planning_workers)):
                catalogs[i] = catalog
            generation_s += perf_counter() - start
            expanded = [snapshots[i].drone_id for i in indices]
        counts = tuple(len(c.plans) for c in catalogs)
        final_score = search.score(plans)
        selection.update({"objective": "leximin_pairwise_robust_slack", "steps": steps,
                          "initial_sorted_slack_m": list(initial_score), "selected_sorted_slack_m": list(final_score),
                          "improved": final_score > initial_score,
                          "initial_candidate_ids": [p.candidate_id for p in initial_plans],
                          "selected_candidate_ids": [p.candidate_id for p in plans],
                          "initial_pairs": search.pair_scores(initial_plans),
                          "selected_pairs": search.pair_scores(plans),
                          "expansion_rounds": len(steps) - 1,
                          "optimization_nodes": search.optimization_nodes,
                          "optimization_budget_exhausted": search.optimization_budget_exhausted,
                          "catalog_optimal": steps[-1]["catalog_optimal"]})
        if config.selection_policy=='efficient':
            initial_efficiency=search.efficiency_score(initial_plans)
            selected_efficiency=search.efficiency_score(plans)
            selection.update(objective='capped_slack_then_duration_distance',
                initial_efficiency_score=list(initial_efficiency),selected_efficiency_score=list(selected_efficiency),
                improved=selected_efficiency>initial_efficiency,slack_cap_m=config.efficiency_slack_cap_m)

    diagnostic = {"strategy": config.planning_strategy, "search_order": config.search_order,
                  "conflict_guidance": config.conflict_guidance,
                  "interval_guidance": config.interval_guidance,
                  "profile_policy": config.profile_policy,
                  "passage_guidance": config.passage_guidance,
                  "candidate_guidance": config.candidate_guidance,
                  "rounds": rounds, "search": search.diagnostics(), "selection": selection,
                  "catalogs": [{"drone_id": s.drone_id, "attempted": c.attempted,
                                "accepted": len(c.plans), "rejections": dict(sorted(c.rejections.items())),
                                "rejection_details": dict(sorted(c.rejection_details.items())),
                                "guided_proposals": c.guided_proposals,
                                "interval_contexts": c.interval_contexts,
                                "interval_proposals": c.interval_proposals,
                                "profile_events": c.profile_events,
                                "profile_proposals": c.profile_proposals,
                                "passage_contexts": c.passage_contexts,
                                "passage_proposals": c.passage_proposals,
                                "candidate_proposals": c.candidate_proposals,
                                "accepted_routing_modes": {mode: sum(p.routing_mode == mode for p in c.plans)
                                                           for mode in sorted({p.routing_mode for p in c.plans})},
                                "attempt_budget_exhausted": c.attempted == config.attempts,
                                "candidate_limit_reached": len(c.plans) == config.candidates}
                               for s, c in zip(snapshots, catalogs)]}
    from .mission_timing import timing_diagnostics
    diagnostic["timing"] = timing_diagnostics(config, catalogs, snapshots, plans or ())
    timings = {"catalog_wall_s": generation_s, "search_wall_s": search_s,
               "optimization_wall_s": optimization_s,
               "per_drone": [{"drone_id": s.drone_id, "generation_s": c.generation_s,
                              "validation_s": c.validation_s} for s, c in zip(snapshots, catalogs)]}
    if not all(counts):
        return Result("no_candidate", "No complete candidate for at least one drone; this is not an impossibility proof.",
                      counts=counts, diagnostics=diagnostic, timings=timings)
    if plans is None:
        status = "search_budget_exhausted" if search.exhausted else "catalog_exhausted"
        return Result(status, "No validated combination found; no claim about continuous-space infeasibility.",
                      counts=counts, search_nodes=search.nodes, diagnostics=diagnostic, timings=timings)
    # Re-evaluate the chosen pairs, independently of the cached search decisions.
    start = perf_counter()
    verified = search.verify_joint(plans)
    if 'reservations' in verified:
        diagnostic['reservations'] = verified['reservations']
    diagnostic['obstacle_checks'] = verified['obstacle_checks']
    timings['joint_verification_wall_s'] = perf_counter() - start
    return Result('validated',
        'Numerically validated coupled point-mass flight on [0, horizon]; separation bound is conditional on integration accuracy and tracking tubes.',
        plans, verified['minimum_distance_lower_bound'], verified['required_distance'],
        counts, search.nodes, diagnostic, timings)


def plan_mission(snapshots: tuple[Snapshot, ...], config: Config = Config(dimensions=3)):
    """Native-only production planning from a frozen snapshot; no Python fallback."""
    from .production import require_production
    from .native_planning import require_planner, plan_mission as native_plan
    from .parallel import worker_count
    from .reservations import ReservationBudgetError
    require_production(config)
    require_planner()
    start = perf_counter()
    try:
        result = (native_plan(snapshots, config)
                  if worker_count(config.planning_workers, len(snapshots)) == 1
                  else _plan_mission(snapshots, config))
    except ReservationBudgetError as error:
        result = Result('reservation_budget_exhausted', str(error))
    return replace(result, timings={**result.timings, 'total_wall_s': perf_counter() - start})


def mission_document(result, config, snapshots, *, aircraft_profile=None):
    config.require_aircraft_ids(s.drone_id for s in snapshots)
    document = {"schema": "swarm-flight3d-v3" if config.coupled_flight else "swarm-trajectory-3d-v2" if config.dimensions == 3 else "swarm-trajectory2d-v1", "prng": "NumPy PCG64 / SeedSequence",
                "numpy_version": np.__version__, "config": asdict(config),
                "snapshots": [asdict(s) for s in snapshots], "status": result.status,
                "reason": result.reason, "counts": result.counts,
                "search_nodes": result.search_nodes,
                "planning_diagnostics": result.diagnostics,
                "minimum_distance_lower_bound": result.minimum_distance_lower_bound,
                "required_distance": result.required_distance,
                "plans": [p.to_dict() for p in result.plans]}
    if config.numerical_backend == 'cpp':
        from .native_backend import identity
        document['numerical_backend_identity'] = identity()
    if aircraft_profile is not None:
        from .aircraft_profiles import AircraftProfile
        if not isinstance(aircraft_profile, AircraftProfile):
            raise ValueError('aircraft_profile must be a validated AircraftProfile')
        aircraft_profile.require_matches(config)
        document['aircraft_profile'] = aircraft_profile.record()
    if config.aircraft_profiles:
        if aircraft_profile is not None:
            raise ValueError('use common or per-drone aircraft profiles, not both')
        document['aircraft_profiles'] = [{'drone_id': i, **p.record()} for i, p in sorted(config.aircraft_profiles)]
        for plan in result.plans:
            if plan.aircraft_profile != config.profile_for(plan.drone_id):
                raise ValueError('plan aircraft profile differs from mission assignment')
    import json
    canonical = json.dumps(document, sort_keys=True, allow_nan=False).encode()
    document["sha256"] = hashlib.sha256(canonical).hexdigest()
    return document
