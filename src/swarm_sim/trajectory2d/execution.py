"""Local flight execution and a separate, omniscient evaluation observer.

The observer can inspect all positions. An Executor owns only its own plan,
local state and clock; it never receives another aircraft's telemetry.
"""

from dataclasses import dataclass
from types import SimpleNamespace
from itertools import combinations

import numpy as np
from scipy.integrate import trapezoid

from .dynamics import Limits, wrap
from .planning import Config, Trajectory


class Executor:
    def __init__(self, plan: Trajectory, limits: Limits, *, execution_backend="cpp"):
        if execution_backend != 'cpp':
            raise ValueError('The Python execution backend has been retired; production requires C++.')
        if plan.flight_states is None:
            raise ValueError('cpp execution requires coupled 3D plans')
        self.execution_backend = execution_backend
        self.plan = plan
        self.limits = limits
        # The actual aircraft starts at its frozen snapshot. A clock offset
        # shifts its reference, never its physical initial position.
        from .flight3d import project
        self.flight_state = plan.flight_states[0].copy()
        # Display/observer projection only; controllers consume flight_state.
        self.state = project(self.flight_state, plan.flight_controls[0], plan.flight_parameters)
        self.wind_estimator = SimpleNamespace(previous=None, value=np.zeros(3))
        self.filter_interventions = 0
        self.filter_unresolved = 0
        self.first_unresolved_local_time = None
        self.last_flight_control = None
        self._reference = None
        self._reference_plan = None
        self._reference_config = None
        self.last_local_time = None
        self.control_updates = 0

    def flight_command(self, local_time, local_dt, config, measured_state=None, constrained=True, control_dt=None):
        """Feedforward plus local error feedback; no fleet or ground telemetry."""
        if not constrained:
            raise ValueError('Production execution requires the constrained C++ controller')
        from .native_backend import create_reference, reference_window
        if self._reference is None:
            self._reference = create_reference(self.plan, config)
            self._reference_plan, self._reference_config = self.plan, config
        elif self._reference_plan is not self.plan or self._reference_config is not config:
            raise ValueError('own reference/configuration changed during execution; revalidate and create a new Executor')
        reference, base = reference_window(self._reference, local_time, local_dt)
        self.control_updates += 1
        measured = self.flight_state if measured_state is None else measured_state
        measurement_dt = local_dt if self.last_local_time is None else local_time - self.last_local_time
        self.last_local_time = float(local_time)
        hold_dt = config.dt if control_dt is None else control_dt
        from .native_backend import local_control_update
        previous = self.wind_estimator.previous
        control, estimate, intervened, feasible = local_control_update(
            measured, reference, base, config, measurement_dt, hold_dt,
            estimated_wind=self.wind_estimator.value,
            previous_measured=previous if self.last_flight_control is not None else None,
            previous_control=self.last_flight_control if previous is not None else None)
        self.wind_estimator.previous = measured.copy()
        self.wind_estimator.value[:] = estimate
        self.filter_interventions += int(intervened)
        self.filter_unresolved += int(not feasible)
        if not feasible and self.first_unresolved_local_time is None:
            self.first_unresolved_local_time = float(local_time)
        self.last_flight_control = control.copy()
        return control


def positions(states):
    """Extract spatial coordinates from [x,y,heading,rate,(z,vz)]."""
    return states[..., [0, 1, 4]] if states.shape[-1] == 6 else states[..., :2]


def _coupled_interval(executor, sensor, config, t, offset, rate, disturbance,
                      elapsed, constrained):
    """Split actuator holds at this drone's own local plan-control boundaries.

    The simulator maps local-clock callbacks to integration time. No other
    drone's clock/state, true wind or ground-station correction is passed to
    the controller. Samples remain on the unchanged observer grid.
    """
    from .flight3d import project
    from .native_backend import propagate_samples
    states = np.empty((len(elapsed), 7))
    controls = np.empty((len(elapsed), 4))
    boundaries = []
    initial_control = None
    cursor = 0.
    while cursor < config.dt - 1e-10:
        local_time = rate * (t + cursor) + offset
        end = config.dt
        if constrained and local_time < executor.plan.horizon - 1e-10:
            next_local = (0. if local_time < -1e-10 else
                          (np.floor(local_time / executor.plan.dt + 1e-8) + 1) * executor.plan.dt)
            end = min(end, cursor + (next_local - local_time) / rate)
        duration = end - cursor
        if duration <= 1e-10:
            raise RuntimeError('non-progressing local control schedule')
        measured = executor.flight_state.copy()
        measured[:3] += sensor.error()
        control = executor.flight_command(local_time, rate * duration, config, measured,
                                          constrained, control_dt=duration)
        if initial_control is None:
            initial_control = control.copy()
        before = executor.flight_state.copy()
        indices = np.flatnonzero((elapsed > cursor + 1e-10) & (elapsed <= end + 1e-10))
        durations = np.r_[elapsed[indices] - cursor, duration]
        segment = propagate_samples(before, control, durations, config, disturbance=disturbance, substeps=4)
        states[indices], controls[indices] = segment[:-1], control
        # At an exactly sampled switch, the displayed control is outgoing.
        matching = np.flatnonzero(np.abs(elapsed - cursor) <= 1e-10)
        controls[matching] = control
        executor.flight_state = segment[-1].copy()
        boundaries.extend(((t + cursor, before, control.copy()),
                           (t + end, executor.flight_state.copy(), control.copy())))
        cursor = end
    executor.state = project(executor.flight_state, controls[-1], config.flight)
    return states, controls, initial_control, boundaries


@dataclass(frozen=True)
class Evaluation:
    times: np.ndarray
    actual: np.ndarray
    nominal: np.ndarray
    distances: np.ndarray
    tracking_errors: np.ndarray
    report: dict
    flight_actual: np.ndarray | None = None
    flight_nominal: np.ndarray | None = None


def check_execution_backend(config, controller='constrained', backend='cpp'):
    from .production import require_production
    require_production(config)
    if backend != 'cpp':
        raise ValueError('The Python execution backend has been retired; production requires C++.')
    if controller != 'constrained':
        raise ValueError('Production C++ execution requires the constrained controller; legacy control is retired.')
    from .native_backend import require_execution_native
    require_execution_native()


def simulate(plans: tuple[Trajectory, ...], config: Config, clock_offsets=None,
             clock_drifts=None, wind=0.0, observer_substeps=10, *,
             controller="constrained", wind_model="rotating", wind_heading_deg=0.,
             position_biases=None, position_noise=0., disturbance_seed=0, execution_backend="cpp"):
    """Offline experiment with independent controllers; wind is a speed bound.

    Wind vectors are deterministic, held constant per nominal mission step and
    bounded in norm by `wind`. Clock error is offset + drift * true_time.
    Monitoring uses a refined grid and Lipschitz bounds to cover between samples.
    The frozen snapshot is not refreshed after planning.
    Position bias/noise affects only local measurements, not physical state or
    the omniscient observer. Other state measurements remain ideal. Constrained
    execution additionally schedules updates at its own local plan boundaries.
    """
    if not plans:
        raise ValueError("cannot execute an empty mission")
    count = len(plans)
    offsets = np.zeros(count) if clock_offsets is None else np.asarray(clock_offsets, dtype=float)
    drifts = np.zeros(count) if clock_drifts is None else np.asarray(clock_drifts, dtype=float)
    if offsets.shape != (count,) or drifts.shape != (count,):
        raise ValueError("one clock offset and drift are required per drone")
    if not np.all(np.isfinite(offsets)) or not np.all(np.isfinite(drifts)) or np.any(drifts <= -1):
        raise ValueError("invalid clocks")
    if not np.isfinite(wind) or wind < 0 or not isinstance(observer_substeps, int) or observer_substeps < 1:
        raise ValueError("invalid wind/substeps")
    check_execution_backend(config, controller, execution_backend)
    if wind_model not in ("rotating", "constant", "gust") or not np.isfinite(wind_heading_deg):
        raise ValueError("invalid wind model/heading")
    if not np.isfinite(position_noise) or position_noise < 0:
        raise ValueError("position_noise must be finite and nonnegative")
    if type(disturbance_seed) is not int or disturbance_seed < 0:
        raise ValueError("disturbance_seed must be a nonnegative integer")
    dimensions = config.dimensions
    biases = np.zeros((count, dimensions)) if position_biases is None else np.asarray(position_biases, dtype=float)
    if biases.shape != (count, dimensions) or not np.all(np.isfinite(biases)):
        raise ValueError("position_biases must contain one finite spatial vector per drone")
    if any(p.dt != config.dt or not np.isclose(p.horizon, config.horizon) for p in plans):
        raise ValueError("plans must use the configured grid")
    if any((p.vertical_states is not None) != (config.dimensions == 3) for p in plans):
        raise ValueError("plans must match the configured dimensions")
    if any((p.flight_states is not None) != config.coupled_flight for p in plans):
        raise ValueError("plans must match the configured flight model")

    if len({p.drone_id for p in plans}) != len(plans):
        raise ValueError('duplicate execution drone IDs')
    config.require_aircraft_ids(p.drone_id for p in plans)
    local_configs = [config.for_drone(p.drone_id) for p in plans]
    for p, local in zip(plans, local_configs):
        assigned = config.profile_for(p.drone_id)
        if assigned is not None and p.aircraft_profile != assigned:
            raise ValueError('execution aircraft profile differs from the validated plan')
        if config.coupled_flight and (p.flight_parameters != local.flight or p.airspeed_limit != local.limits.max_speed):
            raise ValueError('execution physical configuration differs from the validated plan')
        if p.aircraft_profile is not None:
            p.aircraft_profile.require_matches(local)
    executors = [Executor(p, local.limits, execution_backend=execution_backend) for p, local in zip(plans, local_configs)]
    from .perturbations import PositionSensor, wind_at
    sensors = [PositionSensor(p.drone_id, dimensions, b, position_noise, disturbance_seed) for p, b in zip(plans, biases)]
    steps = len(plans[0].accelerations)
    sample_dt = config.dt / observer_substeps
    times = np.arange(steps * observer_substeps + 1) * sample_dt
    actual = np.empty((len(times), count, 6 if config.dimensions == 3 else 4))
    actual[0] = [e.state for e in executors]
    flight_actual = np.empty((len(times), count, 7)) if config.coupled_flight else None
    flight_controls = np.empty((len(times), count, 4)) if config.coupled_flight else None
    if config.coupled_flight:
        flight_actual[0] = [e.flight_state for e in executors]
    control_boundaries = [[] for _ in plans]
    vertical_min = float(actual[0, :, 4].min()) if config.dimensions == 3 else 0.0
    vertical_max = float(actual[0, :, 4].max()) if config.dimensions == 3 else 0.0
    for step in range(steps):
        t = step * config.dt
        for k, executor in enumerate(executors):
            local = local_configs[k]
            # Independent perturbation; not information supplied to the controller.
            disturbance = wind_at(t, executor.plan.drone_id, wind, wind_model, wind_heading_deg)
            elapsed = np.arange(1, observer_substeps + 1) * sample_dt
            if config.coupled_flight:
                from .flight3d import project
                segment, control, initial_control, boundaries = _coupled_interval(
                    executor, sensors[k], local, t, offsets[k], 1 + drifts[k], disturbance,
                    elapsed, controller == 'constrained')
                control_boundaries[k].extend(boundaries)
                first = step * observer_substeps + 1
                flight_actual[first:first + observer_substeps, k] = segment
                flight_controls[first - 1, k] = initial_control
                flight_controls[first:first + observer_substeps, k] = control
                actual[first:first + observer_substeps, k] = project(segment, control, local.flight)
                vertical_min = min(vertical_min, float(segment[:, 2].min()))
                vertical_max = max(vertical_max, float(segment[:, 2].max()))
                continue

    # Omniscient observer starts here. Nothing below feeds the local controllers.
    nominal = np.stack([p.sample(times) for p in plans], axis=1)
    shifted = np.stack([p.sample((1 + drifts[k]) * times + offsets[k])
                        for k, p in enumerate(plans)], axis=1)
    spatial = positions(actual)
    errors = np.linalg.norm(spatial - positions(shifted), axis=2)
    pairs = list(combinations(range(count), 2))
    distances = (np.min(np.stack([np.linalg.norm(spatial[:, i] - spatial[:, j], axis=1)
                                 for i, j in pairs]), axis=0) if pairs else np.full(len(times), np.inf))
    execution_bounds = [local_configs[k].limits.max_speed if config.coupled_flight else
                        float(np.hypot(p.speed, config.vertical_speed_limit)) if config.dimensions == 3
                        else p.speed for k, p in enumerate(plans)]
    radii = [c.aircraft_radius for c in local_configs]
    distance_lower = min((float(np.linalg.norm(spatial[:, i] - spatial[:, j], axis=1).min())
                          - (execution_bounds[i] + execution_bounds[j] + 2 * wind) * sample_dt / 2
                          for i, j in pairs), default=None)
    surface_distance_lower = min((float(np.linalg.norm(spatial[:, i] - spatial[:, j], axis=1).min())
                                 - radii[i] - radii[j]
                                 - (execution_bounds[i] + execution_bounds[j] + 2 * wind) * sample_dt / 2
                                 for i, j in pairs), default=None)
    error_upper = [float(errors[:, k].max() +
                         (execution_bounds[k] + wind + p.speed_bound * abs(1 + drifts[k])) * sample_dt / 2)
                   for k, p in enumerate(plans)]
    clock_max = float(np.max(np.maximum(np.abs(offsets), np.abs(offsets + drifts * config.horizon))))
    arrivals = []
    contact_bearings = []
    approach_errors = []
    approaches_verified = True
    descent_angles = []
    descent_verified = True
    contacts = []
    nominal_contacts = []
    unresolved_contacts = []
    if config.coupled_flight:
        from .contact import first_entry, recorded_sampler, plan_contact, ContactResolutionError
        center = np.r_[config.target, config.target_altitude] if config.target_altitude is not None else config.target
        for k, plan in enumerate(plans):
            try:
                nominal_contacts.append(plan_contact(plan, config))
            except ContactResolutionError:
                nominal_contacts.append(None)
                unresolved_contacts.append(plan.drone_id)
            sampler = recorded_sampler(control_boundaries[k], local_configs[k].flight,
                lambda t, drone_id=plan.drone_id: wind_at(
                    np.floor((t + 1e-9) / config.dt) * config.dt, drone_id, wind, wind_model, wind_heading_deg))
            try:
                contacts.append(first_entry(times, flight_actual[:, k], sampler, center, config.target_radius,
                                            execution_bounds[k] + wind, start_time=config.activation_time))
            except ContactResolutionError:
                contacts.append(None)
                if plan.drone_id not in unresolved_contacts:
                    unresolved_contacts.append(plan.drone_id)
    target_distance = np.linalg.norm(actual[:, :, :2] - config.target, axis=2)
    if config.dimensions == 3 and config.target_altitude is not None:
        target_distance = np.hypot(target_distance, actual[:, :, 4] - config.target_altitude)
    for k in range(count):
        hits = np.flatnonzero((times >= config.activation_time) &
                             (target_distance[:, k] <= config.target_radius))
        event = contacts[k] if config.coupled_flight else None
        reached = event is not None if config.coupled_flight else bool(len(hits))
        arrivals.append(event.time if event is not None else float(times[hits[0]]) if reached else None)
        if reached:
            contact_state = event.state if event is not None else None
            if config.dimensions == 3:
                index = int(np.clip(np.searchsorted(times, arrivals[-1]), 0, len(times) - 1))
                before, after = max(0, index - 1), min(len(times) - 1, index + 1)
                horizontal_speed = np.linalg.norm(actual[after, k, :2] - actual[before, k, :2]) / (times[after] - times[before])
                angle = (float(-np.rad2deg(contact_state[5])) if config.coupled_flight else
                         float(np.rad2deg(np.arctan2(-actual[index, k, 5], horizontal_speed))))
                descent_angles.append(angle)
                if config.descent_angle_deg > 0:
                    descent_verified &= abs(angle - config.descent_angle_deg) <= 1.0
            else:
                descent_angles.append(None)
            delta = (contact_state[:2] if event is not None else actual[hits[0], k, :2]) - config.target
            bearing = float(np.arctan2(delta[1], delta[0]))
            contact_bearings.append(float(np.rad2deg(bearing)))
            assigned = plans[k].approach_angle
            error = float(abs(wrap(bearing - assigned))) if assigned is not None else None
            approach_errors.append(float(np.rad2deg(error)) if error is not None else None)
            if assigned is not None:
                heading = contact_state[4] if event is not None else actual[hits[0], k, 2]
                heading_error = abs(wrap(heading - assigned - np.pi))
                approaches_verified &= bool(error <= np.deg2rad(15) and heading_error <= np.deg2rad(15))
        else:
            descent_angles.append(None)
            if config.descent_angle_deg > 0:
                descent_verified = False
            contact_bearings.append(None)
            approach_errors.append(None)
            if plans[k].approach_angle is not None:
                approaches_verified = False
    report = {
        "execution_backend": execution_backend,
        "execution_model": ("coupled coordinated point mass; own measured-position feedback; bounded horizontal velocity disturbances"
                            if config.coupled_flight else "local measured-position feedback; constant horizontal speed; bounded horizontal synthetic wind"),
        "measurement_model": "bounded position bias/noise; other state measurements ideal; observer uses true simulated state",
        "flight_model": "point_mass" if config.coupled_flight else "legacy",
        "controller": controller if config.coupled_flight else "legacy_planar",
        "wind_model": wind_model, "wind_heading_deg": wind_heading_deg,
        "position_biases_m": biases.tolist(), "position_noise_bound_m": position_noise,
        "disturbance_seed": disturbance_seed,
        "position_measurement_error_bound_m": (np.linalg.norm(biases, axis=1) + position_noise).tolist(),
        "controller_diagnostics": [{"drone_id": p.drone_id,
                                    "safety_filter_interventions": e.filter_interventions,
                                    "safety_filter_unresolved": e.filter_unresolved,
                                    "control_updates": e.control_updates,
                                    "first_unresolved_local_time_s": e.first_unresolved_local_time,
                                    "final_estimated_wind_m_s": e.wind_estimator.value.tolist()}
                                   for p, e in zip(plans, executors)],
        "dimensions": config.dimensions,
        "arrival_mode": "common_xyz" if config.target_altitude is not None else "horizontal_destination",
        "target_altitude_m": config.target_altitude,
        "initial_altitudes_m": [float(p.vertical_states[0, 0]) for p in plans] if config.dimensions == 3 else None,
        "cruise_altitude_range_m": config.cruise_altitude_range,
        "selected_cruise_altitudes_m": [p.cruise_altitude for p in plans],
        "selected_cruise_levels_m": [list(p.cruise_levels_m) for p in plans],
        "speed_profile": config.speed_profile,
        "profile_policy": config.profile_policy,
        "passage_guidance": config.passage_guidance,
        "selected_profile_families": [p.profile_family for p in plans],
        "speed_profile_reference_range_m_s": config.speed_profile_range,
        "requested_descent_angle_deg": config.descent_angle_deg,
        "max_climb_rate_m_s": None if config.aircraft_profiles else config.max_climb_rate,
        "max_descent_rate_m_s": None if config.aircraft_profiles else config.descent_rate_limit,
        "nominal_descent_angles_deg": [float(-np.rad2deg(p.sample_flight(p.arrival_time)[5])) if config.coupled_flight else
                                       float(np.rad2deg(np.arctan2(-p.sample(p.arrival_time)[5], p.speed)))
                                       if config.dimensions == 3 else None for p in plans],
        "actual_descent_angles_deg": descent_angles,
        "nominal_contact_times_s": [c.time if c is not None else None for c in nominal_contacts],
        "nominal_contact_descent_angles_deg": [float(-np.rad2deg(c.state[5])) if c is not None else None
                                               for c in nominal_contacts],
        "contact_detection": "reintegrated first-entry event, 1e-6 s resolution" if config.coupled_flight else "sampled",
        "contact_bracket_width_s": [c.bracket_width_s if c is not None else None for c in contacts],
        "contact_resolution_failures": unresolved_contacts,
        "contact_events_verified": not unresolved_contacts if config.coupled_flight else True,
        "descent_angle_verified_in_execution": bool(descent_verified),
        "vertical_limits_verified": bool(config.dimensions == 2 or (
            vertical_min >= config.min_altitude - 1e-8
            and vertical_max <= config.max_altitude + 1e-8
            and all(actual[:, k, 5].max() <= c.max_climb_rate + 1e-8
                    and actual[:, k, 5].min() >= -c.descent_rate_limit - 1e-8
                    for k, c in enumerate(local_configs)))),
        "clock_offsets_s": offsets.tolist(), "clock_drifts_s_per_s": drifts.tolist(),
        "wind_bound_m_s": wind, "max_clock_error_s": clock_max,
        "max_sampled_tracking_errors_m": errors.max(axis=0).tolist(),
        "tracking_error_upper_bounds_m": error_upper,
        "tube_conditions_verified": bool(max(error_upper) <= config.position_error
                                          and clock_max <= config.clock_error + 1e-12),
        "minimum_sampled_distance_m": float(distances.min()) if pairs else None,
        "minimum_distance_lower_bound_m": distance_lower,
        "aircraft_bounding_radii_m": radii,
        "minimum_surface_distance_lower_bound_m": surface_distance_lower,
        "separation_verified_in_execution_model": surface_distance_lower is None or surface_distance_lower >= config.separation,
        "sampled_separation_violation": any(np.linalg.norm(spatial[:, i] - spatial[:, j], axis=1).min() < config.separation + radii[i] + radii[j] for i, j in pairs),
        "arrival_times_s": arrivals, "all_arrived": all(a is not None for a in arrivals),
        "horizon_s": config.horizon, "arrival_deadline_s": config.arrival_deadline,
        "minimum_post_arrival_s": config.post_arrival_time,
        "arrival_deadline_verified_in_execution": all(a is not None and (
            config.arrival_deadline is None or a <= config.arrival_deadline + 1e-9) for a in arrivals),
        "post_arrival_time_verified_in_execution": all(a is not None and
            a + config.post_arrival_time <= config.horizon + 1e-9 for a in arrivals),
        "routing_modes": [p.routing_mode for p in plans],
        "departure_times_s": [p.release_time for p in plans],
        "departure_delay_interval_s": [config.min_extra_loiter, config.max_extra_loiter],
        "approach_spread_deg": config.approach_spread_deg,
        "approach_assignment": config.approach_assignment,
        "assigned_contact_bearings_deg": [float(np.rad2deg(p.approach_angle))
                                          if p.approach_angle is not None else None for p in plans],
        "actual_contact_bearings_deg": contact_bearings,
        "contact_bearing_errors_deg": approach_errors,
        "approach_sectors_verified_in_execution": bool(approaches_verified),
        "nominal_arrival_times_s": [p.arrival_time for p in plans],
        "nominal_path_lengths_to_arrival_m": [float(trapezoid(
            p.sample_flight(times[times <= p.arrival_time])[:, 3] if config.coupled_flight else
            np.hypot(p.speed, nominal[times <= p.arrival_time, k, 5]), times[times <= p.arrival_time]))
            if config.dimensions == 3 else p.speed * p.arrival_time for k, p in enumerate(plans)],
        "nominal_detour_ratios_including_loiter": [float(p.speed * p.arrival_time / max(
            np.linalg.norm(p.sample(p.arrival_time)[:2] - p.states[0, :2]), 1e-9)) for p in plans],
        "certificate_interval_s": [0, config.horizon],
        "post_arrival_behavior": "Aircraft remain present and fly onward until the common horizon.",
    }
    flight_nominal = None
    safety_events = []
    report["reservation_policy"] = config.reservation_policy
    report["reservation_adherence_verified"] = True
    if config.reservation_policy == "grid":
        from .reservations import occupancy
        for k, plan in enumerate(plans):
            reserved = occupancy(plan, config)
            cells = np.floor(spatial[:, k] / config.reservation_cell_size).astype(int)
            valid = np.array([any(a - 1e-10 <= t <= b + 1e-10 for a, b in reserved.cells.get(tuple(cell), ()))
                              for t, cell in zip(times, cells)])
            if not np.all(valid):
                report["reservation_adherence_verified"] = False
                safety_events.append({"kind": "own_reservation_left", "drone_id": plan.drone_id,
                                      "first_observed_time_s": float(times[np.flatnonzero(~valid)[0]])})
    if config.coupled_flight:
        from .flight3d import envelope_ok, air_velocity
        flight_nominal = np.stack([p.sample_flight(times) for p in plans], axis=1)
        verified = all(envelope_ok(flight_actual[:, k], flight_controls[:, k], c, tolerance=1e-5)
                       for k, c in enumerate(local_configs))
        # Exact hold-boundary records below retain BOTH incoming and outgoing
        # controls, including asynchronous switches between observer samples.
        # The previous sampled control need not be the actual incoming control:
        # another local-clock update may already have happened since that sample.
        boundary_data = []
        for records, local in zip(control_boundaries, local_configs):
            boundary_times = np.array([record[0] for record in records])
            boundary_states = np.array([record[1] for record in records])
            boundary_controls = np.array([record[2] for record in records])
            boundary_data.append((boundary_times, boundary_states, boundary_controls))
            verified &= envelope_ok(boundary_states, boundary_controls, local, tolerance=1e-5)
            boundary_vz = air_velocity(boundary_states)[:, 2]
            report['vertical_limits_verified'] &= bool(
                boundary_states[:, 2].min() >= config.min_altitude - 1e-8
                and boundary_states[:, 2].max() <= config.max_altitude + 1e-8
                and boundary_vz.max() <= local.max_climb_rate + 1e-8
                and boundary_vz.min() >= -local.descent_rate_limit - 1e-8)
        stall_required = np.array([c.flight.stall_speed * c.flight.stall_margin for c in local_configs])[None, :] * np.sqrt(flight_controls[..., 2])
        report.update({
            "physical_limits_verified": bool(verified),
            "airspeed_semantics": "total speed relative to air; ground velocity = air velocity + wind",
            "minimum_airspeed_m_s": flight_actual[..., 3].min(axis=0).tolist(),
            "maximum_airspeed_m_s": flight_actual[..., 3].max(axis=0).tolist(),
            "maximum_bank_deg": np.rad2deg(np.abs(flight_actual[..., 6]).max(axis=0)).tolist(),
            "maximum_load_factor": flight_controls[..., 2].max(axis=0).tolist(),
            "minimum_stall_margin_m_s": (flight_actual[..., 3] - stall_required).min(axis=0).tolist(),
            "maximum_thrust_acceleration_m_s2": flight_controls[..., 0].max(axis=0).tolist(),
            "maximum_airbrake_acceleration_m_s2": flight_controls[..., 1].max(axis=0).tolist(),
            "specific_propulsive_work_j_kg": trapezoid(flight_actual[..., 3] * flight_controls[..., 0], times, axis=0).tolist(),
            "numerical_validation": "RK4: 2 subdivisions/nominal step; 4/observer sample; 4/dense validation sample; no certified ODE error bound",
            "safety_scope": "Separation bounds assume the numerical paths, speed envelope and execution tubes. Not a real-aircraft safety certificate.",
        })
        from .execution_safety import flight_diagnostics
        details, physical_events = flight_diagnostics(flight_actual, flight_controls, times, plans, config, boundary_data)
        report.update(details)
        safety_events.extend(physical_events)
        # A Lipschitz certificate cannot use a violated speed bound.
        speed_bound_valid = bool(np.all(np.isfinite(flight_actual))
                                 and flight_actual[..., 3].min() > 0
                                 and all(flight_actual[:, k, 3].max() <= c.limits.max_speed + 1e-8
                                         for k, c in enumerate(local_configs)))
        report["execution_speed_bound_verified"] = speed_bound_valid
        if not speed_bound_valid:
            report["minimum_distance_lower_bound_m"] = None
            report['minimum_surface_distance_lower_bound_m'] = None
            report["separation_verified_in_execution_model"] = False
            report["tube_conditions_verified"] = False
    else:
        report["physical_limits_verified"] = None
    from .obstacles import execution_clearance
    obstacle_speed_valid = report.get("execution_speed_bound_verified", True)
    if not config.coupled_flight and config.dimensions == 3:
        obstacle_speed_valid = bool(np.all(np.isfinite(actual))
                                    and np.abs(actual[..., 5]).max() <= config.vertical_speed_limit + 1e-8)
    report.update(execution_clearance(spatial, plans, config, sample_dt, wind,
                                     obstacle_speed_valid))
    from .execution_safety import execution_failures, first_bound_event
    for k, plan in enumerate(plans):
        if arrivals[k] is None:
            safety_events.append({'kind': 'destination_not_reached', 'drone_id': plan.drone_id,
                                  'first_observed_time_s': float(config.horizon)})
        elif config.descent_angle_deg > 0 and abs(descent_angles[k] - config.descent_angle_deg) > 1:
            safety_events.append({'kind': 'descent_angle_requirement_failed', 'drone_id': plan.drone_id,
                                  'first_observed_time_s': arrivals[k], 'observed_angle_deg': descent_angles[k],
                                  'requested_angle_deg': config.descent_angle_deg, 'tolerance_deg': 1.})
        if arrivals[k] is not None and config.arrival_deadline is not None and arrivals[k] > config.arrival_deadline + 1e-9:
            safety_events.append({'kind': 'arrival_deadline_exceeded', 'drone_id': plan.drone_id,
                                  'first_observed_time_s': arrivals[k]})
        padding = (execution_bounds[k] + wind + plan.speed_bound * abs(1 + drifts[k])) * sample_dt / 2
        event = first_bound_event('tracking_tube_not_certified', errors[:, k] + padding,
                                  config.position_error, times, drone_id=plan.drone_id)
        if event:
            safety_events.append(event)
        event = first_bound_event('clock_bound_exceeded', np.abs(offsets[k] + drifts[k] * times),
                                  config.clock_error + 1e-12, times, drone_id=plan.drone_id)
        if event:
            safety_events.append(event)
        executor = executors[k]
        if executor.filter_unresolved:
            safety_events.append({'kind': 'control_protection_not_verified', 'drone_id': plan.drone_id,
                                  'first_observed_time_s': float((executor.first_unresolved_local_time - offsets[k]) / (1 + drifts[k]))})
        for j, obstacle in enumerate(config.obstacles):
            event = first_bound_event('obstacle_clearance_not_certified',
                                      config.obstacle_clearance + radii[k] + (execution_bounds[k] + wind) * sample_dt / 2
                                      - obstacle.distance(spatial[:, k]), 0., times,
                                      drone_id=plan.drone_id, obstacle_index=j)
            if event:
                safety_events.append(event)
    for i, j in pairs:
        event = first_bound_event('inter_drone_clearance_not_certified',
                                  config.separation + radii[i] + radii[j] + (execution_bounds[i] + execution_bounds[j] + 2 * wind) * sample_dt / 2
                                  - np.linalg.norm(spatial[:, i] - spatial[:, j], axis=1), 0., times,
                                  drone_ids=[plans[i].drone_id, plans[j].drone_id])
        if event:
            safety_events.append(event)
    report['controller_protection_verified'] = all(e.filter_unresolved == 0 for e in executors)
    if config.speed_schedule == 'operating':
        from .execution_safety import operating_speed_diagnostics
        report.update(operating_speed_diagnostics(flight_actual, times, plans, local_configs, contacts))
    report['control_protection_applicable'] = bool(config.coupled_flight and controller == 'constrained')
    report['safety_events'] = sorted(safety_events, key=lambda event: (event['first_observed_time_s'], event['kind']))
    report['first_safety_event'] = report['safety_events'][0] if safety_events else None
    report['execution_failures'] = execution_failures(report)
    report['execution_validated'] = not report['execution_failures']
    if config.aircraft_profiles:
        report['aircraft_profiles'] = [{'drone_id': p.drone_id, **config.profile_for(p.drone_id).record()} for p in plans]
        report['physical_limits_per_drone'] = [
            {'drone_id': p.drone_id, 'min_airspeed_m_s': c.limits.min_speed, 'max_airspeed_m_s': c.limits.max_speed,
             'max_climb_rate_m_s': c.max_climb_rate, 'max_descent_rate_m_s': c.descent_rate_limit}
            for p, c in zip(plans, local_configs)]
    if execution_backend == 'cpp':
        from .native_backend import identity
        report['execution_backend_identity'] = identity()
    return Evaluation(times, actual, nominal, distances, errors, report, flight_actual, flight_nominal)
