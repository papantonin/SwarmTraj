"""Geometric proposals -> retimed references -> physically integrated candidates.

The 2.5D catalogue is only a source of curves. Its independent XY/Z primitives
are NOT executed. A local flight controller rolls the coupled aircraft out,
and an independent numerical validator rejects untrackable proposals.
"""

from dataclasses import replace

import numpy as np
from scipy.integrate import cumulative_trapezoid

from .dynamics import G
from .catalog import invalid, reject
from .flight3d import (air_velocity, envelope_ok, from_snapshot,
                       guidance, project, propagate)


def convert_candidate(proposal, snapshot, config, diagnostics=None, *, passage=None):
    # dt_real/dt_geometric = |velocity_geometric| / requested TOTAL airspeed.
    fine = np.linspace(0, config.horizon, len(proposal.accelerations) * 4 + 1)
    samples = proposal.sample(fine)
    geometric_speed = np.hypot(proposal.speed, samples[:, 5])
    from .rich_profiles import speed_schedule
    if passage is None:
        schedule = speed_schedule(proposal, config, fine)
    else:
        from .passage_guidance import speed_schedule_for_window
        schedule = speed_schedule_for_window(proposal, config, fine, geometric_speed, passage, diagnostics)
    if schedule is None:
        return reject(diagnostics, "speed_profile_not_feasible" if passage is None else "passage_window_not_reachable")
    scheduled_speed, speed_derivative = schedule
    clock = cumulative_trapezoid(geometric_speed / scheduled_speed, fine, initial=0)
    times = np.arange(len(proposal.states)) * proposal.dt
    tau = np.interp(times, clock, fine)
    reference = proposal.sample(tau)
    norm = np.hypot(proposal.speed, reference[:, 5])
    requested_speed = np.interp(tau, fine, scheduled_speed)
    dilation = requested_speed / norm
    index = np.clip(np.floor(tau / proposal.dt).astype(int), 0, len(proposal.accelerations) - 1)
    yaw_rate = reference[:, 3] * dilation
    gamma_rate = (proposal.vertical_accelerations[index] * proposal.speed**2 / norm**3
                  if config.speed_profile == "constant" else
                  proposal.vertical_accelerations[index] * proposal.speed / norm**2 * dilation)
    gamma = np.arctan2(reference[:, 5], proposal.speed)
    bank = np.arctan2(requested_speed * np.cos(gamma) * yaw_rate,
                      G * np.cos(gamma) + requested_speed * gamma_rate)
    full_reference = np.column_stack((reference[:, :2], reference[:, 4],
                                     requested_speed, reference[:, 2], gamma, bank))
    feedforward = np.column_stack((yaw_rate, gamma_rate))
    speed_ff = np.interp(tau, fine, speed_derivative) * dilation
    states = np.empty((len(times), 7))
    controls = np.empty((len(times) - 1, 4))
    states[0] = from_snapshot(snapshot)
    interval = config.rollout_check_interval
    block_start = 0
    for k in range(len(controls)):
        control = guidance(states[k], full_reference[k], feedforward[k], config, config.dt,
                           speed_feedforward=float(speed_ff[k]))
        following = propagate(states[k], control, config.dt, config.flight)
        if interval == 1 and not envelope_ok(np.stack((states[k], following)), np.broadcast_to(control, (2, 4)), config):
            return reject(diagnostics, "rollout_envelope")
        states[k + 1], controls[k] = following, control
        if interval > 1:
            # Reject immediately before continuing through a singular/invalid
            # state. These are a subset of the SAME original envelope checks.
            if (not np.all(np.isfinite(following)) or following[3] <= 0
                    or following[3] < config.limits.min_speed - 1e-7
                    or following[3] > config.limits.max_speed + 1e-7
                    or abs(following[5]) > np.deg2rad(config.flight.max_flight_path_deg) + 1e-7):
                return reject(diagnostics, "rollout_speed_or_slope")
            if (k + 1) % interval == 0 or k == len(controls) - 1:
                # Check both ends with each segment's own held control, including
                # both sides of switches. Nothing is downsampled or omitted.
                endpoints = np.stack((states[block_start:k + 1], states[block_start + 1:k + 2]), axis=1)
                block_controls = controls[block_start:k + 1, None, :]
                if not envelope_ok(endpoints, block_controls, config):
                    return reject(diagnostics, "rollout_envelope")
                block_start = k + 1
    expected = float(np.interp(proposal.arrival_time, fine, clock))
    if expected > config.latest_nominal_arrival + 1e-9:
        # Retiming preserves geometry but can exceed an otherwise valid geometric
        # deadline. Record it before an expensive independent validation.
        if diagnostics is not None and expected + config.clock_error + config.post_arrival_time > config.horizon:
            diagnostics["required_horizon_s"] = expected + config.clock_error + config.post_arrival_time
        return reject(diagnostics, "retimed_arrival_timing",
                      "arrival_deadline_exceeded" if config.arrival_deadline is not None
                      and expected + config.clock_error > config.arrival_deadline else "post_arrival_window_exceeds_horizon")
    target_distance = np.linalg.norm(states[:, :2] - config.target, axis=1)
    if config.target_altitude is not None:
        target_distance = np.hypot(target_distance, states[:, 2] - config.target_altitude)
    allowed = (times >= proposal.release_time) & (np.abs(times - expected) <= 4)
    if not np.any(allowed):
        return reject(diagnostics, "retimed_arrival_window")
    arrival_index = int(np.argmin(np.where(allowed, target_distance, np.inf)))
    projected = project(states, np.vstack((controls, controls[-1])), config.flight)
    return replace(proposal, states=projected[:, :4], vertical_states=projected[:, 4:],
                   accelerations=np.diff(projected[:, 3]) / config.dt,
                   vertical_accelerations=np.diff(projected[:, 5]) / config.dt,
                   arrival_time=float(times[arrival_index]), flight_states=states,
                   flight_controls=controls, flight_parameters=config.flight,
                   airspeed_limit=config.limits.max_speed,
                   speed_reference=requested_speed if config.speed_profile != "constant" else None)


def validate_coupled(plan, snapshot, config, diagnostics=None):
    """Independent rollout, dense envelope/target tests and immutable metadata."""
    from .planning import approach_is_valid
    s, u = plan.flight_states, plan.flight_controls
    count = round(config.horizon / config.dt)
    if (s is None or u is None or s.shape != (count + 1, 7) or u.shape != (count, 4)
            or plan.states.shape != (count + 1, 4) or plan.accelerations.shape != (count,)
            or plan.vertical_states is None or plan.vertical_states.shape != (count + 1, 2)
            or plan.vertical_accelerations is None or plan.vertical_accelerations.shape != (count,)
            or plan.flight_parameters != config.flight or plan.airspeed_limit != config.limits.max_speed
            or plan.dt != config.dt or plan.speed != snapshot.speed
            or not np.allclose(s[0], from_snapshot(snapshot), rtol=0, atol=1e-9)):
        return invalid(diagnostics, "validation_metadata")
    if not config.activation_time + config.min_extra_loiter - 1e-9 <= plan.release_time <= config.activation_time + config.max_extra_loiter + 1e-9:
        return invalid(diagnostics, "validation_departure_window")
    if not plan.release_time <= plan.arrival_time <= config.latest_nominal_arrival + 1e-9:
        return invalid(diagnostics, "validation_arrival_horizon")
    # Release metadata must describe a real holding segment, not just an ID.
    from .dynamics import propagate as planar
    times = np.arange(count + 1) * plan.dt
    holding = times < plan.release_time
    expected_holding = planar(snapshot.state, snapshot.speed, 0, times[holding])
    if (not np.allclose(s[holding, :2], expected_holding[:, :2], atol=1e-4, rtol=0)
            or np.any(np.abs(s[holding, 2] - snapshot.altitude) > 1e-5)
            or np.any(np.abs(s[holding, 5]) > 1e-5)):
        return invalid(diagnostics, "validation_loiter")
    rolled = propagate(s[:-1], u, plan.dt, config.flight)
    if not np.allclose(rolled, s[1:], rtol=0, atol=1e-9):
        return invalid(diagnostics, "validation_integration")
    projected = project(s, np.vstack((u, u[-1])), config.flight)
    if (not np.allclose(projected[:, :4], plan.states, atol=1e-9, rtol=0)
            or not np.allclose(projected[:, 4:], plan.vertical_states, atol=1e-9, rtol=0)
            or not np.allclose(np.diff(projected[:, 3]) / plan.dt, plan.accelerations, atol=1e-9, rtol=0)
            or not np.allclose(np.diff(projected[:, 5]) / plan.dt, plan.vertical_accelerations, atol=1e-9, rtol=0)):
        return invalid(diagnostics, "validation_projection")
    # Check BOTH sides of every control switch, plus three internal points.
    for fraction in (0, 0.25, 0.5, 0.75, 1):
        interior = propagate(s[:-1], u, fraction * plan.dt, config.flight, substeps=4)
        if not envelope_ok(interior, u, config):
            return invalid(diagnostics, "validation_dense_envelope")
    cruise = plan.cruise_altitude
    if cruise is None:
        return invalid(diagnostics, "validation_cruise_metadata")
    if config.cruise_altitude_range is not None:
        if not config.cruise_altitude_range[0] <= cruise <= config.cruise_altitude_range[1]:
            return invalid(diagnostics, "validation_cruise_range")
    elif abs(cruise - (snapshot.altitude if snapshot.cruise_altitude is None else snapshot.cruise_altitude)) > 1e-8:
        return invalid(diagnostics, "validation_cruise_metadata")
    if abs(s[-1, 2] - cruise) > 0.2 or abs(air_velocity(s[-1])[2]) > 0.1:
        return invalid(diagnostics, "validation_cruise_recovery")
    if config.cruise_levels > 1:
        levels = plan.cruise_levels_m
        if (len(levels) != config.cruise_levels or not np.all(np.isfinite(levels))
                or abs(levels[0] - cruise) > 1e-8 or any(
                    not config.cruise_altitude_range[0] <= z <= config.cruise_altitude_range[1] for z in levels)
                or any(abs(a-b) < 5 - 1e-8 for k,a in enumerate(levels) for b in levels[k+1:])):
            return invalid(diagnostics, "validation_cruise_levels_metadata")
        previous = -1
        for level in levels:
            reached = (times >= plan.release_time) & (times < plan.arrival_time)
            reached &= (np.abs(s[:, 2] - level) <= .5) & (np.abs(air_velocity(s)[:, 2]) <= .2)
            reached[:previous + 1] = False
            # At least one second of consecutively settled grid states.
            run = 0
            for k, ok in enumerate(reached):
                run = run + 1 if ok else 0
                if (run - 1) * plan.dt >= 1 - 1e-9:
                    previous = k
                    break
            else:
                return invalid(diagnostics, "validation_cruise_level_not_reached")
    elif plan.cruise_levels_m:
        return invalid(diagnostics, "validation_unrequested_cruise_levels")
    if config.speed_profile != "constant":
        reference = plan.speed_reference
        low, high = config.speed_profile_range
        if (reference is None or reference.shape != (count + 1,) or not np.all(np.isfinite(reference))
                or np.min(reference) < low - 1e-8 or np.max(reference) > high + 1e-8
                or abs(reference[0] - snapshot.speed) > 1e-8
                or abs(reference[-1] - snapshot.speed) > 1e-8 or np.ptp(reference) < .2 - 1e-8
                or np.ptp(s[:, 3]) < .1
                or np.max(np.abs(np.diff(reference) / plan.dt)) > config.speed_profile_acceleration + 1e-5):
            return invalid(diagnostics, "validation_speed_reference")
    elif plan.speed_reference is not None:
        return invalid(diagnostics, "validation_unrequested_speed_reference")
    before_arrival = (np.arange(len(s)) * plan.dt >= plan.release_time) & (np.arange(len(s)) * plan.dt <= plan.arrival_time)
    if not np.any(before_arrival & (np.abs(s[:, 2] - cruise) <= 0.2) & (np.abs(air_velocity(s)[:, 2]) <= 0.1)):
        return invalid(diagnostics, "validation_cruise_before_arrival")
    arrival = plan.sample_flight(plan.arrival_time)
    delta = arrival[:2] - config.target
    distance = np.linalg.norm(delta) if config.target_altitude is None else np.linalg.norm(np.r_[delta, arrival[2] - config.target_altitude])
    if distance > config.target_radius - config.position_error or not approach_is_valid(plan, config):
        return invalid(diagnostics, "validation_target_or_sector")
    from .contact import plan_contact, ContactResolutionError
    try:
        contact = plan_contact(plan, config)
    except ContactResolutionError:
        return invalid(diagnostics, "validation_contact_event_unresolved")
    if contact is None:
        return invalid(diagnostics, "validation_contact_event")
    if contact.time < plan.release_time - 1e-7:
        return invalid(diagnostics, "validation_contact_before_departure")
    if plan.approach_angle is not None:
        from .dynamics import wrap
        offset = contact.state[:2] - config.target
        if (abs(wrap(np.arctan2(offset[1], offset[0]) - plan.approach_angle)) > np.deg2rad(15)
                or abs(wrap(contact.state[4] - plan.approach_angle - np.pi)) > np.deg2rad(15)):
            return invalid(diagnostics, "validation_contact_sector")
    if diagnostics is not None:
        diagnostics["first_contact_time_s"] = contact.time
    if config.descent_angle_deg > 0:
        if (abs(-np.rad2deg(contact.state[5]) - config.descent_angle_deg) > 1
                or abs(-np.rad2deg(arrival[5]) - config.descent_angle_deg) > 1):
            return invalid(diagnostics, "validation_descent_angle")
    return True
