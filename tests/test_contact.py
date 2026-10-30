"""Event localization must not depend on which observer grid happened to hit."""

import numpy as np
import pytest

from swarm_sim.trajectory2d.contact import first_entry, recorded_sampler, ContactResolutionError


@pytest.mark.parametrize("count", [2, 8, 101])
def test_entry_with_outside_endpoints_is_grid_independent(count):
    sample = lambda t: np.array([-10 + 20 * t, 0., 0.])
    times = np.linspace(0, 1, count)
    event = first_entry(times, np.array([sample(t) for t in times]), sample, [0, 0, 0], 1., 20.)
    assert event.time == pytest.approx(.45, abs=1e-6)
    assert np.linalg.norm(event.state) <= 1
    assert event.bracket_width_s <= 1e-6


def test_no_contact_and_activation_start_are_respected():
    sample = lambda t: np.array([-10 + 20 * t, 0.])
    states = np.array([sample(0), sample(1)])
    assert first_entry([0, 1], states, sample, [0, 5], 1, 20) is None
    assert first_entry([0, 1], states, sample, [0, 0], 1, 20, start_time=.7) is None
    event = first_entry([0, 1], states, sample, [0, 0], 1, 20, start_time=.5)
    assert event.time == .5


def test_unresolved_numerical_grazing_has_bounded_work_not_a_false_contact():
    sample = lambda t: np.array([1 + 1e-10, 0.])
    with pytest.raises(ContactResolutionError):
        first_entry([0, 1], np.array([sample(0), sample(1)]), sample, [0, 0], 1, 20)


def test_observer_sampler_uses_outgoing_control_at_switch():
    from swarm_sim.trajectory2d.flight3d import FlightLimits, trim_control, propagate
    state = np.array([0., 0, 30, 12, 0, 0, 0])
    f = FlightLimits()
    first = trim_control(state, f)
    following = propagate(state, first, .2, f, substeps=4)
    second = first.copy()
    second[0] += .5
    records = [(0., state, first), (.2, following, first), (.2, following, second)]
    sample = recorded_sampler(records, f, lambda t: np.zeros(3))
    np.testing.assert_allclose(sample(.3), propagate(following, second, .1, f, substeps=4), atol=1e-12)


def test_old_seed19_obstacle_candidate_is_rejected_before_selection():
    from dataclasses import replace
    from swarm_sim.trajectory2d import Config, separate_loiter_snapshot
    from swarm_sim.trajectory2d.planning import _candidate, approach_angles, validate_individual
    from swarm_sim.trajectory2d.obstacles import load_obstacles
    from swarm_sim.trajectory2d.contact import plan_contact
    config = Config(seed=19, dimensions=3, generator_mode='geometric', numerical_backend='python', target=(500, 500), target_altitude=30, horizon=180,
                    descent_angle_deg=30, max_descent_rate=10, cruise_altitude_range=(40, 110),
                    approach_spread_deg=180, obstacle_routing='spatial',
                    obstacles=load_obstacles('examples/scenarios/static_obstacles.json'))
    snapshots = tuple(replace(s, altitude=30 + 25*k) for k, s in enumerate(separate_loiter_snapshot()))
    rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([19, 3])))
    plan = _candidate(snapshots[3], config, 0, rng, approach_angles(snapshots, config)[3])
    assert plan is not None
    assert -np.rad2deg(plan_contact(plan, config).state[5]) > 31
    diagnostics = {}
    assert not validate_individual(plan, snapshots[3], config, diagnostics)
    assert diagnostics['rejection'] == 'validation_descent_angle'


def test_unresolved_event_prevents_execution_acceptance_even_if_other_checks_pass():
    from swarm_sim.trajectory2d.execution_safety import execution_failures
    report = dict.fromkeys(['all_arrived', 'tube_conditions_verified', 'separation_verified_in_execution_model',
        'arrival_deadline_verified_in_execution', 'post_arrival_time_verified_in_execution',
        'approach_sectors_verified_in_execution', 'vertical_limits_verified', 'descent_angle_verified_in_execution',
        'obstacle_clearance_verified_in_execution_model', 'controller_protection_verified', 'physical_limits_verified'], True)
    report.update(flight_model='point_mass', contact_events_verified=False)
    assert execution_failures(report) == ['contact_events_verified']
