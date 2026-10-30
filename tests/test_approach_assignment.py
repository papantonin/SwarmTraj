"""Geometric, deterministic, one-to-one allocation of arrival-side slots."""

from dataclasses import replace
from itertools import permutations

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, Snapshot
from swarm_sim.trajectory2d.dynamics import wrap
from swarm_sim.trajectory2d.planning import approach_angles


def scene():
    # IDs intentionally disagree with left-to-right geometry.
    return tuple(Snapshot(i, (x, -200, 0, 0), 12) for i, x in enumerate([90, -90, 30, -30]))


def test_rightmost_drone_receives_rightmost_side_on_half_circle():
    assignments = approach_angles(scene(), Config(dimensions=3, target=(0, 0), approach_spread_deg=180))
    expected = {0: 0, 1: -180, 2: -60, 3: -120}
    for key, degrees in expected.items():
        assert abs(wrap(assignments[key] - np.deg2rad(degrees))) < 1e-12


def test_simple_straight_gate_links_have_fewer_crossings_than_id_allocation():
    snapshots = scene()
    config = Config(dimensions=3, target=(0, 0), approach_spread_deg=180)

    def crossing_count(assignment):
        angles = np.array([assignment[s.drone_id] for s in snapshots])
        gates = 120 * np.column_stack((np.cos(angles), np.sin(angles)))
        starts = np.array([s.state[:2] for s in snapshots])

        def area(a, b, c):
            u, v = b - a, c - a
            return u[0] * v[1] - u[1] * v[0]

        return sum(area(a, b, c) * area(a, b, d) < -1e-9
                   and area(c, d, a) * area(c, d, b) < -1e-9
                   for i, (a, b) in enumerate(zip(starts, gates))
                   for c, d in zip(starts[i + 1:], gates[i + 1:]))

    assert crossing_count(approach_angles(snapshots, replace(config, approach_assignment="id"))) == 2
    assert crossing_count(approach_angles(snapshots, config)) == 0


def test_historical_id_allocation_is_explicit_and_invalid_policy_is_rejected():
    config = Config(dimensions=3, target=(0, 0), approach_spread_deg=180, approach_assignment="id")
    assignments = approach_angles(scene(), config)
    np.testing.assert_allclose([assignments[i] for i in range(4)], np.linspace(-np.pi, 0, 4))
    with pytest.raises(ValueError, match="approach_assignment"):
        replace(config, approach_assignment="unknown")


@pytest.mark.parametrize("spread", [30, 90, 180, 270, 360])
def test_all_slots_are_used_exactly_once_and_input_order_does_not_matter(spread):
    config = Config(dimensions=3, target=(0, 0), approach_spread_deg=spread)
    snapshots = scene()
    assignments = approach_angles(snapshots, config)
    for reordered in permutations(snapshots):
        assert assignments == approach_angles(reordered, config)
    angles = np.array(sorted(assignments.values()))
    assert len(np.unique(angles)) == 4
    gaps = np.diff(np.r_[angles, angles[0] + 2 * np.pi])
    if spread == 360:
        np.testing.assert_allclose(gaps, np.pi / 2)
    else:
        np.testing.assert_allclose(np.sort(gaps), np.sort([np.deg2rad(spread) / 3] * 3 + [2 * np.pi - np.deg2rad(spread)]))


def test_ids_do_not_determine_sides_for_distinct_positions():
    config = Config(dimensions=3, target=(0, 0), approach_spread_deg=180)
    first = scene()
    relabelled = tuple(replace(s, drone_id=label) for s, label in zip(first, [42, 7, 2, 123]))
    original = approach_angles(first, config)
    changed = approach_angles(relabelled, config)
    for before, after in zip(first, relabelled):
        assert changed[after.drone_id] == original[before.drone_id]


@pytest.mark.parametrize("spread", [90, 180, 360])
def test_matching_minimises_global_proxy_distance_against_all_permutations(spread):
    rng = np.random.default_rng(21)
    positions = rng.uniform(-300, 300, (4, 2))
    snapshots = tuple(Snapshot(i, (*xy, 0, 0), speed) for i, (xy, speed) in
                      enumerate(zip(positions, [8, 12, 16, 10])))
    config = Config(dimensions=3, target=(20, -15), approach_spread_deg=spread)
    assigned = approach_angles(snapshots, config)
    angles = np.array([assigned[s.drone_id] for s in snapshots])
    radius = max(120, max(6 * s.speed / config.limits.max_rate(s.speed) for s in snapshots))
    gates = np.array(config.target) + radius * np.column_stack((np.cos(angles), np.sin(angles)))
    cost = np.linalg.norm(positions[:, None] - gates[None, :], axis=2)
    optimum = min(sum(cost[i, j] for i, j in enumerate(p)) for p in permutations(range(4)))
    assert np.trace(cost) == pytest.approx(optimum, abs=1e-10)


def test_assignment_is_translation_and_rotation_equivariant():
    original = scene()
    angle = 2.8  # Crosses the -pi/pi representation seam.
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    translation = np.array([340, -120])
    moved = tuple(replace(s, state=(*(rotation @ s.state[:2] + translation), *s.state[2:])) for s in original)
    before = approach_angles(original, Config(dimensions=3, target=(0, 0), approach_spread_deg=180))
    after = approach_angles(moved, Config(dimensions=3, target=tuple(translation), approach_spread_deg=180))
    for drone_id in before:
        assert abs(wrap(after[drone_id] - before[drone_id] - angle)) < 1e-12


def test_disabled_spread_single_drone_and_coincident_positions_are_deterministic():
    assert approach_angles(scene(), Config(dimensions=3, )) == {i: None for i in range(4)}
    one = approach_angles(scene()[:1], Config(dimensions=3, target=(0, 0), approach_spread_deg=180))
    assert one[0] == pytest.approx(np.arctan2(-200, 90))
    coincident = tuple(replace(s, state=(0, 0, 0, 0)) for s in scene())
    config = Config(dimensions=3, target=(0, 0), approach_spread_deg=360)
    assignments = approach_angles(coincident, config)
    assert assignments == approach_angles(coincident[::-1], config)
    assert len(set(assignments.values())) == 4
