import numpy as np

from swarm_sim.algorithms import flocking_velocity, regular_polygon
from swarm_sim.state import SwarmState


def make_state(positions: np.ndarray, velocities: np.ndarray | None = None) -> SwarmState:
    count = len(positions)
    return SwarmState(
        time=0.0,
        positions=np.asarray(positions, dtype=float),
        velocities=np.zeros((count, 3)) if velocities is None else velocities,
        orientations_rpy=np.zeros((count, 3)),
        angular_velocities=np.zeros((count, 3)),
        motor_rpm=np.zeros((count, 4)),
    )


def test_regular_polygon_is_centered_and_has_requested_radius() -> None:
    points = regular_polygon(6, 2.0, center=(1.0, -2.0, 3.0))
    np.testing.assert_allclose(points.mean(axis=0), [1.0, -2.0, 3.0], atol=1e-12)
    np.testing.assert_allclose(np.linalg.norm(points[:, :2] - [1.0, -2.0], axis=1), 2.0)


def test_adjacency_excludes_self_and_distant_drones() -> None:
    state = make_state(np.array([[0, 0, 1], [0.2, 0, 1], [2, 0, 1]]))
    expected = np.array([[False, True, False], [True, False, False], [False, False, False]])
    np.testing.assert_array_equal(state.adjacency(0.5), expected)


def test_flocking_separates_close_drones() -> None:
    state = make_state(np.array([[-0.05, 0, 1], [0.05, 0, 1]]))
    commands = flocking_velocity(state, cohesion_gain=0.0, alignment_gain=0.0)
    assert commands[0, 0] < 0
    assert commands[1, 0] > 0
