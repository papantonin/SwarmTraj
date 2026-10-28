import numpy as np
import pytest

from swarm_sim.state import SwarmState


def test_observation_mapping() -> None:
    observation = np.arange(40, dtype=float).reshape(2, 20)
    state = SwarmState.from_aviary_observation(observation, time=1.25)
    np.testing.assert_array_equal(state.positions, observation[:, 0:3])
    np.testing.assert_array_equal(state.orientations_rpy, observation[:, 7:10])
    np.testing.assert_array_equal(state.velocities, observation[:, 10:13])
    np.testing.assert_array_equal(state.angular_velocities, observation[:, 13:16])
    np.testing.assert_array_equal(state.motor_rpm, observation[:, 16:20])
    assert state.time == 1.25


def test_bad_observation_is_rejected() -> None:
    with pytest.raises(ValueError, match="N, 20"):
        SwarmState.from_aviary_observation(np.zeros((3, 19)), time=0.0)
