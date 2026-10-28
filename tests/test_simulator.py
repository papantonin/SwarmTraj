import numpy as np

from swarm_sim import SimulatorConfig, SwarmSimulator


def test_physics_backend_returns_all_drone_telemetry() -> None:
    config = SimulatorConfig(num_drones=3, gui=False)
    with SwarmSimulator(config) as simulation:
        initial = simulation.state
        state = simulation.step(initial.positions)

    assert state.count == 3
    assert state.time == simulation.timestep
    assert state.positions.shape == (3, 3)
    assert state.velocities.shape == (3, 3)
    assert state.motor_rpm.shape == (3, 4)
    assert np.all(np.isfinite(state.positions))


def test_two_dimensional_commands_are_accepted() -> None:
    config = SimulatorConfig(num_drones=2, dimensions=2, gui=False)
    with SwarmSimulator(config) as simulation:
        targets_xy = simulation.state.positions[:, :2]
        state = simulation.step(targets_xy)

    np.testing.assert_allclose(state.positions[:, 2], 1.0, atol=1e-3)
