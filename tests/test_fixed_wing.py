import numpy as np

from swarm_sim import FixedWingConfig, FixedWingSwarmSimulator


def test_four_aircraft_start_evenly_spaced_on_one_ring() -> None:
    config = FixedWingConfig(num_drones=4, radius=20.0, altitude=10.0, gui=False)
    with FixedWingSwarmSimulator(config) as simulation:
        state = simulation.state

    radii = np.linalg.norm(state.positions[:, :2], axis=1)
    np.testing.assert_allclose(radii, 20.0)
    np.testing.assert_allclose(state.positions[:, 2], 10.0)
    np.testing.assert_allclose(state.pairwise_distances()[0, 2], 40.0)


def test_loiter_controller_keeps_aircraft_on_the_shared_ring() -> None:
    config = FixedWingConfig(num_drones=4, radius=25.0, altitude=12.0, gui=False)
    with FixedWingSwarmSimulator(config) as simulation:
        for _ in range(20 * config.control_frequency_hz):
            state = simulation.step_loiter()

    radii = np.linalg.norm(state.positions[:, :2], axis=1)
    np.testing.assert_allclose(radii, config.radius, atol=0.35)
    np.testing.assert_allclose(state.positions[:, 2], config.altitude, atol=1e-6)
    assert np.all(np.linalg.norm(state.velocities[:, :2], axis=1) > 0)
