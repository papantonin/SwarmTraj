"""Local-neighbour flocking example driven directly by swarm telemetry."""

from __future__ import annotations

import argparse
import time

import numpy as np

from swarm_sim import SimulatorConfig, SwarmSimulator, flocking_velocity


def run(num_drones: int, duration: float, dimensions: int, gui: bool) -> None:
    rng = np.random.default_rng(7)
    initial = rng.uniform(-0.7, 0.7, size=(num_drones, 3))
    initial[:, 2] = 1.0 if dimensions == 2 else rng.uniform(0.7, 1.3, size=num_drones)
    config = SimulatorConfig(num_drones=num_drones, dimensions=dimensions, gui=gui)

    with SwarmSimulator(config, initial_positions=initial) as simulation:
        started = time.monotonic()
        for _ in range(int(duration / simulation.timestep)):
            commands = flocking_velocity(simulation.state, dimensions=dimensions)
            state = simulation.step_velocity(commands)
            if gui:
                remaining = state.time - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--num-drones", type=int, default=8)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--dimensions", type=int, choices=(2, 3), default=3)
    parser.add_argument("--gui", action=argparse.BooleanOptionalAction, default=True)
    run(**vars(parser.parse_args()))
