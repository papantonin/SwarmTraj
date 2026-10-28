"""Five drones track a rotating polygon; a starting point for custom logic."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import time

import numpy as np

from swarm_sim import SimulatorConfig, SwarmSimulator, regular_polygon


def run(num_drones: int, duration: float, dimensions: int, gui: bool) -> None:
    config = SimulatorConfig(num_drones=num_drones, dimensions=dimensions, gui=gui)
    rows: list[list[float | int]] = []

    with SwarmSimulator(config) as simulation:
        started = time.monotonic()
        steps = int(duration / simulation.timestep)
        for _ in range(steps):
            state = simulation.state

            # Replace this block with your own centralized or distributed algorithm.
            center = np.array([0.4 * np.cos(0.15 * state.time), 0.4 * np.sin(0.15 * state.time), 1.0])
            targets = regular_polygon(num_drones, radius=0.6, center=center, phase=0.25 * state.time)
            if dimensions == 2:
                targets = targets[:, :2]

            state = simulation.step(targets)
            for drone in range(state.count):
                rows.append([state.time, drone, *state.positions[drone], *state.velocities[drone]])

            if gui:
                remaining = state.time - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)

    output = Path("results/telemetry.csv")
    output.parent.mkdir(exist_ok=True)
    with output.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", "drone", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s"])
        writer.writerows(rows)
    print(f"Telemetry written to {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--num-drones", type=int, default=5)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--dimensions", type=int, choices=(2, 3), default=3)
    parser.add_argument("--gui", action=argparse.BooleanOptionalAction, default=True)
    run(**vars(parser.parse_args()))
