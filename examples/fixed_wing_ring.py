"""Visualize fixed-wing drones sharing the same circular orbit."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import time

from swarm_sim import FixedWingConfig, FixedWingSwarmSimulator


def run(
    num_drones: int,
    radius: float,
    altitude: float,
    airspeed: float,
    duration: float,
    clockwise: bool,
    gui: bool,
) -> None:
    config = FixedWingConfig(
        num_drones=num_drones,
        radius=radius,
        altitude=altitude,
        airspeed=airspeed,
        clockwise=clockwise,
        gui=gui,
    )
    output = Path(config.output_directory) / "fixed_wing_telemetry.csv"
    rows: list[list[float | int]] = []

    with FixedWingSwarmSimulator(config) as simulation:
        started = time.monotonic()
        for _ in range(int(duration / simulation.timestep)):
            state = simulation.step_loiter()
            for drone in range(state.count):
                rows.append(
                    [
                        state.time,
                        drone,
                        *state.positions[drone],
                        *state.velocities[drone],
                        *state.orientations_rpy[drone],
                    ]
                )
            if gui:
                remaining = state.time - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)

    output.parent.mkdir(exist_ok=True)
    with output.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s",
                "drone",
                "x_m",
                "y_m",
                "z_m",
                "vx_m_s",
                "vy_m_s",
                "vz_m_s",
                "roll_rad",
                "pitch_rad",
                "yaw_rad",
            ]
        )
        writer.writerows(rows)
    print(f"Telemetry written to {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--num-drones", type=int, default=4)
    parser.add_argument("--radius", type=float, default=25.0)
    parser.add_argument("--altitude", type=float, default=12.0)
    parser.add_argument("--airspeed", type=float, default=12.0)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--clockwise", action="store_true")
    parser.add_argument("--gui", action=argparse.BooleanOptionalAction, default=True)
    run(**vars(parser.parse_args()))
