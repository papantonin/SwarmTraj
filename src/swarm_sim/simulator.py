"""High-level multi-drone API backed by gym-pybullet-drones."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .state import SwarmState


@dataclass(frozen=True, slots=True)
class SimulatorConfig:
    """Configuration of a Crazyflie swarm simulation."""

    num_drones: int = 5
    dimensions: int = 3
    control_frequency_hz: int = 48
    physics_frequency_hz: int = 240
    gui: bool = False
    obstacles: bool = False
    record: bool = False
    output_directory: str = "results"

    def __post_init__(self) -> None:
        if self.num_drones < 1:
            raise ValueError("num_drones must be at least one")
        if self.dimensions not in (2, 3):
            raise ValueError("dimensions must be 2 or 3")
        if self.control_frequency_hz <= 0 or self.physics_frequency_hz <= 0:
            raise ValueError("frequencies must be positive")
        if self.physics_frequency_hz % self.control_frequency_hz:
            raise ValueError("physics_frequency_hz must be divisible by control_frequency_hz")


class SwarmSimulator:
    """Simulate and control several quadrotors through a small stable API.

    The backend simulates full 6-DoF dynamics in both modes. In 2D mode, the
    high-level command keeps each vehicle at its initial altitude and accepts
    either ``(N, 2)`` or ``(N, 3)`` targets.
    """

    def __init__(
        self,
        config: SimulatorConfig = SimulatorConfig(),
        initial_positions: ArrayLike | None = None,
    ) -> None:
        self.config = config
        self._imports = self._load_backend()
        self._initial_positions = self._prepare_initial_positions(initial_positions)
        self._fixed_altitudes = self._initial_positions[:, 2].copy()
        Path(config.output_directory).mkdir(parents=True, exist_ok=True)

        self._env = self._imports["CtrlAviary"](
            drone_model=self._imports["DroneModel"].CF2X,
            num_drones=config.num_drones,
            initial_xyzs=self._initial_positions,
            physics=self._imports["Physics"].PYB,
            pyb_freq=config.physics_frequency_hz,
            ctrl_freq=config.control_frequency_hz,
            gui=config.gui,
            record=config.record,
            obstacles=config.obstacles,
            user_debug_gui=False,
            output_folder=config.output_directory,
        )
        self._controllers = [
            self._imports["DSLPIDControl"](drone_model=self._imports["DroneModel"].CF2X)
            for _ in range(config.num_drones)
        ]
        self._observation, _ = self._env.reset()
        self._step_index = 0

    @staticmethod
    def _load_backend() -> dict[str, Any]:
        try:
            from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
            from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
            from gym_pybullet_drones.utils.enums import DroneModel, Physics
        except ImportError as error:
            raise RuntimeError(
                "simulation dependencies are missing; run ./scripts/setup.sh then "
                "source .venv/bin/activate"
            ) from error
        return {
            "CtrlAviary": CtrlAviary,
            "DSLPIDControl": DSLPIDControl,
            "DroneModel": DroneModel,
            "Physics": Physics,
        }

    def _prepare_initial_positions(self, positions: ArrayLike | None) -> NDArray[np.float64]:
        if positions is None:
            angles = np.arange(self.config.num_drones) * 2 * np.pi / self.config.num_drones
            result = np.column_stack((0.5 * np.cos(angles), 0.5 * np.sin(angles), np.ones_like(angles)))
        else:
            result = np.asarray(positions, dtype=float)
            if result.shape == (self.config.num_drones, 2):
                result = np.column_stack((result, np.ones(self.config.num_drones)))
        if result.shape != (self.config.num_drones, 3):
            raise ValueError(
                f"initial_positions must have shape ({self.config.num_drones}, 2 or 3)"
            )
        if np.any(result[:, 2] <= 0):
            raise ValueError("initial altitudes must be positive")
        return result

    @property
    def timestep(self) -> float:
        return 1.0 / self.config.control_frequency_hz

    @property
    def state(self) -> SwarmState:
        return SwarmState.from_aviary_observation(
            self._observation, self._step_index * self.timestep
        )

    def reset(self) -> SwarmState:
        """Reset simulation time, vehicles and PID controller memory."""

        self._observation, _ = self._env.reset()
        self._step_index = 0
        for controller in self._controllers:
            controller.reset()
        return self.state

    def step(
        self,
        target_positions: ArrayLike,
        target_velocities: ArrayLike | None = None,
    ) -> SwarmState:
        """Advance one control period towards per-drone position targets."""

        targets = self._positions_3d(target_positions)
        velocities = (
            np.zeros((self.config.num_drones, 3))
            if target_velocities is None
            else self._velocities_3d(target_velocities)
        )

        rpm = np.zeros((self.config.num_drones, 4))
        for drone, controller in enumerate(self._controllers):
            rpm[drone], _, _ = controller.computeControlFromState(
                control_timestep=self.timestep,
                state=self._observation[drone],
                target_pos=targets[drone],
                target_vel=velocities[drone],
            )
        return self.step_motor_rpm(rpm)

    def step_velocity(self, target_velocities: ArrayLike, horizon: float = 0.5) -> SwarmState:
        """Advance one period using velocity commands and a receding position target."""

        if horizon <= 0:
            raise ValueError("horizon must be positive")
        velocities = self._velocities_3d(target_velocities)
        targets = self.state.positions + horizon * velocities
        if self.config.dimensions == 2:
            targets[:, 2] = self._fixed_altitudes
        return self.step(targets, velocities)

    def step_motor_rpm(self, motor_rpm: ArrayLike) -> SwarmState:
        """Advance one period with raw ``(N, 4)`` motor RPM commands."""

        rpm = np.asarray(motor_rpm, dtype=float)
        expected = (self.config.num_drones, 4)
        if rpm.shape != expected:
            raise ValueError(f"motor_rpm must have shape {expected}, got {rpm.shape}")
        self._observation, _, _, _, _ = self._env.step(rpm)
        self._step_index += 1
        return self.state

    def _positions_3d(self, values: ArrayLike) -> NDArray[np.float64]:
        array = np.asarray(values, dtype=float)
        if self.config.dimensions == 2 and array.shape == (self.config.num_drones, 2):
            array = np.column_stack((array, self._fixed_altitudes))
        expected = (self.config.num_drones, 3)
        if array.shape != expected:
            raise ValueError(f"target_positions must have shape {expected}, got {array.shape}")
        result = array.copy()
        if self.config.dimensions == 2:
            result[:, 2] = self._fixed_altitudes
        return result

    def _velocities_3d(self, values: ArrayLike) -> NDArray[np.float64]:
        array = np.asarray(values, dtype=float)
        if self.config.dimensions == 2 and array.shape == (self.config.num_drones, 2):
            array = np.column_stack((array, np.zeros(self.config.num_drones)))
        expected = (self.config.num_drones, 3)
        if array.shape != expected:
            raise ValueError(f"target_velocities must have shape {expected}, got {array.shape}")
        result = array.copy()
        if self.config.dimensions == 2:
            result[:, 2] = 0.0
        return result

    def close(self) -> None:
        self._env.close()

    def __enter__(self) -> "SwarmSimulator":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
