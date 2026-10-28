"""Lightweight fixed-wing swarm simulation and PyBullet visualisation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .state import SwarmState


GRAVITY = 9.81


def _wrap_angle(angle: NDArray[np.float64]) -> NDArray[np.float64]:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


@dataclass(frozen=True, slots=True)
class FixedWingConfig:
    """Configuration for aircraft initialized on a common circular orbit."""

    num_drones: int = 4
    radius: float = 25.0
    altitude: float = 12.0
    airspeed: float = 12.0
    control_frequency_hz: int = 60
    clockwise: bool = False
    gui: bool = False
    output_directory: str = "results"
    maximum_bank_angle_deg: float = 55.0
    maximum_roll_rate_deg_s: float = 90.0
    maximum_acceleration: float = 4.0
    maximum_climb_rate: float = 4.0

    def __post_init__(self) -> None:
        if self.num_drones < 1:
            raise ValueError("num_drones must be at least one")
        if self.radius <= 0 or self.altitude <= 0 or self.airspeed <= 0:
            raise ValueError("radius, altitude and airspeed must be positive")
        if self.control_frequency_hz <= 0:
            raise ValueError("control_frequency_hz must be positive")
        if not 0 < self.maximum_bank_angle_deg < 90:
            raise ValueError("maximum_bank_angle_deg must be between 0 and 90")


@dataclass(frozen=True, slots=True)
class FixedWingCommand:
    """Per-aircraft commands, with one value per drone.

    Bank angles are in radians, target airspeeds in m/s and climb rates in m/s.
    """

    bank_angles: ArrayLike
    target_airspeeds: ArrayLike
    climb_rates: ArrayLike


class FixedWingSwarmSimulator:
    """Point-mass fixed-wing model with coordinated turns.

    The horizontal turn rate follows ``g * tan(bank) / airspeed``. Roll, speed
    and climb-rate responses are rate limited. PyBullet provides the 3D view;
    integration is deterministic and can run headlessly faster than real time.
    """

    _COLORS = (
        (0.90, 0.16, 0.16, 1.0),
        (0.12, 0.45, 0.95, 1.0),
        (0.12, 0.75, 0.30, 1.0),
        (0.95, 0.62, 0.08, 1.0),
        (0.65, 0.25, 0.90, 1.0),
        (0.10, 0.75, 0.75, 1.0),
    )

    def __init__(self, config: FixedWingConfig = FixedWingConfig()) -> None:
        self.config = config
        self._p, self._pybullet_data = self._load_pybullet()
        Path(config.output_directory).mkdir(parents=True, exist_ok=True)
        connection_mode = self._p.GUI if config.gui else self._p.DIRECT
        self._client = self._p.connect(connection_mode)
        if self._client < 0:
            raise RuntimeError("could not connect to PyBullet")
        self._body_ids: list[int] = []
        self._configure_world()
        self._initialize_state()
        self._create_aircraft()
        self._update_visuals()

    @staticmethod
    def _load_pybullet() -> tuple[Any, Any]:
        try:
            import pybullet
            import pybullet_data
        except ImportError as error:
            raise RuntimeError(
                "simulation dependencies are missing; run ./scripts/setup.sh then "
                "source .venv/bin/activate"
            ) from error
        return pybullet, pybullet_data

    @property
    def timestep(self) -> float:
        return 1.0 / self.config.control_frequency_hz

    def _configure_world(self) -> None:
        p = self._p
        p.resetSimulation(physicsClientId=self._client)
        p.setAdditionalSearchPath(self._pybullet_data.getDataPath(), physicsClientId=self._client)
        p.setGravity(0, 0, -GRAVITY, physicsClientId=self._client)
        p.setTimeStep(self.timestep, physicsClientId=self._client)
        p.loadURDF("plane.urdf", physicsClientId=self._client)

        if self.config.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self._client)
            p.resetDebugVisualizerCamera(
                cameraDistance=max(35.0, self.config.radius * 2.7),
                cameraYaw=-35,
                cameraPitch=-42,
                cameraTargetPosition=[0, 0, self.config.altitude * 0.5],
                physicsClientId=self._client,
            )
        self._draw_orbit()

    def _draw_orbit(self) -> None:
        points = 120
        angles = np.linspace(0, 2 * np.pi, points + 1)
        for first, second in zip(angles[:-1], angles[1:]):
            start = [
                self.config.radius * np.cos(first),
                self.config.radius * np.sin(first),
                self.config.altitude,
            ]
            end = [
                self.config.radius * np.cos(second),
                self.config.radius * np.sin(second),
                self.config.altitude,
            ]
            self._p.addUserDebugLine(
                start,
                end,
                lineColorRGB=[0.95, 0.80, 0.10],
                lineWidth=2,
                physicsClientId=self._client,
            )

    def _initialize_state(self) -> None:
        count = self.config.num_drones
        direction = -1.0 if self.config.clockwise else 1.0
        angles = np.arange(count, dtype=float) * 2.0 * np.pi / count
        self._positions = np.column_stack(
            (
                self.config.radius * np.cos(angles),
                self.config.radius * np.sin(angles),
                np.full(count, self.config.altitude),
            )
        )
        self._headings = _wrap_angle(angles + direction * np.pi / 2.0)
        nominal_bank = np.arctan(self.config.airspeed**2 / (GRAVITY * self.config.radius))
        self._rolls = np.full(count, direction * nominal_bank)
        self._pitches = np.zeros(count)
        self._airspeeds = np.full(count, self.config.airspeed)
        self._climb_rates = np.zeros(count)
        self._angular_velocities = np.zeros((count, 3))
        self._step_index = 0

    def _create_aircraft(self) -> None:
        p = self._p
        count = self.config.num_drones
        scale = max(0.75, min(1.6, self.config.radius / 20.0))
        for drone in range(count):
            color = self._COLORS[drone % len(self._COLORS)]
            dark = tuple(max(0.0, channel * 0.65) for channel in color[:3]) + (1.0,)
            base_collision = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=np.multiply([0.72, 0.09, 0.09], scale),
                physicsClientId=self._client,
            )
            base_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=np.multiply([0.72, 0.09, 0.09], scale),
                rgbaColor=color,
                physicsClientId=self._client,
            )
            wing_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=np.multiply([0.20, 0.88, 0.025], scale),
                rgbaColor=color,
                physicsClientId=self._client,
            )
            tail_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=np.multiply([0.18, 0.32, 0.02], scale),
                rgbaColor=dark,
                physicsClientId=self._client,
            )
            fin_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=np.multiply([0.17, 0.025, 0.16], scale),
                rgbaColor=dark,
                physicsClientId=self._client,
            )
            body = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=base_collision,
                baseVisualShapeIndex=base_visual,
                basePosition=self._positions[drone],
                linkMasses=[0, 0, 0],
                linkCollisionShapeIndices=[-1, -1, -1],
                linkVisualShapeIndices=[wing_visual, tail_visual, fin_visual],
                linkPositions=np.multiply(
                    [[0.05, 0, 0], [-0.56, 0, 0.02], [-0.56, 0, 0.17]], scale
                ).tolist(),
                linkOrientations=[[0, 0, 0, 1]] * 3,
                linkInertialFramePositions=[[0, 0, 0]] * 3,
                linkInertialFrameOrientations=[[0, 0, 0, 1]] * 3,
                linkParentIndices=[0, 0, 0],
                linkJointTypes=[p.JOINT_FIXED] * 3,
                linkJointAxis=[[0, 0, 0]] * 3,
                physicsClientId=self._client,
            )
            self._body_ids.append(body)

    @property
    def state(self) -> SwarmState:
        horizontal_speed = self._airspeeds * np.cos(self._pitches)
        velocities = np.column_stack(
            (
                horizontal_speed * np.cos(self._headings),
                horizontal_speed * np.sin(self._headings),
                self._climb_rates,
            )
        )
        return SwarmState(
            time=self._step_index * self.timestep,
            positions=self._positions.copy(),
            velocities=velocities,
            orientations_rpy=np.column_stack((self._rolls, self._pitches, self._headings)),
            angular_velocities=self._angular_velocities.copy(),
            motor_rpm=np.zeros((self.config.num_drones, 4)),
        )

    def loiter_command(
        self,
        radius: float | None = None,
        altitude: float | None = None,
        airspeed: float | None = None,
        clockwise: bool | None = None,
    ) -> FixedWingCommand:
        """Create commands that converge to and maintain a shared orbit."""

        radius = self.config.radius if radius is None else radius
        altitude = self.config.altitude if altitude is None else altitude
        airspeed = self.config.airspeed if airspeed is None else airspeed
        clockwise = self.config.clockwise if clockwise is None else clockwise
        if radius <= 0 or altitude <= 0 or airspeed <= 0:
            raise ValueError("radius, altitude and airspeed must be positive")

        direction = -1.0 if clockwise else 1.0
        relative = self._positions[:, :2]
        distances = np.linalg.norm(relative, axis=1)
        radial_angles = np.arctan2(relative[:, 1], relative[:, 0])
        tangent_courses = radial_angles + direction * np.pi / 2.0
        radial_error = distances - radius
        desired_courses = tangent_courses + direction * np.arctan(0.12 * radial_error)
        course_error = _wrap_angle(desired_courses - self._headings)

        feed_forward_turn_rate = direction * airspeed / radius
        desired_turn_rates = feed_forward_turn_rate + 1.8 * course_error
        desired_banks = np.arctan(desired_turn_rates * self._airspeeds / GRAVITY)
        maximum_bank = np.deg2rad(self.config.maximum_bank_angle_deg)
        desired_banks = np.clip(desired_banks, -maximum_bank, maximum_bank)
        climb_rates = np.clip(
            0.8 * (altitude - self._positions[:, 2]),
            -self.config.maximum_climb_rate,
            self.config.maximum_climb_rate,
        )
        return FixedWingCommand(
            bank_angles=desired_banks,
            target_airspeeds=np.full(self.config.num_drones, airspeed),
            climb_rates=climb_rates,
        )

    def step_loiter(self) -> SwarmState:
        """Advance one step while maintaining the configured common orbit."""

        return self.step(self.loiter_command())

    def step(self, command: FixedWingCommand) -> SwarmState:
        """Advance the coordinated-turn model by one control period."""

        count = self.config.num_drones
        banks = self._command_array(command.bank_angles, "bank_angles", count)
        target_speeds = self._command_array(command.target_airspeeds, "target_airspeeds", count)
        climb_rates = self._command_array(command.climb_rates, "climb_rates", count)
        if np.any(target_speeds <= 0):
            raise ValueError("target_airspeeds must be positive")

        dt = self.timestep
        previous_rolls = self._rolls.copy()
        previous_pitches = self._pitches.copy()
        maximum_bank = np.deg2rad(self.config.maximum_bank_angle_deg)
        maximum_roll_step = np.deg2rad(self.config.maximum_roll_rate_deg_s) * dt
        bank_error = np.clip(banks, -maximum_bank, maximum_bank) - self._rolls
        self._rolls += np.clip(bank_error, -maximum_roll_step, maximum_roll_step)

        speed_error = target_speeds - self._airspeeds
        self._airspeeds += np.clip(
            speed_error,
            -self.config.maximum_acceleration * dt,
            self.config.maximum_acceleration * dt,
        )
        self._climb_rates = np.clip(
            climb_rates,
            -self.config.maximum_climb_rate,
            self.config.maximum_climb_rate,
        )
        self._pitches = np.arctan2(self._climb_rates, self._airspeeds)

        heading_rates = GRAVITY * np.tan(self._rolls) / self._airspeeds
        self._headings = _wrap_angle(self._headings + heading_rates * dt)
        horizontal_speed = self._airspeeds * np.cos(self._pitches)
        velocities = np.column_stack(
            (
                horizontal_speed * np.cos(self._headings),
                horizontal_speed * np.sin(self._headings),
                self._climb_rates,
            )
        )
        self._positions += velocities * dt
        self._angular_velocities = np.column_stack(
            (
                (self._rolls - previous_rolls) / dt,
                (self._pitches - previous_pitches) / dt,
                heading_rates,
            )
        )
        self._step_index += 1
        self._update_visuals(velocities)
        self._p.stepSimulation(physicsClientId=self._client)
        return self.state

    @staticmethod
    def _command_array(values: ArrayLike, name: str, count: int) -> NDArray[np.float64]:
        array = np.asarray(values, dtype=float)
        if array.ndim == 0:
            array = np.full(count, float(array))
        if array.shape != (count,):
            raise ValueError(f"{name} must be a scalar or have shape ({count},), got {array.shape}")
        if not np.all(np.isfinite(array)):
            raise ValueError(f"{name} must contain only finite values")
        return array

    def _update_visuals(self, velocities: NDArray[np.float64] | None = None) -> None:
        if velocities is None:
            velocities = self.state.velocities
        for drone, body in enumerate(self._body_ids):
            orientation = self._p.getQuaternionFromEuler(
                [self._rolls[drone], self._pitches[drone], self._headings[drone]]
            )
            self._p.resetBasePositionAndOrientation(
                body,
                self._positions[drone],
                orientation,
                physicsClientId=self._client,
            )
            self._p.resetBaseVelocity(
                body,
                linearVelocity=velocities[drone],
                angularVelocity=self._angular_velocities[drone],
                physicsClientId=self._client,
            )

    def reset(self) -> SwarmState:
        """Place every aircraft back on its evenly spaced initial orbit."""

        self._initialize_state()
        self._update_visuals()
        return self.state

    def close(self) -> None:
        if self._p.isConnected(self._client):
            self._p.disconnect(self._client)

    def __enter__(self) -> "FixedWingSwarmSimulator":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
