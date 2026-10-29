# Drone swarm simulation environment

This project provides a small Python API for developing multi-drone algorithms
without exposing physics-engine details. The selected backend is
[gym-pybullet-drones](https://github.com/learnsyslab/gym-pybullet-drones), built
on PyBullet and Gymnasium. It simulates Crazyflie 2.x quadrotors with 6-DoF
dynamics; ground effect, drag and downwash models are available in the backend.

## Repository navigation

| Location | Purpose |
|---|---|
| [Documentation index](docs/README.md) | Usage, models, planning, validation and PX4/Gazebo guides |
| [Script index](scripts/README.md) | Setup, benchmarks, campaigns, reports and visualizations |
| `src/swarm_sim/` | Library code; coordinated trajectory planning is in `trajectory2d/` |
| `native/` | Optional standalone C++20 physical construction kernel and Python binding |
| `examples/` | Launchers, reusable scenarios, aircraft profiles and Gazebo fixtures |
| `tests/` | Regression tests; run with `./scripts/test.sh` |
| `results/` | Ignored local artifacts, including failed trials and original evidence |

For existing local outputs, open `results/README.md`. Regenerate its grouped
inventory with `.venv/bin/python scripts/index_results.py`. Downloadable
archives are grouped under `results/exports/`; earlier published archive paths
remain available as compatibility links. Historical run directories retain
their original paths and contents.

## Why this backend?

| Solution | Interfaces | Strengths | Limitations / best use |
|---|---|---|---|
| **gym-pybullet-drones + PyBullet** | Python | Relatively lightweight installation, multi-agent support, Gymnasium/RL, direct access to the full state, accelerated headless simulation | More limited sensor/rendering fidelity; well suited to algorithm development |
| **PX4 SITL + Gazebo** | C++, ROS 2, MAVSDK Python/C++ | Actual PX4 autopilot, sensors/noise, cameras, 3D worlds, multiple vehicles | Heavier installation and computation; well suited to validation before hardware deployment |
| **Webots** | C, C++, Python, ROS 2 | Cross-platform, 3D editor, separate controller per robot, Supervisor API | Flight models and autopilot integration need to be configured for the aircraft |
| **AirSim / Project AirSim** | Python, C++, RPC | High-quality Unreal rendering, cameras, multiple vehicles | The original AirSim is no longer actively developed; migration needs evaluation |

The repository isolates the quadrotor backend behind `SwarmSimulator`. A future
Gazebo/PX4 integration could therefore provide the same `SwarmState` without
changing the algorithms. The coordinated fixed-wing trajectory prototype
described below uses its own flight model rather than this PyBullet backend.

## Installation

Requirements: Linux, Python 3.12 and a C/C++ compiler (`build-essential` on
Ubuntu). With Python 3.11+, PyBullet may be compiled during installation.

```bash
chmod +x scripts/setup.sh
./scripts/setup.sh
source .venv/bin/activate
```

The setup script pins a specific `gym-pybullet-drones` commit and does not
install its large optional reinforcement-learning dependencies (PyTorch and
Stable-Baselines3). These can be installed separately if RL is needed later.

The physical trajectory generator also has an optional C++20 backend. Build it
with `./scripts/build_native.sh`, then select
`--generator-mode physical_mission --numerical-backend cpp --workers 1` in the
trajectory CLI. Python remains the default and independently validates native
candidates. See [build instructions and measured results](docs/native_kernel.md).

## Running the examples

A **coordinated 3D planning prototype** is available for fixed-wing aircraft:
reproducible pseudorandom generation, joint selection with separation margins,
then independent local tracking. Each aircraft has its own loiter circle
(four separate circles by default), visible on the map before departure toward
the destination. Altitude layers are configurable, with bounded climbs and
descents and separation checked in XYZ.
See the [3D guide](docs/trajectory_3d.md) for assumptions and limitations.
The [planar prototype](docs/trajectory_2d.md) remains available with `--dimensions 2`.
The launcher starts with small candidate catalogs and enriches conflicting
catalogs when needed. It searches the most constrained drone first. See
[adaptive planning and diagnostics](docs/adaptive_planning.md) for settings,
fixed-mode comparison and the four-drone benchmark. Every planning run exports
`planning_report.json`, including failed searches.
Sampled pair conflicts now also guide new altitude, detour and bounded-departure
proposals. `--conflict-guidance none` restores unguided expansion; see
[witness-guided planning and four-aircraft comparison](docs/conflict_guidance.md).
The launcher also prefers feasible combinations with greater minimum robust
inter-drone margin, with bounded refinement and one targeted catalog extension.
`--selection-policy first` restores first-feasible selection. See
[margin-preferring selection](docs/margin_selection.md) for controls and the
four-aircraft comparison.
Known static cylinders and rectangular buildings can be supplied through CLI
or JSON. The generator proposes detours and checks full-horizon 3D obstacle
clearance with uncertainty margins. See the [obstacle guide](docs/obstacles.md)
for the four-aircraft example and current limitations.
In 3D, the CLI also explores obstacle-derived overflight and underflight cruise
levels, combined with lateral detours. `--obstacle-routing lateral` restores
the historical proposal heuristic; independent XYZ validation is always retained.
Separate arrival deadlines and minimum post-passage observation can be set
using `--arrival-deadline` and `--post-arrival-time`, or the editor's timing
fields. The horizon still covers the complete mission and recovery. See
[mission timing and detailed rejection diagnostics](docs/mission_timing.md).
The coupled executor also estimates wind from its own motion, schedules updates
at local plan-control switches and numerically protects the final held command.
Constant/gust wind, bounded position-measurement errors and clock drift can be
tested reproducibly. Reports identify first observed safety-condition losses;
see [disturbed tracking and four-drone campaigns](docs/robust_tracking.md).
Coupled passage checks now localize the first contact event instead of depending
on the observer grid. Optional in-flight preparation models loiter motion during
planning/loading and blocks late or unacknowledged activation. Optional cell/time
reservations allocate complete flyable plans without replacing distance checks.
See [contact, preparation and reservation workflow](docs/mission_pipeline.md).
Optional free-window-guided generation now biases departure proposals before
rollout. Multiple cruise levels and smooth airspeed-reference profiles are also
available; see [free windows and richer 3D profiles](docs/free_windows_profiles.md)
for settings, four-aircraft comparisons and limitations.
An explicit [adaptive mixed-profile policy](docs/mixed_profiles.md) can introduce
richer families only during catalog expansion or repeated proposal rejection;
strict requested profiles remain the default.
Optional [intermediate passage windows](docs/intermediate_passages.md) guide
local speed variation before rollout, including with fixed departures; every
cell visit and the full integrated fleet are still independently checked.
An incremental [parameter campaign](docs/parameter_campaign.md) tests four
aircraft under varied configurations/disturbances and retains failures.
The standalone `scripts/audit_campaigns.py` checks saved journals for missing
trials, contradictory acceptance records and incomplete campaigns without
rerunning the simulator; see the campaign guide for commands and limitations.
Optional rejection feedback adjusts settling holds and route padding without
changing physical or safety requirements.
An isolated [vertical-flight verification bench](docs/vertical_flight_validation.md)
checks level flight, mild climb/descent and return to level at several speeds,
without destinations, planner calls or changes to the flight model.
The [expanded envelope experiments](docs/flight_envelope_validation.md) add
boundary checks, bank/slope equilibrium classifications and wider speed/time-step
sweeps, preserving rejected cases and numerical precision failures.
Versioned [fixed-wing aircraft profiles](docs/aircraft_profiles.md) load
physical envelopes from JSON using `--aircraft-profile`. Explicit per-ID
`--drone-profile ID JSON` assignments also support
[heterogeneous fixed-wing fleets](docs/heterogeneous_aircraft.md), including
optional enclosing-sphere clearances, editor loading and profile-aware laboratory
checks. Supplied profiles, including the internet-inspired Rascal case study,
are illustrative approximations, not calibrated aircraft or a JSBSim backend.

An optional [single-aircraft PX4/Gazebo experiment](docs/px4_gazebo.md) now exports
the generated reference to an isolated open-source `rc_cessna` SITL mission
(with a downward lidar/extended-ground test fixture) and
records ground-truth spatial tracking. It does not transfer timing/separation
guarantees or replace the fast point-mass planner. See the guide for the pinned
prerelease runtime, optional dependencies and `examples/px4_gazebo.py --run`.

The [shared editor → Gazebo guide](docs/gazebo_editor.md) describes editing one
aircraft, its holding circle, target and obstacles, then flying the same scene:

```bash
.venv/bin/python examples/px4_gazebo.py --edit-scenario --run --gui --gpu \
  --scenario examples/scenarios/px4_single_loiter.json --timeout 1200
```

An opt-in [four-aircraft shared-world experiment](docs/gazebo_fleet.md) uses
`--num-drones 4` and `examples/scenarios/px4_four_loiter.json`. It jointly plans
the references, then observes four independent PX4 missions; PX4 timing and
collision freedom are not certified.

```bash
source .venv/bin/activate
python examples/trajectory_3d.py
# The first window lets you position the drones and destination.
# Alternatively, select the destination on an XY map:
python examples/trajectory_2d.py --pick-target
```

Select a drone or the target in the first window, then click on the map to move
it. The x/y fields also allow precise input, and each drone's heading and speed
can be edited (**total airspeed** setpoint between 8 and 16 m/s in coupled 3D
mode). `--airspeed 15` sets a common initial speed; repeat `--drone-speed` once
per drone to specify different speeds.
The global **Arrivées (°)** (approach spread) field controls the approach sides:
0° preserves the original routes; 360° distributes four arrival directions
around the target. The corresponding option is `--approach-spread 360`
(allow enough time for detours, for example `--horizon 240`).

The launcher generates catalogs in parallel on the available CPUs:
`--workers 0` selects automatically (default), `--workers 2` or `4` sets a cap,
and `--workers 1` keeps execution sequential. Checks are batched without
removing validation points. The [performance guide](docs/planning_performance.md)
explains the settings and the three-scenario benchmark.

Approach angles are assigned automatically based on geometry: drones on the
right favor right-hand sectors, and vice versa. A global assignment minimizes
distances to reference approach gates to avoid unnecessary crossings. Colored,
labeled arrows near the target show the assigned sectors; complete trajectories
are still checked separately.

The **Attente min (s) / max (s)** (minimum/maximum wait) fields set the extra
departure delay: `0 / 0` requests simultaneous departures. On the command line,
use `--departure-delay 0 0` or `--departure-delay 0 2` to wait at most two extra
seconds. The common 3 s delay is configurable with `--activation-time`.
Click **Lancer la mission** (launch mission) to calculate the trajectories and
open the simulation. `--no-edit-scenario` runs the supplied scenario directly.

The viewer supports play/pause and seeking through time. Reports, individual
plans, telemetry and a PNG figure are exported for every run.
3D mode starts with altitude layers at 30, 55, 80 and 105 m. The **Altitude**
fields describe the initial positions. By default, **Croisière automatique**
(automatic cruise altitude) lets the planner choose levels within a common
30–120 m range, configurable in the interface or with `--cruise-range 40 110`.
The choices are validated jointly with the trajectories. Uncheck the box to
restore the individual **Croisière** (cruise altitude) setting;
`--manual-cruise` selects this mode on the command line.

The default target is a common XYZ point at an altitude of 30 m: select **Cible**
(target) to change x/y and its altitude. Different path lengths allow staggered
arrivals even with simultaneous departures. `--arrival-mode layers` remains
available for comparison with an XY destination reached at different altitudes.

The 3D engine is a **coupled point-mass model**: total airspeed, flight-path
angle, bank angle, lift, thrust and drag evolve together. It checks load factor,
stall margin, roll, propulsion and braking. The **Vitesse air** (airspeed)
field no longer represents horizontal speed: at 12 m/s and −30°, the descent
rate is 6 m/s. The [flight model guide](docs/flight_model_3d.md) details the
equations, illustrative parameters (including an optional airbrake) and
limitations. `--flight-model legacy` preserves the historical independent XY/Z
model.

The global **Descente (°)** (descent angle) field or `--descent-angle 30` sets
the flight-path angle when passing the XYZ point, followed by a pull-out.
Requests incompatible with speed, cruise altitude or minimum altitude are
rejected. The XYZ launcher defaults to 30°; `--descent-angle 0` requests a level
pass. The viewer and CSV include total airspeed; the report includes load factor,
bank angle and stall margin. Validation is numerical and conditional: this is
neither a real-flight certification nor a 6-DoF model.

```bash
# Rotating 3D formation with a PyBullet window
python examples/formation.py --num-drones 5 --dimensions 3

# Same dynamics, with the algorithm restricted to XY and altitude held constant
python examples/formation.py --num-drones 8 --dimensions 2

# Local cohesion / alignment / separation algorithm
python examples/flocking.py --num-drones 12 --dimensions 3

# Fast headless execution
python examples/formation.py --duration 3 --no-gui

# Four fixed-wing drones on the same ring
python examples/fixed_wing_ring.py
```

The formation example writes `results/telemetry.csv`, with one row per drone
per control step.

### Fixed-wing aircraft

By default, `fixed_wing_ring.py` places four aircraft at regular intervals on a
25 m radius ring, at an altitude of 12 m and an airspeed of 12 m/s. A yellow
line marks the orbit in the PyBullet view.

```bash
# Configure the scenario and fly clockwise
python examples/fixed_wing_ring.py \
  --num-drones 4 --radius 30 --altitude 15 --airspeed 14 --clockwise

# Quick headless validation
python examples/fixed_wing_ring.py --duration 5 --no-gui
```

This backend uses a coordinated-turn point-mass model:
`turn_rate = g × tan(bank_angle) / speed`. Bank angle, acceleration and vertical
speed are bounded. It is suitable for developing formation and guidance laws;
PX4/Gazebo is a subsequent step toward simulating full aerodynamics, control
surfaces and autopilot behavior.

Data is written to `results/fixed_wing_telemetry.csv`.
`FixedWingSwarmSimulator.state` exposes the same position, velocity and
orientation fields as the quadrotor simulator.

## Minimal API

```python
import numpy as np
from swarm_sim import SimulatorConfig, SwarmSimulator

config = SimulatorConfig(num_drones=4, dimensions=3, gui=False)

with SwarmSimulator(config) as simulation:
    for _ in range(500):
        state = simulation.state

        # Telemetry (NumPy arrays with shape N x 3)
        print(state.positions)
        print(state.velocities)
        print(state.orientations_rpy)
        print(state.angular_velocities)

        targets = state.positions.copy()
        targets[:, 0] += 0.2
        state = simulation.step(targets)
```

Units are SI, and the world frame is XYZ with Z pointing upward. Available
`SwarmState` fields are:

- `time` in seconds;
- `positions[N, 3]` in meters;
- `velocities[N, 3]` in m/s;
- `orientations_rpy[N, 3]` in radians;
- `angular_velocities[N, 3]` in rad/s;
- `motor_rpm[N, 4]`.

Three control levels are provided:

- `step(target_positions, target_velocities=None)`: position setpoints;
- `step_velocity(target_velocities)`: velocity setpoints;
- `step_motor_rpm(motor_rpm)`: direct control of the four motors.

`state.pairwise_distances()` and `state.adjacency(radius)` help implement
distributed algorithms based on local neighborhoods. `examples/flocking.py`
demonstrates this loop.

## Tests

```bash
./scripts/test.sh
```

The test launcher prevents an already sourced ROS 2 installation from loading
its system pytest plugins into the project's virtual environment.

For an end-to-end physics-backend check:

```bash
python examples/formation.py --duration 1 --no-gui
head results/telemetry.csv
```

## Path toward higher fidelity

1. Quickly develop and test formation, consensus, avoidance and allocation
   algorithms in this PyBullet environment.
2. Add noise, communication delays and packet loss to the observation/control
   interface.
3. Implement a PX4 SITL + Gazebo backend exposing the same `SwarmState`, then
   test actual autopilot constraints through ROS 2 or MAVSDK.
4. Deploy the offboard algorithm on physical drones without changing its core.
