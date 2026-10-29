# 3D point-mass flight model

3D mode defaults to `flight_model="point_mass"`. The engine is implemented in
`src/swarm_sim/trajectory2d/flight3d.py`, separately from the planner, its
geometric adaptation (`coupled_planning.py`) and local execution (`execution.py`).
The historical package name `trajectory2d` is retained for imports.
Physical parameter sets can now be loaded through strict, versioned
[aircraft profiles](aircraft_profiles.md). A run can use one common profile or
explicit [per-drone airframe assignments](heterogeneous_aircraft.md); this does
not add a new physics backend or calibration guarantees.

## State, reference frames and equations

The state is `[x,y,z,V,psi,gamma,phi]`, in meters, m/s and radians. Z points
upward; heading 0 points toward +X and π/2 toward +Y. `V` is **total airspeed**,
`psi` is its heading, `gamma` is the flight-path angle (positive when climbing),
and `phi` is bank angle. Flight-path angle is not pitch attitude: this model
does not simulate wing angle of attack.

Controls are `[a_T,a_B,n,p]`: thrust divided by mass, additional drag divided
by mass, load factor `L/(mg)` and roll rate. `a_B` represents an **idealized
airbrake**, not reverse thrust. Assuming coordinated flight without sideslip,
thrust parallel to air-relative velocity and constant atmospheric density:

```text
xdot     = V cos(gamma) cos(psi) + wind_x
ydot     = V cos(gamma) sin(psi) + wind_y
zdot     = V sin(gamma)         + wind_z
Vdot     = a_T - a_B - drag(V,n) - g sin(gamma)
psidot   = g n sin(phi) / (V cos(gamma))
gammadot = g [n cos(phi) - cos(gamma)] / V
phidot   = p
drag(V,n) = c0 V² + ci n²/V²                 # D/m, m/s²
```

In a steady turn with constant flight-path angle, `n=cos(gamma)/cos(phi)`, hence
`psidot=g*tan(phi)/V`. In level flight, `R=V²/[g*tan(phi)]` and `n=1/cos(phi)`.
During a pull-out, however, load factor changes **both** heading and flight-path
angle. Changing bank without increasing lift therefore causes altitude loss.
Thrust and gravity actually change airspeed; they are not replaced by an
artificial constant-speed constraint.

These equations correspond to the coordinated
[MathWorks point-mass model](https://www.mathworks.com/help/aeroblks/fixedwingpointmass.html),
specialized here to thrust aligned with air-relative velocity.
The increase in stall speed with the square root of load factor is described
in the [FAA handbook](https://www.faa.gov/sites/faa.gov/files/pilots/pilot_handbook.pdf).

## Flight envelope and illustrative parameters

| Parameter | Launcher default | Option |
|---|---:|---|
| Allowed total airspeed | 8–16 m/s | `--airspeed`, `--drone-speed`: individual setpoints within this range |
| 1g stall speed | 6 m/s | `--stall-speed` |
| Multiplicative stall margin | 1.15 | `--stall-margin` |
| Load factor | 0.2–2.5 | `--max-load-factor` for the upper bound |
| Bank angle and roll rate | 50°, 60°/s | `--max-bank`, `--max-roll-rate` |
| Absolute flight-path angle and its rate | 45°, 8°/s | `--max-flight-path-angle`, `--max-flight-path-rate` |
| Available thrust / mass | 5 m/s² | `--max-thrust-acceleration` |
| Additional airbrake drag / mass | 5 m/s² | `--max-airbrake-acceleration`; **0 disables it** |
| Drag coefficients `c0`, `ci` | 0.003; 20 | `--parasitic-drag`, `--induced-drag` |
| Maximum climb / descent rate | +3 / −10 m/s | `--max-climb-rate`, `--max-descent-rate` |
| Maximum vertical acceleration | 1 m/s² | `--max-vertical-acceleration` |
| Altitude | 0–200 m | `--max-altitude` for the ceiling |

The stall check is `V >= margin * V_stall_1g * sqrt(n)`. It limits available
load factor together with the structural limit. The simulator does **not** model
post-stall behavior: a violation invalidates the run. Climb/descent-rate and
vertical-acceleration limits remain additional constraints on the physical
envelope, not independent controls.

These numbers do not describe any certified aircraft: **calibrate this table
against the selected aircraft** before interpreting results physically. In
particular, an airbrake providing 5 m/s² is a substantial illustrative capability,
not equipment assumed to exist on every drone. It can be removed; a resulting
infeasible constant-speed descent is then explicitly rejected.

At 12 m/s and −30°, the components are `vz=-6 m/s` and `vxy=10.392 m/s`.
Gravity contributes approximately 4.9 m/s² along the flight path. Baseline drag
is insufficient to hold 12 m/s: the controller must brake. Without braking,
the physics engine lets the aircraft accelerate rather than resetting its speed.
The default catalog tracks the supplied setpoint. Optional smooth pre-approach
speed references now explore acceleration/deceleration through physical controls;
see [richer profiles](free_windows_profiles.md). They do not relax descent,
braking or stall limits and may still yield no candidate.

## From a proposed path to an executable plan

The following describes the default geometric generator. The opt-in
[`physical_mission` generator](phase2_kinodynamic.md) constructs its states and
controls directly, bypassing the legacy vertical template and geometric time
scaling. It uses the same physical dynamics and independent full-horizon
validation and joint separation selection.

1. The historical generator proposes a continuous XY/Z curve with detours,
   a cruise altitude and passage through the XYZ target neighborhood, followed
   by a pull-out.
2. Time is stretched: `dt_real/dtau = sqrt(vxy²+vz²)/V_setpoint`.
   Geometry is retained as a **reference**, without adding independent velocity
   components on top of the requested airspeed. Horizontal loiter phases retain
   their duration.
3. A local controller tracks this reference using the point-mass model while
   respecting thrust, braking, load factor, stall and roll-rate limits. Controls
   are held over 0.2 s steps and integrated using RK4 with two subdivisions.
4. Independent validation reconstructs the steps and checks the envelope at
   endpoints and three interior points using refined integration. Both sides
   of control changes are checked. It also checks XYZ passage, flight-path
   angle at reintegrated first contact and nominal passage (1° tolerance),
   return to cruise altitude (0.2 m and 0.1 m/s), and
   the drone's continued presence until the horizon.
5. Joint search selects only compatible integrated trajectories. Conversion is
   **never** applied after separation checking, which would invalidate that
   check's result.

The optional preparation timeline and conservative cell/time reservations are
described in [mission pipeline](mission_pipeline.md). Neither replaces the
flight-envelope, obstacle or pair-distance checks.

Geometric construction remains deliberately conservative and may reject
physically feasible maneuvers. An invalid precondition is distinct from
`no_candidate` / `catalog_exhausted`: failure within a finite catalog does not
prove that a mission is impossible.

## Execution and guarantees

Each drone executes its nominal controls with corrections based on **its own
state and clock**. It receives no other drone's position and does not query the
ground station. The global observer is used only to evaluate and plot the run.
Synthetic wind is horizontal, deterministic and bounded; it is added to ground
velocity, not air-relative velocity.
Execution now defaults to own-motion wind estimation, local plan-switch
scheduling and sampled protection of the combined actuator request. Bounded
position bias/noise, constant wind and synthetic gust profiles can be enabled;
see [disturbed tracking and validation](robust_tracking.md). Nominal candidate
construction still uses the same dynamics and validation before joint selection.

Distances and error tubes are computed in XYZ. The Lipschitz correction between
samples conservatively uses the model's maximum speed, not just horizontal
speed. During execution, the wind norm increases the ground-speed bound.

**This is numerical validation under assumptions**, not a proof of collision
avoidance for real physical dynamics: the integrator has no formal error bound,
envelope checks are refined but sampled, and compliance with tracking tubes
under disturbances is checked only for the executed run. Any violation invalidates
the result without automatically reducing margins. The exit code is then 3.

The report includes `physical_limits_verified`, speed, bank and load-factor
extrema, stall margin and specific propulsive work `integral(V*a_T*dt)` in J/kg.
The latter is **neither** battery energy **nor** an endurance guarantee:
efficiency, capacity and mission-duration constraints are not modeled. Complete
outputs and parameters are exported using the `swarm-flight3d-v3` schema;
historical projections remain available for the GUI.

Not modeled: angle of attack/sideslip, control surfaces and 6-DoF inertial
dynamics, aerodynamic gust forces, terrain meshes, full sensor/estimator models, takeoff/landing,
failures and performance degradation with altitude. The initial state represents
an aircraft already in flight.
Known static cylinders and boxes are now checked by planning and execution;
see [obstacles](obstacles.md). This does not add contact physics, onboard
obstacle perception or reactive avoidance to the flight model.

## Running and testing

The [isolated vertical verification bench](vertical_flight_validation.md) checks
modest climb/descent and return-to-level fixtures at several total airspeeds,
independent of destinations and trajectory generation. It separates algebraic
consistency, held-force integration accuracy and command discretization effects;
it does not validate real-aircraft calibration or resolve geometric catalog limits.
The [expanded envelope experiments](flight_envelope_validation.md) cover
boundary classifications, bank/slope equilibria and additional speed/time-step
settings without modifying the flight model or relaxing its checks.

```bash
.venv/bin/python examples/trajectory_3d.py
.venv/bin/python examples/trajectory_3d.py --no-gui
# Compare with the historical model:
.venv/bin/python examples/trajectory_3d.py --flight-model legacy --descent-angle 0
# Disable the airbrake: the 30°/12 m/s case is rejected before generation.
.venv/bin/python examples/trajectory_3d.py --max-airbrake-acceleration 0
# Isolated physics checks, without a mission or GUI:
.venv/bin/python scripts/validate_vertical_flight.py --refine
./scripts/test.sh
```

Exact 2.5D regressions remain explicitly tested in legacy mode. Tests for the
new engine cover equilibria, speed relationships, wind, independent integration,
envelope violations, four aircraft with recovery, plan corruption, the absence
of inter-drone telemetry and numerical convergence.
