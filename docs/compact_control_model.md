# Experimental compact-aircraft moments and actuator chain

This extends the C++ force component with own-airframe aerodynamic moments,
angular-rate damping and measured joint inputs. It is separate from the
production point-mass planner. The compact native **flight qualification is
not complete**. A dedicated warm airborne start now skips runway takeoff;
the earlier cold runway failures are retained. The established Cessna adapter
is unchanged.

## Data and coordinates

`examples/gazebo/compact_physics/control_model.json` uses the strict
`swarm-gazebo-control-model-v1` schema. It binds to the canonical physical/lift
pairing from `ForceModel`, rather than inferring inertias or control derivatives
from mass alone. It records span, mean chord, trim angle of attack, stability
and control derivatives, principal base inertias, servo travel/dynamics and
passive wheel radius/mass/damping. All values are illustrative. Unknown fields,
duplicate keys, booleans/nonfinite coefficients, invalid inertias and mismatched
physical bindings are rejected. A different aircraft needs its own record.

Body axes are X forward, Y left, Z up (FLU). Positive body Y rotation lowers the
nose; positive Z rotation turns left. With `qS = rho*V²*S/2`, body rates `p,q,r`
and normalized own controls `a,e,d`, the model uses:

```text
beta = atan2(vy, hypot(vx, vz))
Tx = qS*b * (roll_beta*beta - roll_damping*p*b/(2V) + roll_control*a)
Ty = qS*c * (pitch_stiffness*(alpha-alpha_trim)
             - pitch_damping*q*c/(2V) - pitch_control*e)
Tz = qS*b * (yaw_stiffness*beta - yaw_damping*r*b/(2V) - yaw_control*d)
Fy += -qS * sideforce * beta
```

Lift, polar drag and ideal braking retain the [force model](compact_force_model.md).
For this FLU beta definition, the illustrative restoring dihedral coefficient
is positive. Standard derivatives from other coordinate conventions cannot
be copied without conversion. Zero-flow aerodynamic torque is zero. These
linear derivatives and clipped CL are not a sideslip/post-stall calibration.

Joint mode reads only the own model's actual `servo_0`, `servo_1`, `servo_2`,
`rudder_joint` and `airbrake_joint` positions. Left/right ailerons are differenced;
elevator and rudder are scaled by their declared travel, and brake by its own
travel. Fractions saturate to [-1,1] or [0,1]. Missing joints fail configuration;
unavailable feedback has a bounded startup allowance and then fails explicitly.
After feedback/kinematics have first become valid, losing them fails immediately.
No observer or other aircraft state supplies controller commands.

The authored geometry remains within the 0.30 m sphere and totals 5 kg. With
the control model, base mass accounts explicitly for three passive 0.10 kg,
0.03 m radius wheel shapes, rather than adding mass to the requested total.
The base principal inertias are explicit assumptions, not the earlier solid
fuselage approximation. Geometry reference dimensions must agree with the
control model. Servo PID force, passive damping, effort and rate limits are
explicit; kinematic position bypass is disabled. Gazebo's implicit I/D defaults
must not be mistaken for the declared servo model; see its
[JointPositionController implementation](https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/src/systems/joint_position_controller/JointPositionController.cc).

## Commands

```bash
bash scripts/build_compact_gazebo_physics.sh

# Independent compiled moment signs, saturation and analytical trim checks.
.venv/bin/python scripts/qualify_control_model.py \
  --output results/compact-controls-algebra-NEW

# Constant-input free-body equilibria and small disturbances, no PX4.
.venv/bin/python scripts/qualify_gazebo_trim.py \
  --library build/compact-gazebo/libcompact_polar.so \
  --output results/compact-controls-trim-NEW

# Physical joint steps through Gazebo controllers, no PX4.
.venv/bin/python scripts/qualify_gazebo_actuators.py \
  --library build/compact-gazebo/libcompact_polar.so \
  --output results/compact-joint-steps-NEW

# Explicit controllable geometry export and load test.
.venv/bin/python scripts/build_compact_gazebo_prototype.py \
  --polar-library build/compact-gazebo/libcompact_polar.so \
  --control-model examples/gazebo/compact_physics/control_model.json \
  --output results/compact-control-prototype-NEW --check

# Unarmed PX4 identity/readiness only.
.venv/bin/python scripts/qualify_compact_px4_startup.py \
  --library build/compact-gazebo/libcompact_polar.so \
  --output results/compact-px4-startup-NEW

# Additionally drive the simulated servos/motor while unarmed and verify responses.
.venv/bin/python scripts/qualify_compact_px4_startup.py --actuator-test \
  --library build/compact-gazebo/libcompact_polar.so \
  --output results/compact-px4-actuators-NEW

# Experimental native circuit, initialized in flight: no runway takeoff.
.venv/bin/python scripts/qualify_compact_px4_startup.py --flight-test --airborne-start \
  --library build/compact-gazebo/libcompact_polar.so \
  --output results/compact-px4-airborne-NEW

# Visible cruise/turn qualification, stopping the simulator before landing.
.venv/bin/python scripts/qualify_compact_px4_startup.py \
  --flight-test --airborne-start --flight-phase cruise-turns --gui \
  --library build/compact-gazebo/libcompact_polar.so \
  --output results/compact-cruise-turns-NEW
```

A versioned C++ control API check runs in the pinned runtime before enabling
joint mode in the launcher. Missing/stale libraries fail with rebuild
instructions. The default force-only export remains available.

The new native experiment uses a derived **4999** airframe. It inherits the
pinned 4003 initialization, then sets the compact physical airspeed defaults
and `SIM_GZ_SV_MINA*` / `MAXA*` angle limits in degrees **before** the Gazebo
bridge initializes. The integer PWM `MIN*`/`MAX*` parameters are different.
The bridge caches the angle range during initialization; see the pinned
[PX4 implementation](https://raw.githubusercontent.com/PX4/PX4-Autopilot/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/simulation/gz_bridge/GZMixingInterfaceServo.cpp)
and [angle parameter declarations](https://raw.githubusercontent.com/PX4/PX4-Autopilot/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/simulation/gz_bridge/GZMixingInterfaceServo.hpp).
The derived file is mounted only into the fresh experimental container, with
source hashes and effective parameter readbacks retained. The original image,
4003 file, Cessna launchers, readiness and failsafe settings are unchanged.
The motor joint's negative rate is its inherited CW convention, not reverse
thrust. A 0.2 motor test produced approximately -20 rad/s at the slowed visual
joint; the nominal propeller coefficient still defines a 35 N maximum.

## Observed qualification and remaining gates

Independent C++ checks pass for per-axis damping, control signs and saturation.
Analytical equilibria at 100 km/h level and 175 km/h / -30 degrees balance
forces and moments within the declared tolerances. The dive requires about
14.839 N ideal brake force. Five 20 s free-body Gazebo fixtures pass: level,
dive, pitch perturbation, bank perturbation and yaw perturbation. Heading is
uncontrolled in the lateral free-body cases; their criterion concerns speed
norm and angular rates, not return to an inertial reference heading.

Measured joint-step checks and all six unarmed PX4 servo/motor response checks
pass after explicit native angle configuration. These are actuator-chain and
startup evidence, not closed-loop flight or tracking evidence.

Cold runway trials did not pass. Logs show ground-transition tipping and,
in some trials, takeoff followed by excessive bank. Variants included passive
rolling gear, changed wheel inertias, a 0.001 s contact step, coordinate-correct
lateral derivatives and conservative controller/runway settings. Earlier
failures remain under `results/control-model-development/`; they have not been
reclassified as successful negative tests. The benchmark now aborts on invalid
attitude even near the ground, verifies landing progress before completion,
and never commands an in-air disarm. Only the owned failed simulator is stopped.
These bounded failed configurations do not establish that the requested real
aircraft or every possible simulation model is physically infeasible.

Before the warm-start extension, the integrated suite passed 1,114 tests (four existing Gymnasium float32
warnings). Final rebuilt-plugin checks passed five trim/disturbance fixtures,
four measured-joint step checks and six unarmed native servo/motor checks.
Those passing checks remain distinct from the unsuccessful requested flights.

The optional `--flight-test` performs an **experimental** ordinary takeoff,
native circuit and required landing after identity, readiness, actuator and
native mission-feasibility gates. It has no forced-arm or external connection
interface. It is currently a failing qualification workflow, not a supported
mission launcher. Sensor-noisy airspeed remains in observations; a bounded,
clock-checked own-body velocity read provides zero-wind evaluation only and
never feeds back into PX4 guidance. No point-mass reference is used by this
circuit, and no timed tracking or separation claim is transferred.

## Compact warm airborne start

`--flight-test --airborne-start` selects a separate, explicit warm simulation
bench. The default startup remains unarmed, and `--flight-test` alone retains
the historical cold runway experiment. This is not a cold PX4 boot in flight,
a restored complete checkpoint or a production planner-to-Gazebo adapter.

The initial fixture is straight, level flight at world ENU `(0,0,80)` m,
100 km/h eastward and zero bank. Its body pitch is approximately +6.800 degrees
nose-up (negative Gazebo FLU Y angle), from the independent analytical trim;
pitch attitude is not the zero flight-path angle. The full physical/control
records, generated SDF, initial state and native mission have recorded hashes.
Changing those inputs invalidates release or acknowledgement.

The ordinary ground sensor/readiness checks, native mission feasibility and
normal arming run first. The valid native takeoff/landing records remain in
the derived mission, but execution begins at sequence 1, the cruise-speed
command, followed by the first forward waypoint. No runway takeoff is flown.
The compact bootstrap moves the own model once, clears ground contact for one
step, then applies one mass-proportional impulse to every own link. It never
holds a velocity or replays positions. All subsequent motion comes from the
plant and PX4's own sensor-based controller.

Physical initialization must pass the existing 0.5 m position, 0.5 m/s velocity,
2 degree attitude and 0.1 rad/s angular-rate bounds. A separate estimator gate
retains the 3 m nominal-height and 3 m/s per-horizontal-component tolerances,
fresh post-release receipts and a 3 s wall timeout. Receipt freshness does not
prove sensor source-time alignment. Mission mode starts only after that gate
and native airborne detection. The same documented one-time native
`fw_lat_lon_control` stop/start handles the pinned prerelease controller's
ground-to-air altitude reset; EKF and attitude/rate control remain running.
An immediate state match does not establish stable flight.

The warm bench retains the pinned 4 ms / 250 Hz physics cadence; the separate
cold contact bench retains 1 ms. A first 1 ms warm attempt matched physical
initialization but failed estimator acquisition and is retained as a failure
under `results/compact-airborne-development/first-warm/`. No tolerance was
widened to accept it. Artifacts include `airborne-config.json`,
`airborne-bootstrap-ready.json`, `airborne-release.json`,
`airborne-initial-state.json`, `airborne-check.json`,
`airborne-estimator-check.json` and observer-only `airborne-physics.csv`.

`--flight-phase circuit-landing` remains the default. Its first 4 ms warm trial
passed physical initialization (0.111 m position / 0.0393 m/s velocity error),
estimated-state acquisition (about 0.69 s simulation time), level cruise and
turns. It subsequently failed the original attitude criterion on ground contact
at landing, at approximately 295 s after release. That full trial remains
failed under `results/compact-airborne-development/sensor-cadence/`.

`--flight-phase cruise-turns` is an explicit, separate qualification phase, only
available with warm flight enabled. It requires reaching the end of the native
spatial circuit, at least five continuous seconds within 2 m/s of cruise at
60–100 m, at least 45 degrees of observed course change and at least 5 degrees
of observed bank. All original runtime attitude and speed checks remain in
force. The owned simulator then shuts down: there is no in-air disarm command,
no model deletion and no claim that the uploaded mission landed or completed.
Reports retain `native_mission_completed=false`, `flight_qualified=false` and
the selected phase. This phase does not turn the failed landing trial into a
successful full flight.

The explicit phase passed with the visible Gazebo client in
`results/compact-airborne-development/cruise-turns-gui/`: approximately 171.24 s
from release to phase completion and 168.61 s of continuous cruise hold. Over
that sampled interval, true airspeed was 25.24–29.55 m/s (90.85–106.38 km/h),
altitude 74.97–81.79 m and maximum true bank approximately 16.22 degrees.
These ranges include the warm-start transient; the speed target is 100 km/h,
not an exact speed constraint. Estimated-state acquisition took approximately
1.14 s. The observed course change reached 179.88 degrees. Own actuator checks,
initialization checks and the selected cruise/turn phase passed; the uploaded
native mission was deliberately not completed. Rendered-window screenshots
and `observed-phase-metrics.json` are retained with this ignored run directory.

`--gui` opens the real Gazebo client with software rendering and local
X11/XWayland authentication. It attaches the camera to the compact model once
before arming, with observed tracking acknowledgement; there is no forced
per-frame camera reset or aircraft-control path. Mouse control remains native.
Without `--gui` the same qualification runs headless. Use a fresh output
directory for every run. Preparing PX4 and checking the unarmed actuators still
takes time even though no runway takeoff is executed.

After the warm-start extension, the full suite passed 1,135 tests (the same
four Gymnasium precision warnings). Regression checks cover one-shot link
impulses, altered physical/native identities, invalid initial states, estimator
failure before Mission activation, the single native controller reset,
explicit phase criteria and own-target GUI tracking. GUI rendering was also
observed in the live native client. Mouse navigation was not independently
qualified by this smoke test.

General native reference integration and multi-aircraft experiments remain gated on
further individual flight trials. The high-speed dive and recovery still need
their own closed-loop trials; the bounded level spatial case is described below. Shared scenario schemas and
the production selector have not been widened around a failed qualification.

## Dive prerequisites and airbrake allocation

The compact derived airframe now declares five control surfaces. Its fifth
surface is native type 11 (Airbrake), with flap scaling zero and spoiler scaling
one; the unused sixth servo function is disabled. Previously it inherited a
left-flap declaration from the Cessna fixture. Native type and scaling semantics
are defined by the pinned PX4
[surface types](https://raw.githubusercontent.com/PX4/PX4-Autopilot/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessControlSurfaces.hpp)
and [allocator](https://raw.githubusercontent.com/PX4/PX4-Autopilot/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessControlSurfaces.cpp).
This corrects the declared allocation, not proof that native flight guidance
requests the needed brake fraction during a dive. Direct unarmed servo steps
remain distinct from allocator-driven in-flight braking.

After unarmed actuator checks, `dive-readiness.json` records actual native
parameter readbacks and an independent equilibrium calculation, bound to the
physical/control records. For the requested 175 km/h / -30 degree dive it
requires approximately 24.306 m/s sink, -28.848 degrees body pitch and 14.839 N
of ideal braking. The present native cruise settings remain 3.5 m/s sink and
-15 degrees minimum pitch. The diagnostic therefore reports incompatible
settings; it does not change parameters or arm the aircraft for a dive.

`settings_compatible` checks necessary settings only. Even when true,
`closed_loop_qualified` remains false: dive entry, speed/brake regulation and
pull-out still require measured native flight trials. The analytical trim is
shared by qualification tools through `gazebo_trim.py`; it is an independent
audit calculation, not a production dynamics backend or PX4 controller.

After this allocation correction, all six unarmed native actuator checks passed
again, and the headless warm cruise/turn phase passed again in
`results/compact-dive-development/brake-cruise-regression/`. The full suite
passed 1,141 tests with the four existing Gymnasium precision warnings.
This regression does not qualify a dive or recovery. The native cruise
parameters were not changed to hide the diagnostic's incompatible settings.

## Native brake-response bench

```bash
.venv/bin/python scripts/qualify_compact_px4_startup.py \
  --flight-test --airborne-start --flight-phase native-brake \
  --library build/compact-gazebo/libcompact_polar.so \
  --output results/compact-native-brake-NEW
```

This explicit bench uses the native early landing configuration, read back as
`FW_LND_EARLYCFG=1`, and `FW_SPOILERS_LND=0.4`. It waits for the native loiter
before landing, observes the own physical airbrake joint, and requires two
continuous seconds within 0.03 of the requested 0.4 fraction. Interrupted or
stale observations cannot establish a continuous response. The original
airspeed/attitude checks remain active. The owned simulator stops before ground
contact; this does not qualify a completed landing, dive or pull-out. No
external servo command or ground-truth control loop drives this response.

The pinned [mode manager](https://raw.githubusercontent.com/PX4/PX4-Autopilot/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/fw_mode_manager/FixedWingModeManager.cpp)
sets ordinary automatic mission spoilers to zero and deploys them in landing
configuration. The [fixed-wing allocator](https://raw.githubusercontent.com/PX4/PX4-Autopilot/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessFixedWing.cpp)
then applies that own native setpoint to the declared surfaces. This bench
tests that existing path, not automatic dive-speed braking.

The pinned [longitudinal parameter metadata](https://raw.githubusercontent.com/PX4/PX4-Autopilot/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/fw_lateral_longitudinal_control/fw_lat_long_params.yaml)
declares 15 m/s as the maximum supported `FW_T_SINK_MAX` setting. The requested
24.3 m/s sink exceeds that documented range. Setting a larger readback does not
qualify it. The readiness report now separates matching requested settings,
documented-range compatibility and native automatic waypoint-braking support;
`flight_trial_ready` remains false. Supporting the requested dive requires a
separately developed and tested native flight-control path, rather than simply
raising parameters in the established cruise bench.

The first native-brake attempt stopped before arming because early landing
configuration was disabled; that failed attempt remains in
`results/compact-dive-development/native-brake-first/`. With the explicit,
verified early-configuration setting, the bench passed in
`results/compact-dive-development/native-brake-early-config/`: the measured
brake fraction reached 0.400023 for a 0.4 request and held within tolerance
for 2.088 s. The run also recorded 169.60 s of continuous cruise hold. The
native controller and allocator generated the request; no actuator-test command
was sent during flight. The owned container was removed after copying native
logs. This is native brake-response evidence in landing configuration, not
evidence of automatic dive-speed control. The full regression suite passed
1,142 tests with four existing Gymnasium precision warnings.

## Generated SwarmTraj spatial reference

```bash
.venv/bin/python scripts/qualify_compact_px4_startup.py \
  --flight-test --airborne-start --flight-phase swarm-reference --gui \
  --library build/compact-gazebo/libcompact_polar.so \
  --output results/compact-swarm-reference-NEW
```

This first bounded adapter generates an actual single-aircraft C++ SwarmTraj
mission for the compact profile: straight level cruise from `(0,0,80)` m toward
`(1800,0,80)` m, with a 100 s horizon. The usual integrated planning and
validation run before native launch. The original full-horizon document is
saved as `point_mass_mission.json`; it is not truncated at target passage.
The adapter checks its fingerprint, physical identity and supported geometry.
Arbitrary imported plans, turns, obstacles, dives and fleets are not supported
by this initial compact reference mode.

The generated Gazebo world displays the actual reference in cyan up to arrival,
then muted blue for onward flight. A gold sphere shows the configured target
radius at its actual world coordinates, with a ground disk and vertical beacon.
The compact reference line has a 0.12 m display radius to keep the small aircraft
visible. These static annotations have no collisions and use visibility bit 1,
excluded by the fixture lidar mask. They do not change guidance, physical
geometry or validation. Previously generated worlds are unchanged: launch a
fresh run/output directory to obtain these visuals.

Derived native waypoints preserve sampled XYZ positions, with approximately
100 m spacing. Initial points behind the warm-start aircraft are skipped;
the takeoff record remains for native feasibility but is not executed.
Geographic conversion uses the pinned world origin, with absolute-altitude
MAVLink records for the world reference. The measured home antenna altitude
does not move the original target or turn ground altitude zero into an invalid
negative relative-altitude landing record. Non-position commands retain zero
coordinate fields. Native readiness, feasibility, ordinary arm and warm-start
gates remain active; the established Cessna adapter is unchanged.

After release, observer-only XYZ samples measure distance to the original
3D reference polyline and to the target center. The sampled spatial error must
remain within 10 m, and sampled target distance must enter the original 6 m
target radius. Those bounds are fixed, not fitted to observed outcomes. The
simulator shuts down after the last reference waypoint, before native landing.
Reports distinguish actual generated-reference use from circuit fixtures and
retain `native_mission_completed=false` and `flight_qualified=false`. This
spatial experiment transfers no point-mass timing, separation, obstacle or
between-sample safety certificate to PX4.

The first upload attempt failed before arming due to invalid altitude fields
and remains in `results/compact-reference-development/first-spatial/`. With
absolute reference altitudes and zero coordinates on non-position commands,
the headless generated-reference trial passed in
`results/compact-reference-development/absolute-altitude/`. Across 952 received
post-release ground-truth samples, including the warm-start transient, maximum
spatial error was 8.957 m, p95 was 7.584 m, and closest observed target distance
was 4.464 m. The fixed 10 m tracking threshold and original 6 m target radius
were retained. `independent-spatial-observations.json` records the separate
post-run calculation from all received samples, alongside the original plan,
native mission, telemetry and native logs. The owned simulator was removed
after completing the derived reference, without attempting landing.
The final full suite passed 1,150 tests, with four existing Gymnasium precision
warnings. Focused checks cover the real C++ reference, complete horizon,
fingerprints, unsupported-reference rejection and altitude-frame conversion.
