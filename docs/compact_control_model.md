# Experimental compact-aircraft moments and actuator chain

This extends the C++ force component with own-airframe aerodynamic moments,
angular-rate damping and measured joint inputs. It is separate from the
production point-mass planner. The compact native **flight qualification is
not complete**: cold runway attempts have exceeded the attitude acceptance
limits and remain failed trials. The established Cessna adapter is unchanged.

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

The integrated suite passed 1,114 tests (four existing Gymnasium float32
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

The next decision is whether to qualify the project's usual warm airborne
initialization first or require runway takeoff. Native reference integration
and multi-aircraft experiments remain gated on successful individual flight;
shared scenario schemas and the production selector have not been widened
around a failed qualification.
