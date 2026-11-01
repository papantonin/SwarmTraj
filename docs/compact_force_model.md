# Experimental compact-aircraft force model

This is the first force-model development step for the hypothetical compact
5 kg aircraft. It is a standalone C++ Gazebo physics component, separate from
the production point-mass planner and the existing Cessna adapter. It does not
yet provide a PX4-controlled aircraft.

## Model and limits

The new component replaces all six inherited LiftDrag components when explicitly
selected. Keeping both would double-count aerodynamic forces. It uses the loaded
profile's density, wing area, zero-lift drag, induced-drag factor, maximum lift
coefficient and maximum brake force:

```text
alpha = atan2(-body_velocity_z, body_velocity_x)
CL = clamp(4.75 * (alpha + 0.06) + elevator_fraction, -CLmax, CLmax)
L = 0.5 * rho * V² * S * CL
D = 0.5 * rho * V² * S * (CD0 + k * CL²)
B = max_brake_force * brake_fraction
```

Body axes are X forward, Y left and Z up. Drag and braking oppose velocity;
lift follows body-up projected perpendicular to velocity. Force is rotated
into world coordinates before application to the own base link. The fixed
20 N maximum brake deliberately matches the planner's **ideal** constant-force
assumption; it is not a calibrated deployed airbrake. At zero flow the force
is zero. Invalid fractions/nonfinite inputs fail explicitly.

For coordinated flight, choosing the same load factor yields the same lift
and drag algebra as the planner's normalized polar. Flight-path angle is not
angle of attack. The lift slope, zero-lift angle and elevator effectiveness
remain hypothetical. CL clipping is not a post-stall model; reverse, vertical
and sideslip flow are not qualified. Aerodynamic moments, angular-rate damping,
PX4 actuator binding and wind are not implemented. Static bench fractions
therefore cannot establish trim, stability or closed-loop flight behavior.

## Build and checks

```bash
# Downloads compiler packages into a separate builder image on the first build.
# The original pinned PX4/Gazebo runtime image is unchanged.
bash scripts/build_compact_gazebo_physics.sh

# C++ force algebra against independent analytical/profile equations.
.venv/bin/python scripts/qualify_compact_forces.py \
  --output results/compact-force-algebra-NEW

# Actual Gazebo free-body force/acceleration experiment, no PX4 or gravity.
.venv/bin/python scripts/qualify_compact_gazebo_forces.py \
  --library build/compact-gazebo/libcompact_polar.so \
  --output results/compact-force-gazebo-NEW

# Export/load the full geometry with the experimental force component.
.venv/bin/python scripts/build_compact_gazebo_prototype.py \
  --polar-library build/compact-gazebo/libcompact_polar.so \
  --output results/compact-polar-prototype-NEW --check
```

The builder records compiler version and its image identity. Its compiler
packages come from the distribution repositories, so rebuilding is not claimed
to be byte reproducible. Generated missions record source/plugin hashes.
The qualification subprocesses are network-isolated and expose no host ports
or flight-controller devices. The free-body fixture applies a single starting
impulse; it never holds speed or pose afterwards. CSV telemetry is evaluated
against force/mass at every aerodynamic sample. Missing/truncated output or
failed criteria remain failures, and fresh output directories preserve earlier
failed runs. This bench is not the existing PX4 airborne-start bootstrap.

The default exporter retains its earlier inherited model. The new force model
requires an explicit library path and remains unavailable in production PX4
launchers. `ideal_airbrake_implemented=true` describes the force component;
`px4_flight_qualified=false` and the missing actuator binding remain explicit.
Next work is aerodynamic moments and actuator responses, then trim/control
qualification, an isolated native adapter, and single-aircraft flights before
fleet experiments. No timing or separation validation transfers from the
point-mass model.

## Observed checks

The analytical campaign passed 24 combinations of four speeds (25.5 m/s,
100, 175 and 200 km/h), level/30-degree descent loads and three brake fractions.
The C++ polar agrees with the independently derived planner drag to 1e-10 N
under those manufactured coordinated loads. Additional regressions cover
non-axial dissipative braking, zero flow and invalid commands.

Four actual Gazebo fixtures passed: drag, full braking, lift and a rotated
body frame. At 100 km/h with zero CL, initial drag was 2.552083 N without the
brake and 22.552083 N with it. Zero CL is an isolated force fixture, not level
flight; level trim needs lift and induced drag. The maximum force-balance
residual across the four fixtures was below 9e-12 N. Each fixture used 100 steps
of 0.001 s, zero gravity, one free 5 kg body, and a single starting impulse.
These are discrete numerical checks, not an integration-error certificate.
The full suite passed 1,098 tests with four existing Gymnasium warnings.
Earlier filesystem permission and initialization failures remain under
`results/compact-physics-development/` together with successful runs.

## Reusing the force benches for another aircraft

The analytical and Gazebo free-body benches now accept `--aircraft-profile`.
They bind physical v3 mass, density, wing area, polar, lift limit and brake force
from the selected immutable profile. They no longer assume 5 kg, 20 N or the
compact aircraft's operating speeds. Normalized v1/v2 profiles are rejected:
those records do not identify a unique physical mass and wing area.

```bash
# An existing 2.5 kg example with no airbrake.
.venv/bin/python scripts/qualify_compact_forces.py \
  --aircraft-profile examples/aircraft/custom_fixed_wing.json \
  --force-assumptions examples/gazebo/compact_physics/force_assumptions.json \
  --output results/custom-force-algebra-NEW

.venv/bin/python scripts/qualify_compact_gazebo_forces.py \
  --aircraft-profile examples/aircraft/custom_fixed_wing.json \
  --force-assumptions examples/gazebo/compact_physics/force_assumptions.json \
  --library build/compact-gazebo/libcompact_polar.so \
  --output results/custom-force-gazebo-NEW
```

The historical script and binary names remain compatible; the force component
is reusable. The sphere in the free-body bench is a manufactured numerical
fixture, not a generated aircraft collision mesh. Its mass is the selected
profile mass and its inertia is an explicitly manufactured solid sphere.

Lift slope and zero-lift angle are separate from the authoritative physical
profile. The optional `--force-assumptions` file uses this strict schema:

```json
{
  "schema": "swarm-gazebo-force-assumptions-v1",
  "lift_slope_per_rad": 4.75,
  "zero_lift_angle_rad": -0.06,
  "calibration_status": "illustrative"
}
```

Omitting that file retains the earlier illustrative values. Unknown/duplicate
fields, boolean/nonfinite numeric values and unsupported schemas are rejected.
A calibration label records provenance, not flight certification. Profile and
assumption records each have a canonical hash, and a combined binding hash
identifies the exact pairing. No profile fields are overwritten by lift
assumptions. Those same values are passed to the compiled C++ probe and the
Gazebo component, including when exported with the compact geometry.

Analytical test speeds use declared loiter/cruise/dive and maximum airspeed,
deduplicated in that order. Physical v3 profiles without operating speeds use
four evenly spaced values from the greater of the minimum airspeed and 1g
stall margin to the maximum. Gazebo uses declared cruise, or the midpoint of
that range when no cruise is declared. These are explicit force fixtures, not
mission speed choices. Requests outside the lift/forward-flow domain are
retained as failed trials, never clipped into a passing expectation.

The generalization was exercised with the existing 5 kg and 2.5 kg profiles,
including zero brake force and a separate 5.6/rad, -0.03 rad lift assumption.
All 12 actual Gazebo fixtures (three configurations, four fixtures each) passed;
the largest observed force-balance residual was below 1.7e-11 N. The analytical
regression runs both profiles with the altered lift assumptions and retains
deliberately unsupported lift requests as failures.
The integrated suite passed 1,109 tests, with four existing Gymnasium warnings.
Evidence is retained under `results/generic-force-development/`.

This does not generalize the authored compact geometry or PX4 tuning. The full
prototype exporter still enforces its 5 kg / 0.30 m restriction. Other aircraft
require their own geometry/inertia and actuator definitions before any native
trim or flight qualification. Aerodynamic moments and PX4 actuator binding
remain the next development steps.
