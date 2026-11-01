# Hypothetical compact Gazebo build prototype

This is the explicitly requested **hypothetical simulation prototype** for the
compact 5 kg aircraft. It is a separate build fixture, not a selectable
production airframe or a qualified PX4 flight adapter. Existing Cessna single
and four-aircraft launchers are unchanged.

```bash
.venv/bin/python scripts/build_compact_gazebo_prototype.py \
  --output results/compact-gazebo-prototype-NEW --check
```

The command reads the official Cessna sensor/plugin templates from the already
installed pinned PX4/Gazebo Docker image, with networking disabled. It creates
a fresh directory containing `model.sdf`, a standalone `load-check.sdf`,
`assumptions.json`, proposed but unapplied PX4 parameter values and the upstream
BSD license. `--check` validates the SDF and loads a standalone physics world for
250 steps (one simulated second). It starts no PX4 process, renders no GUI and
issues no aircraft flight commands. All mounts are read-only; no host ports or
flight-controller devices are exposed. Generated files remain under ignored
`results/` and are never implicitly installed in the production adapter.

An explicit experimental C++ force component is now available with
`--polar-library build/compact-gazebo/libcompact_polar.so`. See
[its build and independent force checks](compact_force_model.md). It replaces
the inherited LiftDrag forces and implements the profile polar and ideal brake;
static bench inputs still lack PX4 actuator binding and aerodynamic moments.
The default export described below retains the inherited force model.

The geometry is authored from primitive ellipsoids, with an elliptical 0.56 m
span, 0.12 m² wing. Component mass sums to exactly 5 kg, including the rotor,
control links, small fixed gear and lidar. Component radial bounds remain
within 0.30 m even during joint rotation. These are checks on the authored
simplified shapes, not a real aircraft mass distribution or equipment layout.
The solid-ellipsoid inertias are illustrative assumptions.

Sensors, four servo conventions and the rotor plugin follow the pinned
upstream fixture. The nominal static thrust coefficient is derived from 35 N
at the inherited 1,000 rad/s limit. Six LiftDrag components use assumed areas
and centers with inherited aerodynamic derivatives. They do **not** implement
the planner's `CD0 + k CL²` polar, measured stability derivatives or its ideal
20 N airbrake. Those gaps are recorded explicitly in `assumptions.json`;
`px4_flight_qualified` and `ideal_airbrake_implemented` remain false. Proposed
PX4 airspeed/climb/sink values are exported for review, never applied to an
instance or used to bypass readiness, arming, feasibility or landing checks.

## Verification

The pinned image accepted the generated SDF and completed the 250-step physics
load check. Its usual `gz_frame_id` extension warnings and isolated-network
loopback discovery messages are retained in logs. This checks parsing and
loading only: sensor readiness, trim, stability, airspeed tracking, dive
braking, recovery, takeoff/landing and swarm execution have not been tested.
No point-mass timing, tracking or separation claim transfers to this fixture.

The C++ force/drag and braking algebra and its application in Gazebo have passed
isolated comparisons. Next work is aerodynamic moments and actuator binding,
followed by longitudinal/lateral trim and control
responses, then introduce an explicit isolated PX4 adapter with parameter
readback and single-aircraft flight qualification. Only after those gates
should four independent PX4 instances be exercised. Shared scenario schemas
must persist phase-speed/routing/selection settings before these missions can
be exported; unsupported combinations currently fail explicitly.

See [the compact profile](compact_5kg.md), [the development record](trajectory_development.md)
and [the existing Cessna adapter](px4_gazebo.md).
