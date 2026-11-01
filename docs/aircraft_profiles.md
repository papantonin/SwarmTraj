# Modular fixed-wing aircraft profiles

The coupled 3D application accepts complete, versioned JSON airframe profiles.
`--aircraft-profile` applies one loaded profile to all aircraft in a simulation.
Explicit `--drone-profile ID JSON` assignments additionally support different
airframes in the same run. See the [heterogeneous-aircraft guide](heterogeneous_aircraft.md)
for four-aircraft examples, editor loading, geometry and the public Rascal case study.

This adds a data/configuration layer, not new aircraft equations or a hardware
integration interface. All supported profiles currently select the existing
normalized coordinated point-mass backend.

## Configure your own powered fixed-wing aircraft

Copy [custom_fixed_wing.json](../examples/aircraft/custom_fixed_wing.json) and
edit the complete JSON record. Schema `swarm-aircraft-profile-v3` accepts physical
SI inputs directly; the existing v1/v2 normalized profiles remain unchanged.

The [compact 5 kg example](compact_5kg.md) provides a separate hypothetical
profile with 100 km/h cruise, 175 km/h dive and a 0.30 m enclosing radius,
plus a companion 30-degree passage mission and explicit assumption provenance.

| `airframe` field | Meaning |
|---|---|
| `mass_kg` | Total flying mass, including payload and fuel/battery |
| `wing_area_m2` | Reference wing planform area |
| `air_density_kg_m3` | Constant density for this particular flight condition |
| `zero_lift_drag_coefficient` | Dimensionless CD0 of the parabolic drag polar |
| `induced_drag_factor` | Dimensionless k in CD = CD0 + k CL² |
| `max_lift_coefficient` | Positive CLmax used to derive the 1g stall speed |
| `max_thrust_n` | Positive constant available thrust in newtons |
| `max_airbrake_drag_n` | Additional available braking force; use 0 without an airbrake |

`limits` contains speed, load, bank/roll, flight-path, climb/descent and vertical
acceleration bounds. `geometry.bounding_radius_m` must enclose the whole aircraft.
All fields are required. Unknown fields and conflicting normalized coefficients
are rejected. In v3, do not supply `drag`, `actuators` or
`limits.stall_speed_1g_m_s`: they are derived from the physical inputs.

For mass m, wing area S and density rho, the conversion is:

```text
stall_speed_1g = sqrt(2 m g / (rho S CLmax))
c0 = rho S CD0 / (2 m)
ci = 2 k m g² / (rho S)
max_thrust_acceleration = max_thrust_n / m
max_airbrake_acceleration = max_airbrake_drag_n / m
```

These reproduce the existing drag law `D/m = c0 V² + ci n²/V²` with
`L = n m g`. Conversion occurs once during profile loading; C++ remains the
only production numerical backend. The original physical inputs are immutable
and included in the profile hash and exported own-aircraft records. Changing
mass requires recomputing the derived physics by reloading a new profile.

Try the supplied illustrative aircraft on a level mission first:

```bash
.venv/bin/python examples/trajectory_3d.py --no-gui --num-drones 1 \
  --aircraft-profile examples/aircraft/custom_fixed_wing.json \
  --altitude 80 --target 500 0 --target-altitude 80 --descent-angle 0 \
  --manual-cruise --horizon 120 --candidates 1 --attempts 8
```

Remove `--no-gui` to use the editor and playback. A selected aircraft can also
load the same file through **Profile JSON** / **Load profile**. Use
`--drone-profile ID PATH` for a complete per-ID fleet assignment.
The example has no airbrake and a 3 m/s descent limit, so the default 30-degree
descending mission at 12 m/s is intentionally rejected. Set mission speed,
loiter radius, target and arrival requirements explicitly for the chosen aircraft;
the loader never changes these to force a successful result.

This models powered aircraft admitting coordinated 1g flight and a constant
parabolic polar. It does not model inertia, control surfaces, sideslip, thrust
curves, fuel burn, changing density or post-stall flight. Loading this file does
not replace Gazebo's Cessna airframe, mesh, sensors or PX4 tuning: that is a
separate model integration. No arbitrary-aircraft fidelity or certification is
established by supplying a configuration file.

Physical v3 profiles can also drive the experimental
[C++ force qualification benches](compact_force_model.md#reusing-the-force-benches-for-another-aircraft).
Those benches read mass, polar and brake force from the selected profile, with
separate versioned lift assumptions. They do not create aircraft geometry or
qualify PX4 control. The compact geometry exporter remains specific to its
5 kg / 0.30 m prototype.

## Launching and visualizing

### Operating airspeeds

Physical v3 profiles can declare a complete `speeds` object:

```json
"speeds": {
  "loiter_airspeed_m_s": 10,
  "climb_airspeed_m_s": 11,
  "cruise_airspeed_m_s": 12,
  "descent_airspeed_m_s": 12,
  "dive_airspeed_m_s": 15
}
```

These are recommended **total airspeeds**, not horizontal or vertical speeds.
All five values must be positive, finite, inside the declared speed envelope
and above the 1g stall margin. Dive speed must be at least cruise speed.
The example values are illustrative. `dive_airspeed_m_s` is an operational
setpoint, not a structural design-dive-speed certificate. The existing
`limits.max_airspeed_m_s` remains the hard model limit for every regime.
`max_descent_rate_m_s` separately bounds downward vertical speed.

With no explicit speed argument, the CLI uses the profile's cruise airspeed
for each aircraft. `--flight-regime loiter|climb|cruise|descent|dive` selects a
different declared speed. For example, add `--flight-regime dive` to the level
demonstration above to request 15 m/s. This selects the mission-wide speed;
it does not command a dive angle. Add `--phase-speeds` for an integrated own
operating-speed schedule (loiter, climb/descent to cruise, cruise, selected
terminal regime, deceleration, recovery and cruise). With that option,
`--flight-regime` selects only `cruise`, `descent` or `dive` at passage. Explicit
`--airspeed`/`--drone-speed` conflicts are rejected. The reference slew rate is
`--speed-profile-acceleration`; physical controls and limits still determine
actual acceleration. Cruise must be attained for at least one second before
arrival, terminal speed must be attained and present at reintegrated contact,
and final cruise speed must be recovered (0.2 m/s numerical tolerance).
Independent replay and execution observers check actual airspeeds. Only strict
single-level profiles support this schedule; unsupported overlays and lossy
shared PX4 exports fail explicitly. Older constant-speed missions are unchanged.

`--airspeed` or `--drone-speed` explicitly overrides the default cruise request
within the original physical limits. Combining either with `--flight-regime`
is rejected. Older profiles without `speeds` keep the 12 m/s default and reject
explicit regime selection. A v3 record without this optional object keeps its
original meaning and hash; a partially specified object is rejected.

Library callers can use `profile.airspeed_for('cruise')` (or another regime)
when constructing their initial snapshot and mission request. The editor keeps
existing speed settings when loading a new profile; it does not silently change
the fixed initial snapshot. Turn/load-dependent stall, thrust/braking authority,
full-horizon validation and execution checks still apply after speed selection.

```bash
# Existing editor and 3D visualization, using the historical reference airframe.
.venv/bin/python examples/trajectory_3d.py \
  --aircraft-profile examples/aircraft/fixed_wing_reference.json

# Same application without windows.
.venv/bin/python examples/trajectory_3d.py --no-gui \
  --aircraft-profile examples/aircraft/fixed_wing_reference.json

# Isolated laboratory validation with the lower-authority, no-airbrake example.
.venv/bin/python scripts/validate_vertical_flight.py \
  --aircraft-profile examples/aircraft/fixed_wing_training.json \
  --speeds 10 13 --cases level climb_recovery --refine
```

The reference file reproduces the historical CLI's physical values. Omitting
`--aircraft-profile` retains the original physical defaults and flags.
Production requires coupled point-mass 3D mode; applying
them to legacy or planar dynamics is rejected.

The original reference/training files are **illustrative, not calibrated aircraft descriptions**:

| Profile | Examples of distinguishing parameters | Scope |
|---|---|---|
| `fixed_wing_reference.json` | 8–16 m/s, bank 50°, illustrative airbrake 5 m/s² | Historical reference values |
| `fixed_wing_training.json` | 9–15 m/s, bank 35°, lower roll/vertical authority, no airbrake | Demonstrates different capabilities and explicit refusals |

The training profile cannot satisfy the application's default 30-degree
descending passage at 12 m/s: its downward vertical-speed limit is 4 m/s.
The default mission is therefore rejected, not modified to fit the airframe.
Its absence of an airbrake also causes the laboratory descending fixture to
fail actuator-envelope validation. Successful level/climb checks do not imply
that all original missions can be flown by this variant.

## Separation of responsibilities

`AircraftProfile` contains immutable airframe data, `FlightLimits` and `Limits`
hold the existing model parameters, and `PerformanceLimits` holds vertical-rate
and acceleration bounds. `AircraftProfile.apply(config)` binds these to the
existing `Config` while leaving mission/environment settings unchanged.

Airframe parameters include airspeed range, stall/load envelope, bank/roll and
flight-path bounds, thrust/airbrake authority, drag coefficients, climb/descent
rates and vertical acceleration.

Mission/environment parameters remain outside the file: initial states and
requested airspeeds, holding-circle geometry, destinations, altitude/geofence
bounds, horizon/deadline, obstacles, separation/tracking/clock margins and
search settings. A profile is not permission to relax any of these requirements.

With a loaded profile, existing physical CLI flags such as `--max-bank`,
`--stall-speed`, `--max-climb-rate` or `--max-airbrake-acceleration` are rejected,
even if their explicit values equal the profile. This prevents ambiguous
precedence and a profile hash describing different effective physics. Edit a
copy of the profile instead. Mission flags such as `--airspeed`, `--loiter-radius`,
`--max-altitude` and `--horizon` remain usable under the profile's hard limits.

Without declared operating speeds, initial airspeed remains 12 m/s unless
explicitly configured. With them, a fresh CLI mission defaults to cruise speed
as described above. An incompatible explicit request is never replaced.
Individual requested speeds remain allowed inside the common envelope. A smaller bank limit may
also invalidate the chosen holding radius; that radius is not enlarged silently.
The existing editor displays and checks the loaded common limits.

Passing `--aircraft-profile` more than once is rejected, including repeated
identical paths. Use `--drone-profile ID JSON` for a complete per-ID assignment
instead; do not combine common and per-ID flags. There is no last-profile-wins
or implicit missing-ID fallback behavior.

## Complete file contract

Copy [the reference profile](../examples/aircraft/fixed_wing_reference.json)
or [the training variant](../examples/aircraft/fixed_wing_training.json) to create
a new profile. All sections and fields are required; there are no hidden defaults
inside a loaded file.

The root fields are:

The following contract describes the unchanged v1/v2 normalized formats; the
physical v3 alternative is described above.

- `schema`: `swarm-aircraft-profile-v1`, or `swarm-aircraft-profile-v2` with an
  additional required `geometry: {"bounding_radius_m": positive_number}` object;
- `model`: exactly `coordinated_point_mass_v1`;
- `unit_system`: exactly `SI`;
- `name` and `description`: nonempty identification/provenance text;
- `calibration_status`: `illustrative`, `user_supplied` or `measured`;
- `limits`, `actuators` and `drag`: complete numeric objects as in the examples.

Calibration status is a **user declaration**, not verified measurement provenance
or certification. The loader never imports code, evaluates expressions, contacts
services or chooses a plugin based on file contents.

Numeric names specify units. `stall_margin` and load factors are dimensionless;
load factor is lift divided by weight. Angles/rates in files use degrees and
degrees/s, while internal state angles use radians. The drag law remains:

```text
drag/m = c0 V² + ci n²/V²
c0: parasitic_coefficient_per_m           [1/m]
ci: induced_coefficient_m3_per_s4         [m³/s⁴]
```

Thrust and additional airbrake drag are accelerations, already divided by mass.
These v1/v2 formats do not accept mass or wing area; use v3 to derive the
normalized physics from those quantities. Inertia, control-surface geometry,
battery capacity, meshes and URDF files remain unsupported in every profile.
Unused keys are rejected, rather than silently ignored.

Validation rejects missing/unknown fields, duplicate keys, booleans/strings in
numeric fields, non-finite/overflowing numbers, invalid bounds, unsupported
models/units and oversized files above 1 MiB. This backend requires support for
1g level flight somewhere in the declared speed/load envelope. These input
checks are not a proof that propulsion, every initial state or a mission is
feasible; normal preflight/integrated validation still applies.

V1 profiles do not declare aircraft size; their point-aircraft radius remains
zero. V2 can declare an enclosing sphere, added to pair, obstacle and reservation
clearances as detailed in the [geometry contract](heterogeneous_aircraft.md).
The radius must bound the whole aircraft about its modeled position, not only
half the wingspan; it is not validated automatically against a mesh.
Altitude-dependent performance and real-aircraft calibration remain unmodeled.

## Library use and model binding

```python
from swarm_sim.trajectory2d import Config, load_aircraft_profile

profile = load_aircraft_profile("examples/aircraft/fixed_wing_reference.json")
config = profile.apply(Config(dimensions=3))
assert config.flight == profile.flight
print(profile.name, profile.sha256)
```

The profile is loaded once into frozen parameter objects. Later changes to the
file do not alter an already configured simulation. `to_dict()` and `record()`
return detached copies. Mission generation, numerical validation, local
execution and GUI checks all receive the resulting common configuration; no
validated trajectory is modified afterward.

`mission_document(..., aircraft_profile=profile)` verifies that the effective
physical configuration still matches the declared profile. Changing physical
parameters after binding and then exporting the old profile identity is refused.

## Results and reproducibility

When a profile is selected, `mission.json` includes `aircraft_profile`, containing
the complete effective normalized profile and its canonical SHA-256. This field
is added **before** computing the mission hash. Planning/execution reports and
each individual aircraft payload also retain the same profile record.

Individual payloads still contain only their own plan plus shared configuration
and airframe metadata, never another aircraft's plan/state. Rejected missions
retain the profile in their planning artifacts and do not export executable
individual plans.

Profile identity hashes canonical content, not path, whitespace or JSON field
order. Numbers are normalized to finite floats. Identity includes name,
description, calibration declaration, model and physical values. The digest is
not an authentication mechanism or proof of aircraft calibration.

Without a selected profile, the optional metadata is omitted and the original
configuration/serialization path remains available. Explicit profile records
legitimately change whole-mission hashes; numeric normalization can also change
JSON number spelling relative to historical CLI parameters. Equal flight-state
arrays must not be confused with byte-identical serialized manifests.

## Laboratory tools and remaining limits

The vertical bench accepts `--aircraft-profile` and uses its limits for input
speeds and physical checks. Default test speeds remain 10/11/12/13 m/s; supply
appropriate `--speeds` if those lie outside the new range. The first requested
speed supplies the negative-check baseline, which must itself admit level
flight; otherwise the test setup fails explicitly rather than validating
rejections against an already invalid baseline.

The expanded instantaneous boundary/equilibrium bench now also accepts
`--aircraft-profile`. This selects parameter-relative inequality probes and
speed/bank/slope grids; the no-profile mode retains its historical reference
fixtures. Other coupled exclusions remain visible. Neither bench establishes
full boundary reachability or real-aircraft calibration.

The editor can load a local JSON path for the selected aircraft and display its
limits before planning. Per-ID parameters are bound through generation,
execution and validation, not only stored as paths. Not implemented: arbitrary
model plugins, 6-DoF geometry/inertia-based aerodynamics, unpowered gliders, calibrated
performance curves, 6-DoF dynamics, direct JSBSim execution or onboard deployment.

## Initial common-profile stage: historical verification

The focused loader, CLI and laboratory checks passed 176 tests before the full
regression run. They exercise strict/missing/unused fields, malformed/duplicate
JSON, finite numeric types, immutable snapshots and canonical hashing, supported
backends, refusal of physical overrides and multiple profiles, parameter binding,
mission hashes and individual payload isolation. Real laboratory tests also use
a nonhistorical 20–30 m/s declared speed range to detect hidden 8–16 m/s input
assumptions; that is a synthetic fixture, not calibrated aircraft data.

The complete repository suite subsequently passed **636 tests** in 687.45 s,
with four existing Gymnasium float32-bound warnings. This includes the existing
planning, spawn, serialization, execution, editor and separate backend regressions.

Additional headless checks retained these artifacts:

- `results/trajectory3d/20261027-155450-643214/`: reference-profile four-aircraft
  planning and nominal execution passed every recorded acceptance criterion.
  Full profile records, mission hashes and each own-plan payload were checked.
- `results/trajectory3d/20261027-160312-358736/`: no-profile, one-worker repeat;
  exact flight states/controls and catalog/search results match the explicit
  reference-profile run with automatic workers. Manifest hashes differ as
  expected from profile metadata and configuration serialization, not geometry.
- `results/trajectory3d/20261027-155622-875060/`: the training profile refused
  the incompatible default descending mission, retained profile identity and
  exported no individual executable payloads.
- `results/benchmarks/vertical-flight-20261027-155449-438179.json`: training
  profile at 10/13 m/s, level and climbing fixtures, two command resolutions;
  all eight positive trials and twelve expected negative checks passed.
- `results/benchmarks/vertical-flight-20261027-160002-903081.json`: the same
  profile's descending fixture was rejected for the missing braking authority;
  the original signal and physical limits were retained.

That initial stage did not change GUI widgets; its GUI-related tests were
headless. Subsequent per-ID/editor development and current recorded experiments
are described in [heterogeneous aircraft](heterogeneous_aircraft.md). These are
numerical, model-conditional experiments, not aircraft certification or a guarantee
that arbitrary files describe real flyable vehicles.
