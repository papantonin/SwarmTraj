# Modular fixed-wing aircraft profiles

The coupled 3D application accepts complete, versioned JSON airframe profiles.
`--aircraft-profile` applies one loaded profile to all aircraft in a simulation.
Explicit `--drone-profile ID JSON` assignments additionally support different
airframes in the same run. See the [heterogeneous-aircraft guide](heterogeneous_aircraft.md)
for four-aircraft examples, editor loading, geometry and the public Rascal case study.

This adds a data/configuration layer, not new aircraft equations or a hardware
integration interface. All supported profiles currently select the existing
normalized coordinated point-mass backend.

## Launching and visualizing

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
`--aircraft-profile` retains the original defaults and physical flags, including
legacy/2D support. Loaded profiles require coupled point-mass 3D mode; applying
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

Initial airspeed is still 12 m/s unless explicitly configured. It is **not**
automatically replaced if incompatible with a new profile. Individual requested
speeds remain allowed inside the common envelope. A smaller bank limit may
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
The current model does **not** directly consume mass, wing area, inertia,
control-surface geometry, battery capacity, meshes or URDF files. Adding those
keys does not add corresponding physics: they are rejected as unknown fields.
A new backend/schema would be needed to exploit them consistently.

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
model plugins, mass/geometry-based aerodynamics, unpowered gliders, calibrated
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
