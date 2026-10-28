# Heterogeneous fixed-wing aircraft

The coupled 3D simulator can now bind a different immutable JSON airframe to
each aircraft. This is a configuration and validation extension of the existing
coordinated point-mass model, **not** a JSBSim/FlightGear or 6-DoF integration.

The initial scope remains four aircraft. Per-ID parameters reach candidate
generation, physical rollout, independent individual validation, execution,
incoming/outgoing actuator-switch checks and observer diagnostics. A controller
receives only its own effective physical configuration and plan. Fleet bindings
remain on the offline planning/evaluation side.

## Four-aircraft example

This deliberately mild, level-transit example uses the reference, training and
two approximate Rascal variants. Each aircraft keeps its own altitude; this is
an explicit XY destination scenario, **not** a common-XYZ arrival demonstration.
Larger holding circles and a 120 s horizon are explicit scenario settings, not
automatic adjustments to make a restrictive aircraft pass validation.

```bash
.venv/bin/python examples/trajectory_3d.py \
  --arrival-mode layers --descent-angle 0 --manual-cruise \
  --loiter-radius 60 --loiter-spacing 160 --horizon 120 \
  --candidates 4 --attempts 40 --selection-policy first \
  --conflict-guidance none --workers 2 \
  --drone-profile 0 examples/aircraft/fixed_wing_reference.json \
  --drone-profile 1 examples/aircraft/fixed_wing_training.json \
  --drone-profile 2 examples/aircraft/rascal110_empty_approx.json \
  --drone-profile 3 examples/aircraft/rascal110_fuelled_approx.json
```

Add `--no-gui` for the recorded headless experiment. Omitting per-ID assignments
still supports one common `--aircraft-profile`; omitting both retains the
historical physical CLI options. Do not mix common/per-ID profile flags or
physical overrides. With `--drone-profile`, supply **exactly one** assignment for
every ID, including repeated use of a profile on different aircraft. Missing,
duplicate, extra, negative and non-integer IDs are rejected, not filled in using
an implicit reference airframe. The library also requires exact membership.

Initial speed, holding radius, destination, world altitude limits, obstacles,
clock/tracking bounds and deadlines remain mission/environment inputs. No speed,
radius, descent requirement or margin is silently changed. A shared smooth-speed
permission range must fit **every** assigned airframe; it is not clipped.

The library equivalent uses immutable `(ID, AircraftProfile)` tuples:

```python
from swarm_sim.trajectory2d import Config, load_aircraft_profile

reference = load_aircraft_profile('examples/aircraft/fixed_wing_reference.json')
training = load_aircraft_profile('examples/aircraft/fixed_wing_training.json')
config = Config(dimensions=3, aircraft_profiles=(
    (0, reference), (1, training), (2, reference), (3, training)))
own_config = config.for_drone(1)
assert own_config.flight.max_brake_acceleration == 0
assert own_config.aircraft_profiles == ()
```

## Editor and playback

Select a drone, enter a local JSON path in **Profile JSON**, then click
**Load profile**. The editor shows the selected aircraft's
airspeed bounds, bank limit, enclosing radius and vertical-rate limits. The
loaded profile must admit the current initial state/holding circle; a failed
load leaves the previous configuration and snapshot unchanged. Change speed or
holding geometry explicitly before trying a more restrictive profile.

Loading the first per-ID profile snapshots the other aircraft's existing common
physics into an explicitly named `configured_common_airframe` profile. It does
not reset the other aircraft to factory defaults. Its metadata is a new
configuration snapshot, not preservation of the original file's identity.
Selecting the destination disables profile loading. **Reset** restores
the original configuration; profiles cannot change during execution.

Playback uses per-aircraft speed-bound lines, not one misleading common speed
band. The pair-distance plot shows center distances and a line for the largest
pair's center-distance requirement. That line is an upper reference across
pairs; actual acceptance uses the correct requirement separately for every pair.
Aircraft markers remain schematic, not scaled 3D meshes.

## Enclosing spheres and safety margins

Schema `swarm-aircraft-profile-v1` remains unchanged and does not declare size.
Its radius is zero, preserving the historical point-aircraft calculation; that
does **not** establish that a real airframe fits inside a point. To include size,
use schema `swarm-aircraft-profile-v2` and this additional required section:

```json
"geometry": {"bounding_radius_m": 2.5}
```

All existing fields remain mandatory. The radius must be a positive finite
number enclosing the entire aircraft about its modeled position, at every
attitude. Half the wingspan alone need not enclose a fuselage or tail. This is
a user/model assumption, not an automatically measured bound.

Let `r_i` be the sphere radius, `V_i` the aircraft's declared maximum total
airspeed, `e` the position-error bound and `tau` the clock-error bound. For
coupled flight, planning requires:

```text
sampled_min_center_distance - (V_i + V_j) dt/2
    >= separation + r_i + r_j + 2e + (V_i + V_j) tau
```

`separation` is retained as an additional surface-clearance requirement when
positive radii are present. This is more conservative than treating it as
already including physical size. Reservations inflate each aircraft by its own
radius in addition to the existing half-separation, error, clock and motion
allowances. Obstacle checks require:

```text
min_signed_distance_of_center_to_obstacle - V_i dt/2
    >= obstacle_clearance + r_i + e + V_i tau
```

Execution uses individual speed bounds plus the disturbance bound, includes
each radius in pair/obstacle acceptance, and reports both center and surface
lower bounds. A violated speed bound invalidates the corresponding separation
claim. The declared world altitude range still constrains the modeled position,
not a terrain-contact mesh or a shrunken sphere-containing geofence.

The usual limitations apply: finite catalogs are incomplete, numerical RK4 has
no certified integration-error bound, sampled envelope checks are not a formal
continuous-time proof, and execution tubes are verified for the actual simulated
run rather than guaranteed for all possible perturbations.

## Internet-sourced Rascal case study

The [SIG manufacturer page](https://sigmfg.com/products/sig-rascal-110egarf)
publishes dimensions and a flying-weight range for the Rascal 110 EG. It does
not supply all of our aerodynamic, actuator or structural limits. In particular,
the product description is not calibration data for this reduced model.

The public [ThunderFly FlightGear-Rascal model, pinned revision](https://github.com/ThunderFly-aerospace/FlightGear-Rascal/blob/add3958a81a7e9676f83859b8bf564377ecbbe03/Rascal110-JSBSim.xml)
provides the numerical case-study inputs used here: wing area 10.57 ft², empty
weight 13 lb and fuel capacity 1.5 lb. It also defines an angle-of-attack-dependent
drag contribution whose zero-angle value is 0.028, an induced coefficient 0.04
and a maximum tabulated lift coefficient 1.4. These are **simulation-model
parameters**, not verified real-aircraft measurements. No upstream XML, engine,
mesh or executable code is imported or run by this project.

For the two local approximate profiles, assume fixed density `rho=1.225 kg/m³`,
`S=10.57*0.09290304 m²`, and masses `13*0.45359237 kg` or
`14.5*0.45359237 kg`. From the parabolic polar `CD=CD0+k CL²`, using coordinated
lift `L=nmg`, the simplified coefficients are:

```text
c0 = rho S CD0 / (2m)
ci = 2 k m g² / (rho S)
Vs_1g = sqrt(2 m g / (rho S CLmax))
```

This holds the zero-angle drag contribution constant and omits the upstream
angle-of-attack, sideslip, control-surface and engine dynamics. It is not a
general JSBSim conversion or a fit of its full polar. The approximate stall
speeds are 8.288 and 8.754 m/s; they are analytical estimates from those
assumptions, not observed stall tests. The selected 10–20 m/s operating interval,
30° bank and 30°/s roll-rate bounds, load/path/vertical bounds and 2.5 m sphere are **illustrative
choices**, not source-certified capabilities. The fuelled model's nominal
minimum speed need not admit 1g flight with the added stall margin: preflight
and envelope validation still reject such states.

Empty-model thrust acceleration is assumed to be 3 m/s². The fuelled variant
uses the same assumed absolute thrust, divided by the larger mass. Neither
has an assumed airbrake; fuel burn, endurance and altitude-dependent performance
are absent. Both declare `calibration_status="illustrative"`. Neither should be
used as an aircraft performance specification. Assuming thrust at the XML's
empty-weight mass is a laboratory choice, not a claim that a real gasoline
aircraft can operate with an empty fuel tank.

## Profile-aware laboratory checks

Both isolated tools accept one profile per run:

```bash
.venv/bin/python scripts/validate_flight_envelope.py \
  --aircraft-profile examples/aircraft/rascal110_empty_approx.json

.venv/bin/python scripts/validate_vertical_flight.py \
  --aircraft-profile examples/aircraft/rascal110_fuelled_approx.json \
  --speeds 12 13 --cases level climb_recovery --refine
```

For the expanded envelope bench, an explicit profile selects speed samples
across its own interval, and bank/slope grids spanning its declared limits.
It also selects parameter-relative boundary probes for all 17 inequalities.
The unchanged no-profile mode retains the historical reference fixtures.
Other simultaneous violations are retained: an individual boundary probe is
not necessarily a flyable state, particularly for coupled or unreachable bounds.
The separate scalar oracle, analytical steady-turn comparison and unchanged
numerical tolerances remain in use. Explicit speeds outside the profile are
rejected before output creation.

`--dynamic` still requests the original ±8° laboratory pulse; it does not shrink
the test to fit the airframe. In the no-airbrake Rascal approximation, the
descending fixture at 12 m/s fails. That is retained as a failed requested trial
(exit 2), not relabeled as a successful negative test. Accepted static grid
points and successful mild climbs do not establish support for arbitrary maneuvers.

## Reproducibility and artifacts

Fleet assignments are sorted by ID. Mission artifacts include complete per-ID
profile records and their canonical digests before the mission hash. Each plan
records its **own** profile. Export and execution reject inconsistent assignment
metadata. A local drone payload contains its own effective configuration, an
empty fleet-binding tuple/list, its own plan/profile and optional own reservations;
it does not include another aircraft's profile, plan or snapshot.

The common-profile path remains available. Added configuration fields legitimately
change mission hashes even when physical trajectories are unchanged; v1 profile
hashes themselves remain unchanged. Wall-clock timings stay outside mission hashes.

Recorded current-development experiments:

- `results/trajectory3d/20261027-163229-712035/`: the four-profile level-transit
  command above, without GUI. Planning and nominal execution passed; all four
  aircraft arrived, no protection failures occurred, and the observer's minimum
  surface-distance lower bound was 44.94 m against an 8 m requirement. Mission
  hashing, per-ID physics and own-only payload contents were checked separately.
- `results/trajectory3d/20261027-163644-052032/`: the same four-profile scenario
  and **identical plan dictionaries**, with 0.05 m/s rotating wind, ±0.005 s clock
  offsets, 0.05 m position biases and 0.02 m noise. Execution passed; the four
  tracking-error upper bounds were 0.391/0.388/0.487/0.543 m against a 2 m tube.
- `results/trajectory3d/20261027-163809-900937/`: four-profile level transit with
  the additional static box `[160,100,0]`–`[200,140,150]` m. Planning and execution
  passed; the obstacle surface-clearance lower bound was 19.76 m against 5 m
  required. Nominal obstacle requirements differed by aircraft, retaining their
  own radius and maximum-speed/clock allowances.
- `results/trajectory3d/20261027-163308-173107/`: common-reference compatibility
  run. All plan dictionaries, catalog counts and search-node counts exactly
  match the earlier common-profile implementation's recorded baseline;
  nominal execution passed. Added configuration fields legitimately change
  the mission manifest hash, not the trajectories.
- `results/benchmarks/flight-envelope-20261027-163231-169963.json`: empty Rascal,
  66 boundary probes, six invalid-input guards and 275 equilibrium classifications.
  All contracts passed; 61 equilibria were admitted and integrated, 214 rejected.
  Rejected classifications are not successful flights.
- `results/benchmarks/vertical-flight-20261027-163232-554333.json`: fuelled Rascal,
  12/13 m/s level and climbing fixtures, nominal and refined command intervals;
  all eight positive trials and the separate expected negative checks passed.
- `results/benchmarks/vertical-flight-20261027-163307-202208.json`: empty Rascal,
  requested descending fixture at 12 m/s rejected with its original limits.
- `results/gui-smoke/aircraft-editor-20261027-163300-003427.png`: a real Tk window
  was opened; loading two profiles, drone/target selection and reset were exercised
  programmatically through its GUI event loop. The saved layout was inspected.

These are model-conditional functional checks, not real-aircraft certification,
scalability or timing benchmarks. Mass/geometry-driven 6-DoF dynamics, calibrated
propulsion/performance curves, terrain meshes and direct JSBSim execution remain
separate future work.

The final complete regression run (`./scripts/test.sh`) passed **664 tests** in
698.16 s, with four existing Gymnasium float32-bound warnings. No production
source was changed during that run. New regression coverage includes strict v2
geometry, exact per-ID membership, individual physical envelopes, own-only
configuration/payloads, metadata identity, conservative radius inflation,
spawn/serial equivalence, heterogeneous execution and transactional editor loads.
The separate focused loader/CLI/editor/envelope batch passed 146 tests before
the complete run. Timings overlap other checks and are not speedup measurements.
