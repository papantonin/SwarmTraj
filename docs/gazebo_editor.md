# Shared Python editor → PX4/Gazebo (one aircraft)

The Python editor, planner and Gazebo launcher now use one shared scenario.
The renderer is not a kinematic replay: a real PX4 SITL instance flies the
image's original Cessna aerodynamic model. The calculated point-mass reference
is an illustrative planning proxy, **not a calibrated Cessna model**.
Read the [backend prerequisites and limitations](px4_gazebo.md) first.
Fresh editor scenes now select the physical C++ planner by default. See
[default selection and saved-file compatibility](planner_defaults.md).
For four aircraft, use the separate opt-in
[shared-world fleet workflow](gazebo_fleet.md), not the single-aircraft JSON.

## Launch

```bash
# Edit the supplied loitering-aircraft scene, then calculate and fly it.
.venv/bin/python examples/px4_gazebo.py --edit-scenario --run --gui --gpu \
  --scenario examples/scenarios/px4_single_loiter_cpp.json --timeout 1200

# Start a fresh editor with the historical obstacle file.
.venv/bin/python examples/px4_gazebo.py --edit-scenario --run --gui --gpu \
  --seed 19 --obstacles examples/scenarios/static_obstacles.json --timeout 1200

# Calculate/export the saved scene without opening the editor or Gazebo.
.venv/bin/python examples/px4_gazebo.py \
  --scenario examples/scenarios/px4_single_loiter.json

# Replay an edited scenario saved by a previous run (replace the path).
.venv/bin/python examples/px4_gazebo.py --run --gui --gpu \
  --scenario results/px4-gazebo/RUN/scenario.json --timeout 1200
```

`--edit-scenario` opens the existing Python/Matplotlib preparation window.
It starts in **Mission** view, with the map and selection-specific inputs.
Detailed timing, loiter, cruise and planner options are grouped under
**Advanced**. The summary keeps configured constraints visible; switching views
does not change their values. See the [editor guide](scenario_editor.md).
`--gui` controls the subsequent Gazebo window. `--run` explicitly authorizes
starting the isolated simulator and flying the simulated aircraft. Without
`--run`, editing and calculation only produce artifacts. Closing the editor
cancels without starting Docker. `--scenario` contains its own obstacles;
do not combine it with `--obstacles`. `--seed` overrides a saved seed only when
explicitly supplied. A shared scenario implies the warm airborne-start mode.
The unedited single-aircraft benchmark remains available with native physical reference planning.

Reference planning is fixed to the physical C++ engine. The editor has no
generator/backend switch. `--workers 1` uses the complete serial native planner;
`--workers 2` or `--workers 0` uses deterministic spawn scheduling around native
catalog/search operations. These settings never change physical limits or PX4
controllers. Missing native dependencies fail explicitly.

Archived v1/v2/v3 records retain their saved modes for inspection. Python,
geometric and legacy production requests are rejected; command-line C++ aliases
cannot silently replace an archived choice. Explicitly migrate to a new file
with [the scenario migration tool](planner_defaults.md#archived-scenarios-and-explicit-migration).
**Reset** restores the original recorded inputs. The benchmark without an edited
scenario now also constructs its fresh reference with the physical C++ planner.

New scenarios use `swarm-px4-editor-v3` with required
`mission.generator_mode` and `mission.numerical_backend`. Existing v1 files mean
`geometric` plus `python`; v2 requires its explicit generator and means `python`.
Newer fields in older records are rejected. The historical standalone benchmark
without an edited/saved scene does not accept generator/backend/process overrides.
See the [migration gates and example commands](generator_migration.md).

## Editing

### Trajectory-only completion (default for edited/saved scenarios)

Edited/saved scenarios now default to `--end-behavior remove-at-target`.
There is no return or landing pattern, and no landing-pattern search. After
initial loiter, each simulated aircraft follows its own spatial reference.
An own ground-truth sample inside the configured target radius (XYZ, or XY
when common altitude is disabled) requests removal of that aircraft's Gazebo
model. The service must acknowledge and the model must disappear from scene
information before completion is accepted. No in-air disarm, actuator command
or real-aircraft termination operation is sent. Removal is simulation-only.
Other aircraft continue independently; a missed target or failed removal fails
the experiment rather than being counted as a successful arrival.

The immutable point-mass plan still contains and validates the full horizon,
including onward flight. This change does **not** relax that planner or its
collision checks. Only the derived native flight/display is shortened: cyan
visuals end at nominal arrival, and a five-second native fly-through tail allows
target acquisition without stopping a fixed-wing aircraft. The removal trigger
is observed position, not nominal time or mission-waypoint acceptance. Discrete
sampling and transport latency mean disappearance is not instantaneous at the
mathematical sphere boundary. No real-flight safety claim follows from deletion.

This pinned Cessna normally requires a landing item. In this explicitly
simulation-only mode, its fresh instance sets `MIS_TKO_LAND_REQ=1` (require a
takeoff item, not a landing), records the previous/effective value and reads it
back. The mission still undergoes the native feasibility, readiness and arming
checks; all other parameters/failsafes remain unchanged. The historical mode is
available with `--end-behavior land`, retaining its landing screens and normal
landing/disarm acceptance. The standalone historical benchmark, without a
shared editor/scenario, still defaults to `land`.

Reports include `target_contact`, `removal`, `end_behavior`, and
`simulation_mission_requirement`; `target-contact.json` and
`target-removal.json` retain the observed event and removal evidence. The native
mission has no landing commands in trajectory-only mode.

Native single-aircraft run `results/px4-gazebo/20261028-162939-172968`
completed this mode: contact was observed at t=116.0 s, distance 5.964 m within
the 6 m XYZ radius, model removal was acknowledged and absence verified, and
the loiter/spatial experiment gates passed. Removing a model also removes its
simulated sensors; subsequent PX4 timeout messages can therefore appear before
container shutdown. Those logs are retained, not suppressed. This is neither
normal landing/disarm nor a claim of uninterrupted sensor health.

The full suite passed 789 tests before a final display-only preview correction.
The final targeted adapter suite passed 118 tests, including deterministic,
finite preview bounds set before adding 3D obstacle collections (avoiding an
intermittent Matplotlib autoscale overflow). This preview fix changes rendering
only, not physical states, planning, native commands or removal criteria.

### Scene controls

Position, heading, airspeed and obstacle dimensions are in **Mission** view.
The detailed timing, cruise, loiter and generator/backend controls listed below
are in **Advanced**. **Reset** restores the original scenario and Mission view.

- Select **Drone 0**, click the map or enter X/Y and **Altitude (m)**. The
  selected point is the aircraft's actual initial position, **not** the center
  of its holding circle. **Heading (°)** is the tangent heading: 0° east, 90° north.
- **Loiter radius (m)** sets the holding-circle radius. **Counterclockwise** selects
  counter-clockwise motion; unchecked means clockwise. Radius and heading
  determine the circle center displayed on the map.
- **Airspeed (m/s)** is total airspeed. **Initial loiter (s)** is the planned
  common activation/holding duration; **Extra wait min/max** adds the existing
  deterministic candidate delay. The selected plan's full release time becomes
  the native loiter duration. No hidden extra random delay is introduced.
- Select **Target** to change X/Y and the common destination altitude. The
  existing common-altitude checkbox retains its meaning: unchecked requests an
  XY destination, not a common XYZ point. Initial/cruise/target heights need not
  be identical; moderate altitude transitions are exported as spatial waypoints.
- Select an **Obstacle** to edit its XYZ center and half-dimensions. For a
  cylinder, **Radius (m)** is its radius and **Half-height (m)** is half its
  height. A box uses **Half-width X (m)**, **Half-width Y (m)** and **Half-height (m)**.
  Click the map to
  move an obstacle center. **+ Cylinder**, **+ Box** and **Remove obstacle**
  add/remove solids. **Apply** commits the current fields; **Reset**
  restores the input scene, including its obstacles and holding geometry.
- **Horizon** remains the full planning duration, including initial loiter and
  onward flight. It is not the wall-clock test timeout or an arrival-only budget.
  **Arrival deadline (s)** and **After arrival (s)** retain the planner's separate
  arrival deadline and minimum onward-flight duration and are saved in the JSON.

Click **Calculate → Gazebo**. The same planning/physical/obstacle validation as
the Matplotlib model runs before Gazebo is started. A failed calculation leaves
the editor open, with the diagnostic and all placements available for correction.
Finite catalog exhaustion does not establish physical infeasibility. No margin,
deadline, speed or geometry is silently changed to obtain a solution.

Editor field population is batched: selecting an aircraft, target or obstacle
updates the displayed values without submitting text-box input or synchronously
redrawing the whole window for every field. A single deferred redraw presents
the final state; **Apply** also rebuilds the scene only once. Normal keyboard
editing and all launch validation remain unchanged. Matplotlib/Tk rendering is
CPU-side; `--gpu` accelerates the subsequent Gazebo renderer, not this editor.
Concurrent Gazebo instances can therefore still affect editing responsiveness.

The editor also uses a deferred-paint TextBox: a canvas click no longer causes
every unfocused input to redraw the full figure. Leaving a focused field still
submits its value and releases keyboard shortcuts; cursor and hover updates
queue a repaint instead of blocking the mouse/keyboard handler. Performance
checks must dispatch complete mouse press/release events (or native Tk events),
not just call button callbacks, since those bypass TextBox focus handling.

The first prototype accepts exactly one aircraft (ID 0), initial altitude
20–100 m, holding radius 80–500 m, initial loiter at least 5 s and airspeed
10–16 m/s. These are explicit prototype bounds, not the complete airframe
envelope. The planner proxy declares 8–16 m/s, but PX4's native minimum is
10 m/s. Steep target dives and alternative physical-airframe JSON bindings
remain unsupported; the profile-loading controls are disabled for this bridge.
The ground below the edited initial position must be clear, because normal
PX4 ground readiness/arming is still part of the warm bootstrap. Keep edited
XY inputs within ±2300 m and the complete calculated curve within ±2800 m.
The 6 km flat sensor-visible ground, original physics and native checks remain.

## Shared data and execution

```text
scenario.json: snapshot + mission inputs + obstacle solids
         → original validated point_mass_mission.json
         → scene.sdf + spatial PX4 mission + planning-preview.png
         → warm airborne initialization → native loiter → reference waypoints
```

The versioned `swarm-px4-editor-v3` JSON is intentionally strict (v1/v2 remain
readable with their historical meanings). Snapshot
`state` is `[x, y, heading_radians, signed_yaw_rate_rad_s]`; `speed` is m/s,
altitudes and all obstacle dimensions are metres. A positive yaw rate is a
counter-clockwise turn; `radius = speed / abs(yaw_rate)`. Mission keys use the
existing planner names; `activation_time` is the initial hold duration and
`horizon` includes it. The saved JSON is reusable with either launch command.
Unknown fields, multiple aircraft and unsupported configurations are rejected.

The original integrated reference is retained unchanged and fingerprinted.
Gazebo gets the same target/obstacle coordinates and dimensions and a cyan
reference annotation. Obstacles have physical collisions; reference/target
annotations are non-colliding and invisible to the downward lidar. Coordinates
remain ENU: X east, Y north, Z up. The worker translates between the edited
world and the actual geographic home beneath the initial position; no map or
reference is recentered behind the user's back.

Bootstrap initializes position, velocity, bank and turn rate once, using one
pose change and one mass/inertia-proportional force/torque impulse. Planner
bank sign is converted to the Gazebo aircraft's roll convention. Actual initial
position/velocity/attitude/angular-rate gates and PX4's estimated-state gate
must pass. The documented native controller reinitialization is still performed
once. Afterwards the bootstrap only observes; it does not force the flight path.

The estimated-state gate retains its strict 3 m altitude / 3 m/s nominal
horizontal component bounds and three-second timeout. It also rejects callback
receipts from before atomic release-request publication or older than one wall
second. Receipt time is not the missing MAVSDK estimate source timestamp.
`airborne-estimator-check.json` records acquisition checks on success or failure.
The [latest editor and startup qualification](generator_migration.md#c-editor-and-ordered-startup-qualification-30-october-2026)
includes the native C++ editor export and repeated fleet experiments.

PX4 executes `NAV_LOITER_TIME` with the original circle center, altitude, radius,
direction, airspeed and selected hold duration, then flies the post-loiter
reference samples. The command does not require an extra heading-alignment
revolution when its timer expires. **This is spatial execution, not exact
reproduction of the point-mass clock or departure phase.** Native acquisition,
airspeed changes, waypoint acceptance and rounded turns introduce deviations.
Do not transfer timing, tracking-tube or swarm-separation claims to PX4.

With `--end-behavior land`, the separate return/landing uses a native pattern. The historical four
cardinal patterns are tested first, followed by orientations spaced 22.5° apart,
at distances of 600, 800 and 1000 m from home (48 proposals maximum). Every
proposal retains the 100 m circle radius, 60 m pattern altitude, radius-100 m
join buffer and at least 30 m routing buffer. No obstacle is removed and the
original reference is not changed to make a return fit. The first accepted
proposal is deterministic. Its metadata includes the rejected proposals, their
limiting obstacle indices and geometric sections; the final report records it
under `recovery`. If all fail, the error gives the best rejected geometric
proposal or a ground-footprint diagnostic, without claiming global infeasibility.
This screen is not a proof of actual PX4 swept-aircraft clearance or a proven
30 m tracking bound.

For a reproducible additional-obstacle case, run:

```bash
.venv/bin/python examples/px4_gazebo.py --run --gui --gpu \
  --scenario examples/scenarios/px4_cylinder_800_200.json --timeout 1200
```

This keeps the original two obstacles and adds a cylinder centered at
`(800,200,75)` m, radius 125 m, spanning Z=0–150 m. Add `--edit-scenario` to
adjust the scene first. With seed 19, the old four-proposal search rejected the
return even though the reference to the target was generated. The expanded
search accepts proposal 8, heading 112.5° ENU, distance 600 m, with screened
distance 55.46 m against the unchanged 30 m requirement. This is a geometric
screening result, not actual-flight clearance certification.

Native run `results/px4-gazebo/20261028-125121-969501` completed with this
three-obstacle scene: loiter, target passage, return, landing and normal disarm
all passed the experiment gates. The minimum sampled target distance was
1.442 m; full-reference spatial-error p95 was 4.696 m (maximum 8.596 m).
The minimum sampled point-to-solid distance to the added cylinder was
85.577 m. Across all obstacles and the complete run it was 13.479 m, near
the original cylinder during the landing ground roll at t=373.304 s. There
was no sampled point penetration. In particular, the 30 m proposal-screening
buffer is **not** an actual-flight/ground-roll clearance guarantee. Logs,
telemetry, fingerprints and rejected-return metadata are retained. The owned
container was removed after normal landing. The updated focused suite passed
104 tests; the full suite was not rerun for this change.

## Results and limitations

Every fresh run saves `scenario.json`, the immutable original plan,
`reference.json`, `scene.sdf`, `scene.json`, `planning-preview.png`, native mission,
telemetry, plots, logs and bootstrap evidence. The final report includes:

- measured loiter duration, radial errors, altitude error and signed angular
  progress, with an explicit experiment-only loiter gate;
- sampled distance to the target and whether the configured target radius was
  actually reached (XYZ or XY according to the editor setting);
- full-reference spatial errors and observed obstacle-center penetration;
- actual landing/disarm and the existing completion/spatial gates.

For this workflow, a successful backend test additionally requires the loiter
experiment and sampled destination gates. Failure is not hidden by merely
completing the uploaded mission. All original obstacle/planning margins and
the 25 m p95 spatial experiment threshold are unchanged. That percentile
threshold is **not** an obstacle-clearance guarantee.

The bridge remains one-aircraft, one pinned uncalibrated Cessna fixture. It does
not provide arbitrary drone models, live replanning, reactive obstacle avoidance,
time-aligned following, a radio-cutoff test or real-hardware deployment. Sensor
noise and bootstrap/controller transients mean Gazebo flights are not bitwise
reproducible even when the seeded theoretical reference is identical.

## Completed integration trial

Run `results/px4-gazebo/20261028-093427-372485` exercised the real Tk editor
and native GUI/SITL end to end. Widget edits placed the aircraft at
`(400,-100,80)` m, heading 30°, airspeed 15 m/s, clockwise holding radius
120 m and duration 40 s. The target was `(1250,450,70)` m; the cylinder was
moved to `(200,150,75)` m and the box retained. Planning was performed before
launch, not replaced by a second Gazebo-side calculation.

- Initial physical position error: 0.120 m; velocity error: 0.039 m/s.
- Observed initial loiter: 40.008 s; radial-error p95: 5.181 m;
  maximum altitude error during that hold: 2.978 m.
- Full reference spatial-error p95: 3.966 m; maximum: 6.233 m.
- Minimum sampled XYZ target distance: 1.202 m, inside the unchanged 6 m radius.
- Minimum sampled aircraft-point-to-solid distance over the complete flight:
  181.826 m; no sampled point penetration. This is not swept-body certification.
- Native mission completed, landed and disarmed normally. Backend, spatial,
  loiter and destination experiment gates passed; the owned container was
  cleaned up and native logs retained.

The run preserves its own source fingerprints. Later changes in this iteration
adjusted editor layout, preserved optional deadline/onward-duration JSON fields
and corrected scene-count/plot metadata; these were checked by GUI smoke tests,
the saved-example export and 100 focused passing tests. The preceding full
suite passed 749 tests before those small follow-ups. No multi-aircraft flight
or clock-aligned PX4 following was tested.
