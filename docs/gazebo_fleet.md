# Four fixed-wing aircraft in one Gazebo world

This opt-in backend extends the [single-aircraft editor bridge](gazebo_editor.md).
It uses **one** Gazebo world and four native PX4 instances, not four separate
simulations or kinematic trajectory replays. The original single-aircraft
command remains unchanged when `--num-drones` is omitted.

```bash
.venv/bin/python examples/px4_gazebo.py --num-drones 4 \
  --edit-scenario --run --gui --gpu \
  --scenario examples/scenarios/px4_four_loiter_cpp.json --timeout 1200
```

The example retains the two original solids and the additional cylinder at
X=800, Y=200, radius 125 m, height 150 m. The four aircraft start on separate
100 m clockwise circles, with 400 m grid spacing and initial altitudes
60, 70, 80 and 90 m. All use 15 m/s airspeed and a 40 s initial hold. The common
target is `(1200,500,80)` m. These are editable initial conditions, not a new
certified model envelope. The default planning horizon is 200 s; it includes
the loiter and onward reference but not the separate native return/landing.

Remove `--edit-scenario` to calculate and launch the example directly. Remove
`--run --gui --gpu` to calculate/export only. The editor must accept a complete
joint plan before Docker starts. By default, `--end-behavior remove-at-target`
removes each aircraft's model on its own observed target contact; there is no
return/landing search or landing command. The other three flights continue.
Use `--end-behavior land` to retain the historical return/landing experiment,
including all four geometric return screens. See the
[trajectory-only completion details](gazebo_editor.md#trajectory-only-completion-default-for-editedsaved-scenarios)
for the simulation-specific mission requirement and unchanged planner scope.
Closing it cancels without starting the simulator. The scenario schema is
`swarm-px4-editor-fleet-v3`, with explicit `mission.generator_mode` and
`mission.numerical_backend`. Fleet-v2 retains its generator and selects Python.
Archived fleet-v1/v2/v3 records remain readable with their original generator
and backend values; retired production settings cannot launch or be replaced by
CLI aliases. Use [explicit migration](planner_defaults.md) to write a new scenario.
Fresh scenes and `px4_four_loiter_cpp.json` use the sole physical C++ planner;
`px4_four_loiter.json` remains an unchanged historical v1 fixture.
There is no generator/backend checkbox. `--workers 1|2|0` selects serial, two
spawn processes or automatic scheduling; all numerical work remains C++.
Instance isolation and every native flight gate remain unchanged. Single-aircraft
JSON is never silently converted into four coincident aircraft. IDs must be
exactly 0, 1, 2, 3.
Ground spawn XY points must be at least 30 m apart even if flight altitudes
differ, because the warm bootstrap first initializes all aircraft on the ground.

## Architecture

```text
four edited snapshots + target + obstacles
            → one joint planning/separation validation
            → immutable original four-aircraft mission
            → four fingerprinted own-plan projections
            → one shared scene / four PX4 autopilots
            → observer-only individual + pair evaluation
```

Each instance has its own model (`rc_cessna_lidar_0` through `_3`), PX4 working
directory, parameters, native log, MAVLink system ID and SDK process. API ports
are localhost 14540–14543 inside the same network-isolated container. Native
PX4 commands explicitly select the instance; a mission upload cannot be routed
by a shared default system identity. [PX4's multi-vehicle documentation](https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation)
describes the underlying instance mechanism; this experiment keeps the pinned
runtime and original airframe/checks of the single-aircraft backend.

All four workers must pass normal readiness and mission feasibility before any
arming. After arming and mission entry selection, another preparation barrier
precedes warm airborne initialization. Workers then initialize in deterministic
ID order: a successor may release only after its predecessor passes the unchanged
actual/nominal-estimated-state gates, performs its one controller reset, and
receives its own Mission-start acknowledgement. A missing/invalid predecessor
acknowledgement or timeout aborts the experiment. This reduces simultaneous
initialization bursts and deliberately staggers native starts. Each worker still
checks its own actual/estimated state and reinitializes only its own native
fixed-wing controller once. No worker is fed other aircraft states for guidance.
This startup coordination does **not** establish clock-aligned PX4 departure.
The Python bootstrap becomes observer-only after its one pose/impulse operation.

The estimated-state gate snapshots telemetry values and monotonic receipt times
under one lock. It requires receipts after atomic release-request publication,
no older than one wall second,
and a ground-truth simulation timestamp no earlier than the physical initial
snapshot. Its original strict altitude error `<3 m`, nominal east/north velocity
component errors `<3 m/s`, and three-second acquisition timeout are retained.
MAVSDK provides no source timestamp for `PositionVelocityNed`; receipt freshness
does not prove exact source-time alignment. `airborne-estimator-check.json`
retains every acquisition check on success or failure. Each worker publishes
`warm-start-complete.json` atomically after its own gates/reset/Mission ACK;
these startup messages contain no peer state and are not in-flight guidance.

Four trajectory annotations use different colors. The camera now starts in a
free overview; see the keyboard controls below. The
single target/obstacle geometry is shared, with physical obstacle collisions
and non-colliding/lidar-invisible reference annotations.

## Free camera and keyboard controls

Free overview is the default for four-aircraft GUI runs, including existing
commands without a camera flag. Use `--camera free` explicitly or
`--camera follow` to initially follow Drone 0. Single-aircraft mode is unchanged.
The overview includes all four theoretical paths, target and obstacle solids;
the render camera's far clip is extended to 20 km so distant scene geometry
does not disappear. This changes only the GUI, never lidar range or physics.

Click inside the Gazebo 3D view before using shortcuts:

| Keys | Camera action |
|---|---|
| Z / W, S | Forward / backward along the horizontal viewing direction |
| Q / A, D | Strafe left / right (AZERTY and QWERTY layouts) |
| R, F or Page Up, Page Down | Move up / down in world altitude |
| Left / Right arrows | Turn left / right |
| Up / Down arrows | Look up / down |
| + / − | Double / halve translation step (5–400 m; initially 50 m) |
| 0 or Home | Stop following and restore the free overview |
| 1, 2, 3, 4 | Follow Drone 0, 1, 2, 3 respectively |
| F8 | Disable / enable camera shortcuts (useful before typing in GUI fields) |

Movement keys automatically detach a followed drone. Native mouse pan/orbit/
zoom remains available; the next key starts from the observed camera pose,
not a stale internally predicted position. Hold a translation key for repeated
steps. Navigation uses Gazebo's native half-second animated pose moves, not a
continuous first-person game controller; rapid stale key events are dropped
rather than replayed after release. A full overview makes aircraft small;
use the mouse wheel, movement/altitude keys or a numbered follow shortcut to
inspect an aircraft closely.

`px4_camera.py` uses native `KeyPublisher` events, `/gui/track`,
`/gui/move_to/pose` and observed `/gui/camera/pose`. It has no MAVSDK import and
issues no world/entity/actuator commands. It receives no flight-controller
ownership and does not change the generated mission or obstacle validation.
The runtime writes `free-camera.config`, `camera.log`, `camera-ready.json`
and movement requests in `camera-events.jsonl` alongside the other run artifacts.
GUI closing is still allowed and does not itself stop the independent flights.

## Artifacts and acceptance

Each fresh `results/px4-gazebo-fleet/RUN/` contains the original joint mission,
reusable scenario, shared scene and fingerprints. `drone-0/` through `drone-3/`
contain each own-plan projection, preview, bootstrap checks, native mission,
telemetry, individual report and native PX4 rootfs/logs.

Trajectory-only individual checks retain loiter, observed destination, spatial
tracking and sampled obstacle penetration gates, and require acknowledged model
removal with verified absence instead of landing/disarm. Each pair is observed
over its own overlapping telemetry interval: the first aircraft's disappearance
must not hide later conflicts between remaining aircraft. There is no
extrapolation after deletion, and a failed worker is never silently dropped.
The aggregate report identifies `all_aircraft_removed_at_target` separately
from `all_aircraft_landed`.

In historical `land` mode, individual checks retain the existing loiter, destination, spatial tracking,
sampled obstacle penetration and normal landing/disarm gates. An aircraft that
lands first keeps observing ground truth until the others have landed. The
aggregate report interpolates all four telemetry streams onto a common 0.1 s
observer grid without extrapolation, evaluates all six pairs and reports the
minimum interpolated distance. The example requests 30 m separation. Aggregate
experiment acceptance requires all individual gates and this sampled pair gate.
A failed worker aborts the experiment; it is not silently dropped from a
three-aircraft result. Native logs are retained on failed runs too.

## Important limitations

The jointly validated point-mass reference does **not** give a PX4 separation
certificate. PX4 spatial mission execution loses exact reference timing, rounds
turns and introduces tracking deviations. The per-aircraft return/landing
patterns are obstacle-screened but not jointly certified against other moving
aircraft, and landing ground roll is not modeled by that geometric screen.
The pair observer detects sampled/interpolated conflicts; it does not prevent
them and is not a continuous-time swept-body proof. Even a successful trial
leaves `timed_tracking_supported`, `swarm_separation_certified` and
`safety_certificate_transferred` false.

No real-hardware interface, radio isolation experiment, onboard deployment,
reactive avoidance or emergency maneuver is added. The same uncalibrated Cessna
fixture and 20–100 m initial-height/10–16 m/s input bounds apply. Four autopilots
and sensors consume more CPU/RAM than one; low real-time factor is expected on
the tested two-core laptop. No FPS or real-time-performance guarantee is made.

## Completed four-aircraft trial

### Trajectory-only removal trial

Run `results/px4-gazebo-fleet/20261028-161903-390290` exercised the supplied
three-obstacle scene in `remove-at-target` mode using the pinned PX4/Gazebo
runtime (headless, GPU rendering requested). All four aircraft passed their
own initial-loiter, sampled destination, spatial-tracking and removal gates.
Observed target-contact times were 125.56, 141.02, 150.12 and 169.12 simulator
seconds for IDs 0–3; sampled contact distances were 5.93, 5.94, 5.25 and 5.45 m,
inside the unchanged 6 m radius. Removal was acknowledged and model absence
verified separately for each ID, while later aircraft continued flying. No
landing command was present in their derived native missions, and
`landing_completed` is false for all four reports.

The minimum pairwise interpolated distance was 139.43 m against the 30 m
experiment threshold. Every pair used its own overlapping observed lifetime;
there was no extrapolation after removal. This remains an experimental sampled
observation, not a continuous-time separation certificate. The owned container
was stopped and removed after all four successful completions. Earlier
development runs `20261028-161339-937973` and `20261028-161527-865841` in the
single-aircraft results directory failed (contact-helper name shadowing, then
an incorrect assumption about Gazebo Scene's root name); their failure logs
remain retained and neither is counted as a completed trajectory-only flight.

Run `20261028-162342-331418` also completed the previously prepared 180-degree
arrival scene with a 0–20 s extra departure window. All four aircraft reached
the unchanged 6 m XYZ target radius (sampled contact distances 4.76–4.84 m),
passed individual tracking/removal gates and disappeared without landing. The
minimum observed pair distance was 189.80 m; aggregate experiment acceptance
passed. This test was headless with GPU rendering requested and the full test
suite running concurrently, not a controlled rendering/performance benchmark.

The scenario JSON still describes scene/planner inputs, not runtime lifecycle.
Use `--end-behavior land` explicitly when replaying an old landing experiment;
the effective policy is fingerprinted in the original mission and recorded in
the reference/fleet metadata.

### Historical landing trial

Run `results/px4-gazebo-fleet/20261028-132045-248056` flew the supplied
three-obstacle example in one shared Gazebo world. All four native missions
passed the individual loiter, destination and spatial experiment gates and
landed/disarmed normally. Measured startup release times differed by 0.004 s;
that is an observation of this run, not a scheduling bound.

| Drone | Minimum sampled XYZ target distance | Minimum sampled obstacle distance |
|---|---:|---:|
| 0 | 4.783 m | 35.339 m |
| 1 | 2.675 m | 107.285 m |
| 2 | 0.601 m | 64.375 m |
| 3 | 1.660 m | 47.400 m |

All target distances were within the unchanged 6 m arrival radius. The
minimum pair distance on the common interpolated observer grid was 133.276 m
(Drones 1 and 2), against the 30 m experiment threshold, over shared simulator
time 4.432–479.216 s. This result includes the waiting ground observers and
does not certify continuous-time swept-aircraft separation.

The full suite passed 763 tests in 936.57 s; the focused seven fleet contracts
also passed after the additional ground-spawn separation guard. A real Tk
editor smoke test displayed all four aircraft and the three obstacles,
selected Drone 3, then cancelled without launching another simulation. The full test
suite ran concurrently with this native flight, so this is not a controlled
performance benchmark. Native `Accel #0 fail: TIMEOUT!` events were retained
for Drones 0, 2 and 3; completion must not be described as an all-sensor-health
validation or proof of robustness under CPU load. Their cause was not isolated.

All four native ULogs remain inside each `drone-N/px4-rootfs/fs/log/`.
`runtime_end.json` reports the historical single-aircraft log-copy attempt as
false; that field does not describe retention of the directly mounted fleet
rootfs logs. The owned container was stopped and removed after successful
completion. The source fingerprint for the trial predates the additional
preflight XY ground-spawn guard and CLI help/implicit-airborne-flag follow-ups;
the demonstrated scene satisfies that guard. No flight-control logic changed
in those follow-ups.

Earlier run `20261028-131931-274242` failed before flight because the separate
rootfs directories lacked the installed `etc` link. Its failure reports and
console logs are retained. The startup script was corrected before the
completed trial; no preflight, mission or landing check was bypassed.
