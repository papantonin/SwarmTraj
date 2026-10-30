# One-aircraft PX4 / Gazebo experiment

This optional experiment exercises the existing generator against an actual
open-source fixed-wing simulator and autopilot. It is not the project's NumPy
physics dressed up with a different renderer: the generated reference is flown
by PX4's mission/navigation/control stack and Gazebo's `rc_cessna` airframe.
The local `rc_cessna_lidar` variant inherits that airframe and adds a downward
range sensor, needed by PX4's unmodified terrain-dependent landing checks.

The [shared editor workflow](gazebo_editor.md) now supports one edited initial
holding circle, target XYZ, static obstacles and moderate altitude transitions.
The historical straight/level benchmark below remains available unchanged.

**Scope:** one simulated aircraft, spatial playback, optional static obstacles,
no multi-aircraft execution and no real-hardware connection. It is not yet the
live-designation/frozen-telemetry pipeline. Do not transfer the point-mass
tracking/separation claims to this backend.

## Prerequisites and launch

For first-time host setup, Docker installation and troubleshooting, follow the
[new-machine installation guide](installation.md).

The tested host is Linux x86-64 with Docker, Python 3.12 and this project's
existing `.venv`. The container reuses that virtual environment read-only;
other hosts/interpreters are not validated. Do not mount a real flight controller.

```bash
# Optional dependencies and pinned simulator image. No host ROS/Gazebo install.
bash scripts/setup_px4_sitl.sh

# Offline reference generation only: no simulator and no flight commands.
.venv/bin/python examples/px4_gazebo.py

# Actual isolated PX4/Gazebo flight, without a window.
.venv/bin/python examples/px4_gazebo.py --run

# Same experiment with a Gazebo window on a local X11/XWayland desktop.
.venv/bin/python examples/px4_gazebo.py --run --gui --timeout 1200

# Explicit Intel/AMD Mesa GPU access; recommended for interactive viewing.
.venv/bin/python examples/px4_gazebo.py --run --gui --gpu --seed 19 --timeout 1200

# Same JSON obstacle geometry as the Matplotlib simulator.
.venv/bin/python examples/px4_gazebo.py --run --gui --gpu --seed 19 --timeout 1200 \
  --obstacles examples/scenarios/static_obstacles.json

# Experimental warm airborne initialization, same one-aircraft scenario.
.venv/bin/python examples/px4_gazebo.py --run --gui --gpu --airborne-start \
  --seed 19 --timeout 1200 --obstacles examples/scenarios/static_obstacles.json
```

## Experimental airborne start (single Cessna only)

`--airborne-start` is an explicitly limited **warm simulator initialization**,
not a cold PX4 boot in flight or a complete aircraft/controller checkpoint.
The original level reference is unchanged: its start is world ENU `(300,0,80)` m,
15 m/s eastward, zero initial bank/flight-path angle. The target and obstacles
remain at the same world coordinates. The native Cessna model is first placed
on the ground below that start for normal sensor/EKF readiness and arming.
The GUI is deferred until airborne initialization and Mission activation.

The sequence is:

1. Attach PX4 to the pre-created official airframe plus the existing lidar.
   Preserve firmware/SDK checks, network isolation, original parameters,
   preflight/arming checks and asynchronous mission-feasibility checks.
2. Arm normally. Select the first forward spatial waypoint (sequence 3),
   avoiding the runway takeoff and the first point now behind the aircraft.
   The unchanged takeoff/landing records remain in the derived mission; the
   demonstration uses the verified native 15 m/s trim speed.
3. The simulator-only Python system applies one position change, then one
   translational impulse on the following 4 ms physics step. Each link receives
   a mass-proportional force at its center of mass. Separating those steps keeps
   the ground-contact impulse out of the initial velocity. There is no sustained
   velocity command, pose override, trajectory replay or injected estimator data.
   Initial pitch is zero in this straight/level fixture, not calibrated AoA trim.
4. Verify the observed physical start within 0.5 m and 0.5 m/s of the request.
   Wait for PX4's own airborne detection and estimated height/velocity to acquire
   the new state. The latter gate is sampled every 20 ms; telemetry is requested
   at 50 Hz. Timeout fails before Mission activation, without widening limits.
5. Reinitialize `fw_lat_lon_control` once with its native stop/start commands,
   then activate Mission. This pinned prerelease controller otherwise retains
   its ground-level TECS altitude reference across an AUTO-mode height reset.
   EKF, attitude and rate modules remain running; no safety parameters change.
   **This simulator bootstrap is not an operational real-aircraft procedure.**
6. Open the GUI, attach the chase camera, and let PX4 and ordinary Gazebo
   physics fly. After the initialization impulse, the plugin is observer-only.

The local geographic origin is PX4 home at the ground spawn beneath the start.
Only derived geographic mission coordinates are translated by `(300,0,0)` m;
telemetry is translated back for world-frame comparison. No original reference
samples, obstacle positions or target positions are moved. Separate return and
landing are relative to that ground spawn, not the previous `(0,0)` runway.

Additional artifacts are `airborne-bootstrap-ready.json`,
`airborne-release.json`, `airborne-initial-state.json`, `airborne-check.json`
and `airborne-physics.csv`. The last is a 10 Hz observer-only recording of the
first approximately 30 s after release. `initial_state_passed` refers only to
the immediate physical initialization, **not** stability of the ensuing flight.
The flight report records controller reinitialization and estimated-state
acquisition. Camera initialization in this mode occurs after arming/Mission,
unlike the standard ground-start path; GUI failure still fails the experiment.

Observed trial `20261027-212901-335406`: startup proceeded to Mission with
original safety parameters. Across 293 physical observer samples spanning
29.992 s, altitude stayed between 78.344 and 81.266 m; ground-speed magnitude
ranged from 11.251 to 15.000 m/s. XYZ distance to the reference polyline had a
maximum of 9.414 m and p95 of 8.730 m. `airborne-observed-30s.json` retains this
one-off analysis. An actual GUI-camera capture was inspected and camera status
confirmed FOLLOW. Seventy focused tests passed. These observations establish
only this startup trial, not arbitrary-start robustness, full-flight acceptance,
time-aligned tracking, continuous obstacle clearance or real-aircraft safety.
The scene still uses the illustrative point-mass profile, not a calibrated
Cessna trajectory model.

That trial was deliberately interrupted after reference completion to repeat
the startup; it is recorded as `KeyboardInterrupt`, **not** a completed-flight
pass. It also retained a later roll warning and a full-flight sampled minimum
center-to-obstacle distance of only 0.232 m during the broader mission. That is
not acceptable swept-aircraft clearance and reinforces the existing limitation
that the separate PX4 return/landing is not obstacle-planned. The original
spatial maximum included a ground sample just before release because sequence 3
had already been selected. Subsequent code filters reference observations by
the physical release timestamp; all post-release transients remain included and
the original diagnostic report is not rewritten.

Repeat startup `20261027-213456-342490` used the final source fingerprints and
passed the same immediate-state gate: position error 0.114 m and velocity error
0.039 m/s. In 293 physical samples over 29.996 s, altitude ranged from 78.363 to
81.059 m, ground speed from 11.246 to 15.000 m/s, and maximum spatial distance
to the reference was 10.294 m (p95 9.595 m). Mission execution and chase-camera
FOLLOW were observed; the simulation was left running. All 72 focused adapter
tests passed. This second startup observation is not a completed-flight or
obstacle-clearance acceptance result. It confirms warm-start feasibility for
this fixed demonstration, not arbitrary editor inputs or guaranteed repeatability.

Retained diagnostics (all under `results/px4-gazebo/`):

- `20261027-211924-054270`: world startup failed to resolve the accessory model;
  corrected by using its explicit mounted SDF path. Interrupted, no flight.
- `20261027-212008-445358`: physical placement worked but the original takeoff
  transition lost about 29 m altitude; deliberately interrupted, not accepted.
- `20261027-212225-442402`: direct waypoint entry alone remained unstable,
  with altitude reaching about 6.43 m and attitude/bias warnings; interrupted.
- `20261027-212511-813250` and `20261027-212756-517040`: estimated-state gates
  timed out before Mission activation. Native logs showed the brief acquisition
  interval was missed by 200 ms polling; telemetry rate / polling were improved,
  rather than relaxing the 3 m / 3 m/s acquisition thresholds.

These historical straight/level trials predate the
[shared editor bridge](gazebo_editor.md). The bridge has explicit input bounds;
arbitrary attitudes or airframe profiles remain unsupported.
Battery/motor/servo/EKF/controller trim states are not
initialized from a full-flight checkpoint; bootstrap transients remain.

## Destination and static obstacle display

The destination is always drawn as a gold sphere at the reference arrival
altitude, with a gold ground disk and a narrow vertical beacon. The current
demo destination remains ENU `(1200,500)` m, at approximately 80 m altitude.
Because the planner's default target is XY-only, this display does not introduce
a new common-altitude constraint. The sphere uses the configured arrival radius
(6 m), not a physical collision body. These annotations have no collisions,
cast no shadows and use visibility bit 1 only; the downward lidar excludes this
bit, so a destination annotation cannot become fictitious landing terrain.

The theoretical reference is also displayed automatically: bright cyan from
the synthetic initial flight state to target arrival, muted blue for the rest
of the planned horizon. Neither line represents the separate PX4 takeoff/join
or return/landing procedures. The line is a 1.2 m diameter visual annotation,
not a flight corridor, uncertainty tube or clearance guarantee. To keep the
scene lightweight, a deterministic polyline reduction allows at most 0.5 m
sample-to-replacement-segment deviation; original flight samples, mission
waypoints, timing and validation remain unchanged. A fractional arrival sample
is linearly interpolated only for this display. The line has no collisions or
shadows and is excluded from the lidar like the gold target markers. Look for
`swarm_reference_arrival` and `swarm_reference_onward` in the model tree.

Reference-display smoke check (`20261027-205552-046132`): 65 focused adapter
tests passed and the generated world passed Gazebo's SDF validator. An actual
GUI-camera capture (`2026-10-27T19:56:54.488529088.png`) shows the cyan reference,
the gold destination and both gray obstacles. Twelve display segments replace
the sampled curve in this scene; the original mission and source fingerprints
were retained. PX4 took off and the range sensor returned finite downward
ranges. This is a visual/startup check, not a completed-flight acceptance or a
claim that actual PX4 motion exactly follows the theoretical reference.

`--obstacles FILE` reads the [same box/cylinder JSON](obstacles.md) as the
Matplotlib launcher. The immutable point-mass mission contains this geometry;
the generator performs its normal obstacle checks without relaxed margins.
A fresh `scene.sdf` is derived from that mission: boxes use twice the JSON
half-extents, and vertical cylinders use the XY radius and twice the Z
half-extent as their height. The gray solids have matching visual and collision
geometry and remain visible to the range sensor. No obstacles are loaded
implicitly; omit the option for an empty scene. Invalid/unplannable inputs fail
before Docker starts. The worker verifies the derived scene against the recorded
mission before sending an arm command. Scene assets and the generated SDF are
fingerprinted in the run artifacts.

**This is not certified PX4 obstacle avoidance.** PX4 waypoint acceptance and
corner rounding can leave the model's tracking tube; takeoff/join and the
separate return/landing pattern are not obstacle-planned. The launcher prints
this limitation when obstacles are supplied. The observer reports the minimum
sampled *center-to-solid signed distance* over recorded flight samples and
rejects a detected center penetration, but a nonnegative result is not a
continuous-time swept-aircraft clearance check. The report field
`gazebo_obstacle_clearance_validated` remains false. No real-aircraft or swarm
safety claim transfers to this experiment.

The scene uses Gazebo's ENU world with PX4 home at its nominal origin. This is
the existing laboratory fixture, not a georeferenced terrain import or arbitrary
home/spawn mapping. Use Gazebo's model list to locate `swarm_target` and
`swarm_obstacle_0`, `swarm_obstacle_1`, etc. Press Escape to release chase-camera
following and inspect the scene; camera follow at short range will not show
every distant object simultaneously.

Scene smoke check (`20261027-204703-953755`): 92 focused adapter/obstacle tests
passed. Gazebo accepted the generated world with `gz sdf -k`; a GUI-camera
capture (`2026-10-27T19:50:14.302462223.png`) was inspected and shows both gray
solids and the gold airborne/ground destination markers. PX4 took off, mission
execution advanced and the downward lidar returned finite ground ranges. The
GUI was reopened during inspection after it shut down. This is a scene/flight
startup check, not a completed-flight or obstacle-clearance acceptance result.

GUI mode requires `DISPLAY`, a readable `XAUTHORITY` file and the local X11 socket.
It mounts the authorization file read-only; it does not run `xhost +`, publish a
remote display server or grant privileged access. Without `--gpu`, rendering uses
software OpenGL, which can be very slow on a laptop. Use the headless command if no compatible
display is available. `comparison.png` is also produced without a GUI.
The sensor server uses EGL `--headless-rendering` even when the separate GUI is
shown. It does not require host GPU/device passthrough on the tested software renderer.

`--gpu` is an explicit, Linux Intel/AMD Mesa opt-in: the launcher checks and exposes
only `/dev/dri/renderD128` with read/write permissions and sets
`LIBGL_ALWAYS_SOFTWARE=0`. It never exposes serial/flight-controller devices,
the full `/dev` directory or a privileged container. The host render node must
be accessible; no host driver installation, permission changes or `--gpus all`
NVIDIA configuration is performed. Network isolation remains unchanged.
The node being present does not prove hardware acceleration: check `ogre2.log`
for `GL_RENDERER` and confirm the actual GPU instead of `llvmpipe`. The runtime
manifest records the requested graphics mode/node, not a measured FPS guarantee.

GPU restart check (`20261027-200718-366515`): the renderer reported
`Mesa Intel(R) UHD Graphics (TGL GT2)`; both server and GUI retained open
`/dev/dri/renderD128` descriptors, network mode remained `none`, and the aircraft
took off and advanced through the mission. At inspection the flight was still
in progress; this is not another completed-flight acceptance result or an FPS
measurement. The 41 focused adapter tests passed. A texture-budget warning and
material-assignment diagnostics remained, so GPU access alone is not proof that
all visual defects have been resolved. The previous software-rendered run was
explicitly interrupted for the restart, with its logs retained.

`--seed` selects the generator seed. `--timeout` is a wall-clock flight budget
(default 600 s), separate from the reference's 100 s horizon. Takeoff, acquisition,
return and landing are additional PX4 procedures outside that reference horizon.
`--output` must name a **new** directory. Every run has independent artifacts.
The GUI example increases only the wall-clock test budget for software rendering,
not the reference horizon, speed/envelope limits or safety margins.

The runtime image is cached in Docker (roughly 2 GB unpacked); the launcher does
not remove that cache or modify unrelated containers. It stops and removes only
the uniquely named test container and retains the reports/logs on the host.

## Exact software selection

The [official PX4 Gazebo documentation](https://docs.px4.io/v1.17/en/sim_gazebo_gz/index)
lists `gz_rc_cessna` as a plane model, airframe 4003. The underlying
[PX4 Gazebo models](https://github.com/PX4/PX4-gazebo-models) provide its geometry,
sensors, joints, motor/servo systems and aerodynamic force plugins.

This implementation pins the available official runtime image to:

```text
px4io/px4-sitl-gazebo@sha256:5ace9dbca657e385cc5c529432fd6d09db6165e482de9df5ab129a941a36aa54
human-readable tag at verification: v1.18.0-beta1-660-g61907962c9-amd64
PX4 git: 61907962c9220437ec01a98fa26bfbf5cbb254b5
Gazebo Sim: 8.15.0 (Harmonic)
MAVSDK Python: 4.1.0
```

**This is a development/prerelease PX4 build, not stable v1.17.** The advertised
v1.17 runtime tag was unavailable when checked; the available image is pinned by
digest rather than silently using `latest`. To change it, deliberately review
the image/model, update the expected firmware hash and revalidate the adapter.
MAVSDK Python 4.1 uses the native `Mavsdk`/plugin API, not the older asynchronous
`System().connect()` interface. The adapter refuses an untested SDK version.

The open-source Cessna is a software test airframe, not calibration of the user's
aircraft. Its observed PX4 settings are 10 m/s minimum, 15 m/s trim and 20 m/s
maximum airspeed. This first demonstration uses 15 m/s; it does **not** validate
a 25 m/s aircraft or automatically increase those limits.

The checked-in [sensor fixture](../examples/gazebo/models/rc_cessna_lidar/model.sdf)
uses SDF merge-inclusion of the image's official `rc_cessna`; its original
aerodynamic and actuator plugins are not replaced. It adds a 1 g fixed sensor
link and a 30 Hz single-beam simulated lidar (0.1–100 m, nominal Gaussian range
noise 0.02 m), following the official `x500_lidar_down` mounting/bridge convention.
This is a test accessory, not a calibrated real sensor installation. The mass
addition means this is explicitly an adapted, not bit-identical stock airframe.
PX4 still uses airframe 4003, original `FW_LND_USETER=1` and `FW_LND_ABORT=3`.
The sensor's Gazebo observations feed PX4's estimator through its normal bridge;
the Python observer never supplies synthetic terrain heights to the controller.
The local [world fixture](../examples/gazebo/worlds/default.sdf) keeps the default
physics timestep, gravity, origin and atmosphere, but extends both ground-plane
footprints to 6 km. A GPU lidar intersects **rendered** ground geometry; the
image's 500-by-500-metre visible tile does not cover the reference or landing
approach. An infinite physics plane alone does not fix that sensor problem.

The ground appearance uses an original 32-by-32-pixel grass/field motif
(`materials/textures/grass.ppm`, under 12 KB on disk), repeated every 40 m on
a four-vertex/two-triangle mesh. The mesh remains exactly at Z=0 with the same
6 km footprint; the original collision plane, dynamics, gravity and sensor rate
are unchanged. The static Ogre2 skybox is the one already shipped with the
pinned Gazebo runtime (`<scene><sky/>`), with a blue fallback background. There
are no streamed textures, terrain elevation or volumetric
cloud effects. The local mesh/texture are included in source fingerprints.

Visual smoke check (`20261027-202434-191322`): `gz sdf -k` accepted the world,
42 focused adapter tests passed, and the actual Gazebo GUI-camera screenshot
was obtained through `/gui/screenshot` and inspected. It shows the green tiled
ground and static cloudy sky, not the previous white/gray scene. The renderer
reported Intel UHD Graphics; the range sensor still returned finite downward
measurements and PX4 advanced through the flight. The flight was left running;
this check is not a completed-flight acceptance or measured FPS guarantee.

With `--gui`, the adapter initializes chase-camera following after Gazebo/PX4
readiness and before arming. It disables PX4's unacknowledged one-shot camera
publication, waits for acknowledged `/gui/follow` and `/gui/follow/offset`
services, and checks `/gui/currently_tracked` for the intended model and offset.
After acknowledgement, it explicitly sets both camera gains to 1 and verifies
the reported follow gain. Gazebo's default 0.01-per-frame gain otherwise leaves
the view far behind a 15 m/s fixed-wing aircraft.
The startup view is 6 m behind, 3 m to the side and 3 m above the model, in its
local frame. Failed GUI initialization stops before arming, rather than claiming
that the view is following. The handshake is bounded and skipped when headless;
it sends no aircraft controls and does not change flight planning or physics.
Once attached, Gazebo maintains the follow itself. The adapter does not keep
forcing the camera back during flight: press Escape in the 3D view to cancel
following and explore the scene freely. The confirmed startup state is retained
as `camera_follow` in the flight report.

Follow-camera smoke check (`20261027-203519-731790`): 56 focused tests passed.
Startup followed the acknowledged setup before PX4 takeoff. Two in-flight status
samples reported `FOLLOW`, target `rc_cessna_lidar_0`, offset `(-6,-3,3)` and
follow gain 1. The camera position moved from about `(326,9,81)` to `(714,241,81)`
metres as the aircraft advanced. GUI-camera captures were inspected to confirm
the aircraft remained framed against the textured scene. Source fingerprints
matched and the simulation was left running. This is a visual following check,
not a completed-flight or trajectory-safety acceptance result. The earlier
diagnostic run used the slow default gain and was explicitly interrupted;
its original report/logs remain retained.

## What happens during a run

1. The normal seeded catalog, coupled point-mass rollout and independent
   validator generate one reference. It starts at ENU `(300,0,80)` m, heading
   east at 15 m/s, and uses a destination `(1200,500)` m at the same nominal
   altitude. All positions through the full 100 s reference horizon are retained,
   including onward flight after destination passage. The profile is still the
   historical illustrative point-mass profile, **not** a calibrated Cessna model.
2. The launcher starts its own Docker container using `--network none`, no host
   UDP ports, no serial/flight-controller access and no privileged mode. The
   optional `--gpu` grants only the explicit DRM render node. Only the repository
   is mounted read-only; this run's fresh output directory is writable.
3. Inside that same container, MAVSDK discovers PX4 on container-local loopback.
   The worker verifies `PX4_SITL`, the exact firmware hash and airframe 4003, then
   waits for the original PX4 readiness checks, position/home and ground truth.
   No checks, landing requirements, failsafes or envelope parameters are disabled.
   After upload, it also waits for PX4's asynchronous mission feasibility result
   before arming: a successful upload acknowledgement alone is insufficient.
4. The reference's canonical mission hash and position arrays are checked before
   use. WGS84 ECEF/tangent-plane conversion maps local ENU to geographic mission
   coordinates about PX4 home. PX4 telemetry's local NED axes are not treated as ENU.
5. A **derived spatial waypoint mission** samples the reference at about 40 m
   arc-length intervals. Tiny coupled vertical transients remain unchanged;
   nominally level reference classification allows at most 0.1 m vertical
   excursion. This is an adapter scope check, not a relaxed flight-envelope limit.
6. PX4 performs a runway takeoff and acquisition, flies the spatial samples with
   its own controllers, then follows a separate standard landing sequence:
   `DO_LAND_START`, `NAV_LOITER_TO_ALT` at ENU `(-600,0,60)` m with 100 m radius,
   and `NAV_LAND` at home. Its glide slope is below the airframe's original 8°
   landing limit. The entire mission remains subject to PX4 feasibility checks.
   The flat, no-wind 6 km ground fixture is part of this experiment; optional
   obstacle geometry does not make this separate landing pattern obstacle-aware.
7. An independent observer records Gazebo ground truth forwarded by PX4's
   `HIL_STATE_QUATERNION`, plus PX4 attitude and airspeed telemetry. The ground
   truth timestamp supplies simulation time, not host wall time.
8. The test waits for reference completion and PX4-reported on-ground/disarmed
   status after landing, then ends the simulation. It never forces an in-air
   disarm. A switch from Mission to Hold while airborne fails the test promptly.
   Aborted/failed tests also stop the owned simulator and preserve evidence.

The first input state is synthetic; takeoff/acquisition brings the simulator to
the reference. It is **not** an exact initial-state match to a live Gazebo
snapshot. No other aircraft's telemetry exists in this one-aircraft fixture.
MAVSDK keeps a simulated ground-station heartbeat active to preserve the
airframe's unmodified data-link failsafe requirements while telemetry is logged.
The mission runs in PX4, but **radio cutoff/no-ground-communication behavior is
not tested** here. The generator seed reproduces the reference, not bit-identical
Gazebo flights with simulated sensors and estimator initialization.

## Measurements and acceptance

MAVLink waypoint acceptance and PX4 corner rounding change the executed path
and timing. Consequently:

- `backend_test_passed`: the reference waypoints were completed, the test
  completed with reported landing/disarm, and enough ground-truth samples exist;
- `spatial_tracking_passed`: after acquisition and before the separate landing
  procedure, at least 50 ground-truth samples exist and the 95th percentile XYZ
  distance to the **reference polyline** is at most 25 m;
- `spatial_error_max_m` and altitude error are reported separately, not hidden
  by the percentile gate. The 25 m gate is an explicit, coarse adapter experiment
  criterion, not the planner's original 2 m tracking tube;
- `timed_tracking_supported`, `swarm_separation_validated` and
  `safety_certificate_transferred` remain **false** even when the spatial
  experiment passes. Nearest-polyline distance is not synchronized tracking
  error, continuous-time clearance or proof of complete reference coverage;
- reported landing/disarm is not a detailed landing-quality or contact-load
  certification. Attitude and airspeed are asynchronous PX4 telemetry samples,
  not a complete airframe-envelope audit of Gazebo dynamics.

Exit 0 requires both backend and spatial experiment gates. Exit 3 records a
flight/quality failure; exit 2 records a launcher/input/dependency error. A native
worker crash that cannot write a report is retained explicitly as a failed
worker, not silently counted as a successful flight.

## Artifacts

Each run under `results/px4-gazebo/` contains:

- `point_mass_mission.json`: original, unmodified model-validated reference plan;
- `reference.json`: ENU reference samples/times, seed, original mission hash,
  source fingerprints and explicit loss-of-timing declaration;
- `scene.sdf`, `scene.json`: destination annotations and matching static solid
  geometry, original mission identity, scene hash and explicit safety limitations;
- `runtime.json`: pinned image, simulator identity, container isolation and SDK;
- `px4_mission.json`: derived MAVLink records, sample indices, 15 m waypoint
  acceptance radius and timing-not-preserved declaration;
- `telemetry.csv`: simulation timestamps, global/ENU ground truth, mission
  progress and asynchronous attitude/airspeed samples;
- `comparison.png`: generated reference versus Gazebo flight, including the
  separate takeoff/join and return/landing procedures;
- `report.json`, `adapter.log`, `px4-gazebo.log` and `runtime_end.json`: acceptance,
  retained failures, PX4/Gazebo diagnostics and owned-container cleanup status.
- `px4-logs/`: copied native ULog files, and `px4-events.json.xz`: matching event
  definitions. These preserve structured abort/failsafe events that might not
  appear in MAVLink status text. `gazebo.log` retains available rendering logs.
  `ogre2.log` retains available graphics renderer/shader diagnostics, including
  the actual renderer selected with `--gpu`.
  Native log collection is best effort and recorded in `runtime_end.json`.

Some early failed runs may lack artifacts that could not be created before the
failure. Do not overwrite them or relabel them as expected successful tests.

## Recorded validation (2026-10-27)

These are observed software-in-the-loop experiments, not a safety certification.
All paths below are relative to `results/px4-gazebo/` and are retained locally.
The complete regression suite passed: **701 tests**, including 37 new offline
adapter/fixture tests, in 729.97 s under concurrent simulator load. The four
pytest warnings are the existing Gymnasium float32-bound conversion warnings.
Shell syntax, Python compilation, local documentation links and both experiment
source-fingerprint matches were also checked. Native ULog diagnostic inspection
used `pyulog==1.3.0`; that diagnostic package is not needed by the launcher.

| Run | Configuration | Reference samples | Spatial error p95 / maximum | Observed outcome |
| --- | --- | ---: | --- | --- |
| `20261027-194054-904133` | seed 7, headless, lidar + extended ground | 1044 | 3.759 / 36.750 m | Reference completed, landing detected, automatic disarm; exit 0 |
| `20261027-194452-610005` | seed 19, Gazebo GUI enabled, same fixture | 1036 | 3.977 / 35.673 m | Reference completed, landing detected, automatic disarm; exit 0 |

An additional read-only inspection of that run's native ULog confirmed
`mission_result.finished=1`, `valid=1`, `failure=0`, final arming state disarmed,
and no nonzero landing abort status. The flight ended at about 384 s of simulator
time, including all takeoff/return/landing phases. Its source fingerprints match
the final adapter/fixture sources. These observations do not establish a bound
on future tracking errors. In particular, the measured maximum is much larger
than the original planner's tracking tube and cannot be used as swarm clearance.
The second flight's native log likewise confirms finished/valid, no mission
failure, no landing abort, and final disarm. Gazebo's GUI/rendering initialization
and camera tracking are recorded in `gazebo.log`; the desktop automation provider
did not expose the container's Qt window or screenshot capability, so there was
no independent automated visual inspection of that window. The generated
`comparison.png` was inspected separately. Both test containers were stopped
and removed, while all reports and native logs were retained.

Important retained diagnostic cases:

- `20261027-183508-592639`: no valid landing sequence; PX4 refused Mission mode.
  This led to a real reachable landing pattern, not disabling landing checks.
- `20261027-184915-651578`: upload acknowledged before Mission mode became
  available. The adapter now explicitly waits for PX4 mission feasibility.
- `20261027-185036-432681`: stock Cessna, no downward range sensor. Native
  events identify **terrain measurement not found**; the flight is failed.
- `20261027-185337-962633`: interrupted diagnostic flight after identifying an
  incorrectly classified sensor orientation; retained as `KeyboardInterrupt`,
  not counted as a completed flight.
- `20261027-185447-630763`: valid downward sensor but the stock visible ground
  ends outside the landing approach. Native events identify **terrain estimate
  timed out**; the flight is failed. The final world covers that approach.

PX4 also emitted transient `Accel #0 ... TIMEOUT` warnings during concurrent
simulator/regression load (one in the headless run, three in the GUI run), and
a heading-reference preflight warning after disarm in the first successful run.
These messages remain in the reports/native logs;
the coarse completion/spatial gates are **not** an all-sensor-health acceptance
gate. Do not infer clean real-time sensor behavior or hardware readiness from
exit 0. Run isolated resource/load and fault-injection tests before expanding
the scope.

## Next integration stages

This experiment deliberately does not implement a time-controlled fixed-wing
follower, arbitrary in-flight initialization, validated PX4 obstacle avoidance,
airframe calibration, a distributed deployment bridge or multi-drone Gazebo
execution. Those require separate interfaces and validation before any original
spatio-temporal safety claim can be made. ROS 2 is not required for this first
MAVLink/MAVSDK integration and no host ROS workspace is changed.
