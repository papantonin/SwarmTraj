# Coordinated trajectories with vertical separation

The launcher now uses 3D mode by default. From the repository root:

```bash
.venv/bin/python examples/trajectory_3d.py
# The existing launcher opens the same 3D mode:
.venv/bin/python examples/trajectory_2d.py
# To keep using the planar prototype:
.venv/bin/python examples/trajectory_2d.py --dimensions 2
```

The default engine is a **coupled 3D point-mass model**: total airspeed,
flight-path angle, bank, lift, thrust and drag are linked. See the
[flight model, equations and limitations](flight_model_3d.md).
The older 2.5D kinematic model remains available with `--flight-model legacy`.
Default physical parameters are illustrative, not those of a real aircraft.
The launcher defaults to physical C++ planning with one worker. Explicit
`--workers 0` selects automatic spawn workers; Python planning retains its
automatic CLI worker default. See [defaults and compatibility](planner_defaults.md)
and [performance measurements and settings](planning_performance.md).
Catalogs are built adaptively by default, with most-constrained-first search
and detailed rejection/conflict reports. See
[adaptive planning and diagnostics](adaptive_planning.md). The fixed-catalog
comparison is available with `--planning-strategy fixed --search-order id --selection-policy first`.
Expanded catalogs use sampled conflict witnesses to bias new proposals by
default; `--conflict-guidance none` disables this heuristic. Full integrated
validation remains mandatory; see [conflict guidance](conflict_guidance.md).
The CLI now refines feasible combinations to prefer greater robust separation
margin; see [margin-preferring selection](margin_selection.md).
Arrival deadlines and minimum post-passage observation are now configurable
independently of the total horizon; see [mission timing](mission_timing.md).
Optional moving preparation and full-horizon cell/time allocation are described
in [mission pipeline](mission_pipeline.md); use `--prepare-in-flight` and
`--reservation-policy grid` to enable them explicitly.
Additional cruise levels and smooth speed profiles are opt-in through
`--cruise-levels` and `--speed-profile`; `--interval-guidance safe_windows`
uses reservation windows during expanded-candidate generation. See
[settings and comparison](free_windows_profiles.md).
`--profile-policy adaptive_mixed` explicitly allows simple and rich candidates
in one adaptive catalog; `--cruise-levels` then sets a maximum. Without that
flag, profile requests remain strict. See [mixed catalogs](mixed_profiles.md).
With `--generator-mode geometric`, `--passage-guidance safe_windows` fits local speed variation to
[intermediate cell windows](intermediate_passages.md); it requires explicit
smooth-speed permission and grid reservations, and remains off by default.
`--candidate-guidance rejections` separately adapts initial and expanded
proposals from their own rejection history; see the
[parameter campaign and construction limits](parameter_campaign.md).

Interface labels and runtime messages are in English.

The default `physical_mission` generator constructs complete
individual missions directly through the coupled physical model, including an
explicit recovery primitive from the actual passage state. It supports one to
three cruise levels and constant or smooth airspeed references, including strict
and explicitly declared mixed profiles. Geometric intermediate-passage fitting
is unsupported and rejected explicitly. Select `--generator-mode geometric`
to use the historical generator, including geometric passage-window fitting.
See [physical construction, scope and verification](phase2_kinodynamic.md).
The editor's **Physical generator** checkbox selects the same mode; it is
disabled for planar/legacy flight. See the [migration guide](generator_migration.md)
for versioned shared scenarios and the separate PX4/Gazebo workflow.

The physical generator defaults to `--numerical-backend cpp`; run
`./scripts/build_native.sh` in an existing environment. Explicit
`--numerical-backend python` selects the independent reference implementation.
The native backend covers physical proposals, construction,
independent validation, catalogs and bounded joint selection. With `--workers 1`
the complete mission planner runs in one native call; multiple workers retain
Python's deterministic spawn scheduling. Both paths retain independent final checks;
the editor and shared PX4 scenario v3 records retain the explicit choice. See the
[native kernel guide](native_kernel.md) for supported modes and measurements.

`--execution-backend cpp` independently selects the compiled constrained local
controller and simulator integration for coupled 3D execution. Its default is
`python`; both geometric and physical plans are supported. Global observation
and final acceptance remain Python. This choice does not change the editor's
numerical-backend checkbox or the separate PX4/Gazebo workflow. Planar, legacy flight
and the historical legacy controller reject C++ execution explicitly.

## Preparing drones and destination

The preparation map remains an XY projection: select a drone, then click on
the map or enter x/y. Fields on the right let you edit:

- **Altitude (m)**: the drone's actual altitude at the start of simulation.
- **Cruise (m)**: an individual setting available in manual
  mode; displays **Auto** when the generator selects the altitude.
- **Heading (°)** and **Airspeed (m/s)**: heading and total
  airspeed setpoint (the field remains **XY speed** in legacy mode).

By default, the four drones already start on separate layers: **30, 55, 80 and
105 m**. These states represent aircraft already loitering, not an instantaneous
climb from the ground. Initial altitudes remain per-drone inputs: they describe
the actual state at planning time.

The global **Automatic cruise** checkbox is
enabled by default in the 3D launcher. Set **Cruise min (m)** and **max (m)**,
then click **Apply** or **Launch mission**.
The default range is **30–120 m**. The individual cruise-altitude field is
disabled in this mode. To return to a fixed level per drone, uncheck the box
and enter **Cruise (m)** for each aircraft. **Reset**
restores the startup mode and range.

Each 3D candidate contains its own cruise altitude selected within the common
range. The catalog alternates levels near the initial altitude (clamped to the
range) and pseudorandom levels within that range. Joint search selects one
candidate per drone while checking complete trajectories and transitions.
Selection therefore does not rely on independent draws assumed to be safe:
levels are validated together. The program retains a compatible combination
without claiming global energy, altitude or distance optimality.

The range constrains **cruise altitude**, not every altitude throughout the
mission: initial altitude and the XYZ target may lie outside it, provided they
remain within general limits (0–200 m by default). For example, a drone may
start at 30 m, cruise within 40–110 m, then reach the target at 30 m. Each
transition remains continuous and bounded. A single-value range such as
60–60 m is allowed.

The range, initial states and seed determine candidates reproducibly. The range
is never automatically widened to bypass failure. A narrow range or short
horizon may reduce available solutions. Selected altitudes are displayed in
the terminal and 3D legend, then exported in plans and the report.

The **Extra wait min/max** and **Approach spread (°)** fields retain their
functions. Angles are assigned automatically based
on XY positions relative to the target, globally minimizing distances to entry
gates on a common circle rather than following identifier order. A drone on the
right therefore favors a right-hand sector. Sectors remain distributed across
the full requested spread, with exactly one drone assigned to each. The map
updates arrows and identifiers after each move. This heuristic aims to limit
entanglement; safety of actual XYZ trajectories over time is always validated
separately.

For comparison with an older run, `--approach-assignment id` restores identifier
order; the default is `geometric`. Catalogs differ between these policies:
a solution found with one is not guaranteed with the other.
`0 / 0` removes additional waiting; the activation delay is common and separate.
The interface checks initial separation in XYZ, allowing two drones to occupy
the same map location if sufficiently separated vertically. Their subsequent
transitions remain subject to planner checks.

The 3D launcher's destination is a **common XYZ point** (30 m altitude by
default). Select **Target**, then set **Altitude (m)** along with x/y.
The **Target: common altitude** checkbox is enabled
by default. Two modes remain available for comparison:

| Mode | Arrival condition |
|---|---|
| Checked (default) | Reach a sphere around the same XYZ point. Select **Target**, set **Altitude (m)** and confirm. |
| Unchecked | Comparison mode: reach only the destination's XY disk, with unconstrained arrival altitude within the model limits. |

Without a common altitude, aircraft pass over the same horizontal position;
they do not reach the same 3D point. In common-XYZ mode, the planner prepares
altitude changes before arrival, a pass at the common altitude and a return
to the cruise layer afterward. Arrivals at the same point must be staggered
in time. Simultaneous departure remains a request, not a promise of feasibility
for every placement.

Closing preparation cancels the mission. Changes apply with **Apply** or
**Launch mission**; **Reset** restores the command-line scenario.

The timing row exposes **Horizon (s)** (total duration), **Arrival deadline (s)**
(optional absolute arrival deadline; **Auto** means no separate deadline) and
**After arrival (s)** (minimum post-passage time). A trajectory must fit its
complete recovery as well as this minimum time. Increasing horizon does not
relax an explicit deadline. Failure reports now distinguish pre-arrival
transition time, target arrival and recovery exceeding the horizon, and can
suggest a longer horizon without changing any setting automatically.

## Descent angle when passing the XYZ point

The global **Descent (°)** (descent angle) field sets a common angle for all
drones. Confirm it with **Apply** or **Launch mission**. The XYZ launcher
defaults to **30°** (`0°` for layered mode; the `Config` API retains an initial
angle of 0°). A positive value requests descent through the XYZ point, followed
by a pull-out and return to cruise altitude. This setting requires common-XYZ
target mode.

This is the **flight-path angle relative to horizontal**, not aerodynamic
pitch attitude. In point-mass mode, for total airspeed `V`:

```text
vz_passage = -V * sin(angle)
v_horizontal = V * cos(angle)
```

Thus, **12 m/s at 30° gives a 6 m/s descent rate and 10.39 m/s horizontally**.
Speed is a setpoint tracked by a controller, not a value imposed independently
of forces. Flight-path angle is also limited by propulsion, braking, load
factor, stall margin and room for the pull-out. Only in legacy mode does the
relationship remain `vz = -v_xy*tan(angle)`.

The profile reaches an intermediate altitude, establishes the requested vertical
speed, then holds the slope on the final segment. It passes the target altitude
with that vertical speed. The pull-out starts immediately after nominal passage:
vertical speed returns continuously toward zero before the return to cruise
altitude. All phases respect vertical-speed, acceleration, altitude and
separation limits.

The pull-out temporarily descends **below target altitude**. The XYZ point must
therefore be high enough to respect minimum altitude: the generator does not
instantaneously reset `vz` to zero. A cruise altitude too low to establish the
slope is also rejected. In automatic mode, level selection accounts for the
required height while remaining within the supplied range.
`descent_not_feasible` identifies a conflict involving vertical speed, cruise
height or pull-out clearance.

The independent checker verifies the nominal angle at arrival and at the
reintegrated first-entry event (**1°** tolerance for point-mass mode; legacy
retains its 0.5° sampled-contact check). The coupled execution observer uses the
same event-localization algorithm and unchanged 1° tolerance. The report
distinguishes nominal closest-approach angles from nominal and actual contact
angles; see [contact diagnostics](mission_pipeline.md).
In point-mass mode this is the **air-relative** flight-path angle, not the
ground-relative angle under wind. A larger error produces exit code 3.
Checking one run does not prove angle tracking under every disturbance.

```bash
# Airborne pass at 10°, followed by a pull-out
.venv/bin/python examples/trajectory_3d.py --descent-angle 10

# Requested setpoint: 30°, coupled model by default
.venv/bin/python examples/trajectory_3d.py --descent-angle 30

# Same 10° setting with immediate departures and an explicit XYZ target
.venv/bin/python examples/trajectory_3d.py --descent-angle 10 \
  --target 500 300 --target-altitude 30 --cruise-range 40 110 \
  --departure-delay 0 0 --activation-time 0 --approach-spread 180 --horizon 150
```

## Examples

```bash
# Common XYZ target, immediate departures, approaches spread over 180°
.venv/bin/python examples/trajectory_3d.py \
  --departure-delay 0 0 --activation-time 0 --approach-spread 180 \
  --target 500 300 --target-altitude 30

# Explicit comparison: pass at separate altitudes, with no common XYZ goal
.venv/bin/python examples/trajectory_3d.py --arrival-mode layers

# Change initial layers: 40, 70, 100 and 130 m
.venv/bin/python examples/trajectory_3d.py --altitude 40 --altitude-spacing 30

# Common range: the algorithm selects a cruise altitude per drone
.venv/bin/python examples/trajectory_3d.py --cruise-range 40 110 \
  --target 500 300 --target-altitude 30 --departure-delay 0 0 \
  --activation-time 0 --approach-spread 180

# Choose an initial altitude for each drone
.venv/bin/python examples/trajectory_3d.py \
  --drone-altitude 30 --drone-altitude 55 --drone-altitude 80 --drone-altitude 105

# Same XYZ destination, with climbs/descents from the layers
.venv/bin/python examples/trajectory_3d.py \
  --target 500 250 --target-altitude 30 --horizon 150 --approach-spread 180

# Cruise altitudes different from initial altitudes
.venv/bin/python examples/trajectory_3d.py \
  --cruise-altitude 40 --cruise-altitude 65 --cruise-altitude 90 --cruise-altitude 115 --descent-angle 0

# Manual mode, with cruise altitude initially equal to departure altitude
.venv/bin/python examples/trajectory_3d.py --manual-cruise --descent-angle 0

# Vertical limits and headless execution
.venv/bin/python examples/trajectory_3d.py --no-gui \
  --max-climb-rate 3 --max-vertical-acceleration 1 --max-altitude 200
```

Options `--airspeed`, `--drone-speed`, `--drone-position`, `--target`, `--seed`,
`--horizon` and clock/wind disturbances remain available. `--cruise-range`,
`--cruise-altitude` and `--manual-cruise` are mutually exclusive. Individual
cruise settings are not used while an automatic range is active. The Python
API accepts the range through
`Config(dimensions=3, cruise_altitude_range=(40, 110), ...)`; `None` preserves
manual mode for existing callers. Altitudes and vertical speeds are in meters
and meters per second.

## Geometric proposals and verification

The profiles below remain the source of **curve proposals**. In point-mass
mode they are retimed, tracked using the coupled flight engine and then
validated on the actually integrated trajectory. They are not directly
executable plans. In legacy mode they are executed as generated.

On each primitive of duration `dt = 0.2 s`, vertical acceleration is constant:

```text
z(t+u)  = z(t) + vz(t)*u + 0.5*az*u²
vz(t+u) = vz(t) + az*u
```

Each transition is a trapezoidal or triangular profile with zero vertical speed
at its endpoints. Durations are aligned to the grid by adjusting acceleration
without exceeding limits. Defaults are:

- `vz ≤ 3 m/s`, `vz ≥ -10 m/s` in the launcher (the API without
  `max_descent_rate` keeps a descent limit equal to the climb limit);
- `|az| ≤ 1 m/s²`;
- altitude between 0 and 200 m.

Profiles remain at initial altitude until departure and reach cruise altitude
before arrival. In common-altitude mode, the final transition's start varies
between candidates. Profiles that do not fit within the horizon are rejected,
without teleportation, additional waiting or changes to limits. All aircraft
remain present after arrival until the horizon.
In automatic-cruise mode, return toward the selected level begins at nominal
target passage, without manual mode's two-second level segment. For a level
pass, vertical speed starts at zero and acceleration remains bounded; a
descending pass instead performs the pull-out described above. This avoids
unnecessarily keeping all aircraft at target altitude after passage. The
pre-departure waiting interval is unchanged.

For a common XYZ destination, candidates explore several approach lengths
(base radial radius multiplied by a factor from 1 to 2.5) and lateral detour
amplitudes (one to two times the base parameter). They can therefore have
different flight durations even with simultaneous departures. These variations
change neither destination, assigned approach side nor waiting interval.
Candidates exceeding the horizon are rejected. The base radius remains at
least 120 m and six times the minimum turn radius.

Search uses backtracking with forward checking: once a candidate is selected,
incompatible candidates for remaining drones are immediately removed. A branch
is abandoned as soon as any drone has no remaining option. Order remains
deterministic; the retained combination is checked again with the same margins.
The budget limits explored branches, not CPU time. `--max-search-nodes` adjusts
this budget. Failure within a finite catalog remains possible and does not
authorize reducing margins, target radius or destination constraints.

In legacy mode, independent verification reconstructs vertical primitives and
checks speed, acceleration and altitude, including extrema within steps.
Distances use `(x,y,z)`. Each plan's speed bound is
`V = sqrt(v_xy² + max(|vz|)²)` in legacy mode. In point-mass mode, the conservative
bound is the global airspeed limit, **16 m/s**, conditional on envelope
compliance. The same margin criteria apply:

```text
required_nominal_distance = separation + 2*spatial_error + (V_i+V_j)*clock_error
continuous_distance >= sampled_minimum - (V_i+V_j)*dt/2
```

The configured spatial error is a **3D Euclidean norm**, not an independent
2 m tolerance on each axis. Bounds cover climbs, descents and exits, so layer
crossings are also checked. Vertical spacing between layers is insufficient
to validate convergence to a common altitude: all pairs are checked again
over the entire horizon.

The controller receives only its own plan and local state. In point-mass mode
it commands thrust, airbrake, load factor and roll; legacy XY/Z controls remain
independent. The observer evaluates execution on a grid ten times finer,
without feedback to controllers. Synthetic wind remains **horizontal**;
bounded position bias/noise is optional, while other state measurements remain
ideal. Vertical wind and lift loss are not simulated. The default coupled
executor estimates horizontal drift from its own motion and protects combined
held commands. See [disturbed tracking](robust_tracking.md) for profiles,
clock drift, diagnostics, acceptance criteria and remaining limitations.

## 3D viewer and exports

The window shows 3D trajectories and loiter circles, current positions,
destination and approach directions. Drag the mouse to rotate the view;
**Pause / Play** and **Time** provide replay controls.
Plots show 3D distances, altitudes, spatial errors and, in point-mass mode,
total airspeed. Vertical scale is visually exaggerated to distinguish layers;
coordinates and computed distances remain in meters without exaggeration.

Each run creates a `results/trajectory3d/<date-time>/` folder containing the
manifest, a plan per drone, CSV telemetry (including altitude and vertical
speed), report and `overview.png`. The `swarm-flight3d-v3` schema contains
physical states and controls, their units, engine parameters and speed bound.
The CSV adds airspeed, flight-path angle, bank and ground speed. Exported XY/Z
primitives are a compatibility projection, not a second independent flight
engine.

Exported 3D distances are numerically integrated on the observation grid;
historical detour ratios remain horizontal. The report includes
`cruise_altitude_range_m` and `selected_cruise_altitudes_m`. Each plan contains
its selected level in `cruise_altitude_m`. The report also includes
`requested_descent_angle_deg`, `nominal_descent_angles_deg`,
`actual_descent_angles_deg` and `descent_angle_verified_in_execution`.
The green band on the altitude plot represents the allowed cruise range.

Safety remains conditional on the stated bounds and limited to the simulated
horizon. RK4 integration and flight-envelope checks are **numerical**; no
formal integration-error bound is certified. Known static cylinders and boxes
are supported with full-horizon clearance checks; see [obstacles](obstacles.md).
This model includes neither terrain meshes, a battery budget, takeoff nor landing.
Reaching an altitude
of 0 m represents only a coordinate in this model, not a landing procedure.
Full flight dynamics and hardware validation remain separate stages.
