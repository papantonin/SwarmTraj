# Coordinated 2D trajectory prototype

The launcher now opens 3D mode by default, as described in the
[3D guide](trajectory_3d.md). To reproduce the planar scenarios in this document,
add `--dimensions 2` to the commands below.
The launcher also defaults to adaptive catalogs and MRV search; see
[adaptive planning and diagnostics](adaptive_planning.md). Model and safety
bounds below are unchanged. `--planning-strategy fixed --search-order id`
retains full-catalog generation; add `--selection-policy first` for historical
first-feasible selection. See [margin selection](margin_selection.md) for the
new CLI default.

Interface labels and runtime messages are in English.

## Running the scenario

From the repository root, the existing virtual environment is ready to use:

```bash
source .venv/bin/activate
python examples/trajectory_2d.py --dimensions 2
```

The first window is used to prepare the scenario:

1. Select **Drone 0**, **Drone 1**, etc. or **Target** in the list on
   the right.
2. Click on the map to move the selected item, as often as necessary. The
   selected point is the **drone's initial position**, not its loiter-circle
   center. Its circle moves with it.
3. For precise coordinates or positions outside the current view, enter **x**
   and **y**, then click **Apply**. A drone's **heading** can
   also be changed: 0° points east, 90° north. Its circle adapts to the heading.
   **XY speed (m/s)** sets the selected drone's planar airspeed, between 8 and
   16 m/s in the current model (12 m/s by default). Confirm with **Apply**
   before selecting another drone.
4. **Fit all** displays all items; **Reset**
   restores the positions and target supplied at startup. Toolbar zoom and pan
   controls let you explore the map.
5. Click **Launch mission** to calculate and then visualize
   the trajectories.

The global **Extra wait min (s)** and **max (s)** fields set the additional waiting
interval before leaving the circles. The default is 0–22 seconds. Enter **0 / 0**
to remove this wait and request simultaneous departures; **0 / 2** permits at
most two additional seconds; **2 / 2** imposes the same two-second delay for all.
Bounds must be finite and satisfy `0 ≤ min ≤ max`. They are applied when the
mission starts and restored by **Reset**.

The common activation delay (3 s by default, `--activation-time`) is separate.
Each candidate's departure time is:

```text
release_time = activation_time + Uniform(wait_min, wait_max)
```

Guidance switches on the first simulation step at or after that time (0.2 s
resolution). With `0 / 0`, all drones switch on the same step. Collision checks
are unchanged: a very short interval can reduce the available combinations.
The planner rejects the mission if it cannot find a valid combination; it adds
no waiting outside the configured interval to bypass this constraint. The
report exports departure times and the configured interval.

The global **Approach spread (°)** (approach spread) field controls the spread of
entry sides around the target. It applies to the whole swarm and is confirmed
with **Apply** or **Launch mission**:

| Value | Behavior |
|---|---|
| 0° (default) | Original generation, with no assigned arrival side. |
| 90° | Arrival sides distributed over a 90° arc. |
| 180° | Arrival sides distributed over a semicircle. |
| 360° | Sides distributed over the full circle: four directions 90° apart for four drones. |

The arc is centered on the direction from the target to the mean initial
position. For 360°, the circle endpoint is not duplicated. Assignment is
**geometric**, not based on identifier order: drones to the right of the target
favor right-hand sectors, and vice versa. A global assignment minimizes the
sum of XY distances between initial positions and entry gates placed on a
circle with common radius
`R = max(120 m, max_i(6*v_i/max_turn_rate(v_i)))`.
The solver uses [SciPy's linear assignment](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.linear_sum_assignment.html).
Each sector receives exactly one drone without changing the requested spread.
Selection remains deterministic; positions are sorted geometrically before
solving, and identifiers only break ties between identical positions. A single
drone receives a single direction regardless of spread. Colored arrows near
the target, labeled with drone identifiers, show the requested entry directions
and update after repositioning. This geometric distribution provides no
cryptographic unpredictability guarantee.

Optimality applies only to these straight reference links: actual approach
radius and detours may vary between candidates. Fewer crossing links may help
the search but guarantee neither shorter computation time in every scenario nor
collision avoidance. Curved trajectories, timing and separation margins are
still fully checked. `--approach-assignment id` replays the old identifier-based
assignment for comparison; the default is `--approach-assignment geometric`.
Changing assignment changes the catalogs: a previously validated case may
become `catalog_exhausted` with the same budget, without proving physical
infeasibility. The program never silently falls back to a different assignment.

To actually change approach side, trajectories pass through an offset entry
gate and then an alignment point before heading to the target. The approach
radius is at least 120 m and six times the drone's minimum turn radius. The two
pseudorandom points are generated toward this entry. A candidate is accepted
only if its first nominal sample inside the target disk is on the assigned
side and points inward, with a 15° tolerance on both side and heading. An early
target passage from the wrong side is therefore rejected even if the alignment
points were correctly placed. This check uses nominal samples; it does not
formally certify every possible tangential contact between samples.

Spatiotemporal separation is still checked throughout the mission, including
detours and post-arrival exit. A large spread may require a longer horizon;
the program rejects the mission if no combination is found, without silently
reducing the spread. The report includes requested sides, angles actually
observed on entering the disk and their differences. Sector compliance during
disturbed execution is checked separately from safety: position and clock
bounds alone do not guarantee this angular tolerance. An error greater than
15° produces exit code 3.

Fields for the last selected item are applied at launch even if **Apply**
was not clicked. Insufficient initial separation is shown in red and prevents
launch. Other constraints remain the planner's responsibility: freely placing
items does not guarantee a combination will be found within the horizon and
search budget. Rejection diagnostics are printed in the terminal and exported
in `mission.json`. Closing the preparation window cancels the mission.

The selected speed remains constant during flight but may differ between
drones. Changing speed preserves the loiter circle's radius and direction by
adjusting its turn rate (`yaw_rate = speed / radius`, with the appropriate
sign). A speed incompatible with maximum bank and the selected radius is
rejected: reduce speed or increase `--loiter-radius`. Temporal separation
margins and travel times use each aircraft's own speed. Speeds are saved in
plans and displayed on the preparation map and simulation legend.

On a fresh installation, `bash scripts/setup.sh` also installs this prototype's
dependencies. To add only the prototype to an existing virtual environment:
`python -m pip install -e '.[trajectory2d]'`.

The window shows the four aircraft, their four separate loiter circles (labeled
**Loiter 0** through **Loiter 3**), dashed nominal trajectories, executed
traces, protection disks, destination and inter-aircraft distances.
**Pause / Play** (pause/play) and the **Time** (time) slider allow mission
replay. Zoom and pan are available in the Matplotlib toolbar. Playback runs
at four times real time by default.

```bash
# Keep the supplied positions and start the simulation directly
python examples/trajectory_2d.py --no-edit-scenario

# Distribute arrivals around the entire target
python examples/trajectory_2d.py --approach-spread 360 --horizon 240

# Simultaneous departures after the common 3 s activation delay
python examples/trajectory_2d.py --departure-delay 0 0

# Immediate departures at t=0, with no common or random wait
python examples/trajectory_2d.py --departure-delay 0 0 --activation-time 0

# Limit the additional wait to 2 seconds
python examples/trajectory_2d.py --departure-delay 0 2

# Common initial airspeed (m/s), subsequently editable in the editor
python examples/trajectory_2d.py --airspeed 15

# Different speeds, in identifier order 0, 1, 2, 3
python examples/trajectory_2d.py --no-gui \
  --drone-speed 10 --drone-speed 12 --drone-speed 14 --drone-speed 16

# Supply initial positions and destination without an interface (meters)
python examples/trajectory_2d.py --no-gui \
  --drone-position -15 -40 --drone-position 40 -15 \
  --drone-position -65 40 --drone-position 40 15 --target 140 50

# Select the destination by clicking on an XY snapshot map
python examples/trajectory_2d.py --pick-target

# Prefill the destination in the editor and change the seed
python examples/trajectory_2d.py --target 180 -60 --seed 19

# Change individual circles (radius and center spacing in meters)
python examples/trajectory_2d.py --loiter-radius 30 --loiter-spacing 100

# Test clock offsets distributed between -0.1 and +0.1 s and synthetic wind
python examples/trajectory_2d.py --clock-offset 0.1 --wind 0.05

# Deliberately exceed the planned clock-error bound
python examples/trajectory_2d.py --clock-offset 1 --no-gui

# Headless execution with PNG figure export
python examples/trajectory_2d.py --no-gui
```

Each run creates a new folder under `results/trajectory2d/` containing
`mission.json`, one `drone_<id>.json` file per aircraft, `report.json`,
`telemetry.csv` and `overview.png`. `--output path` accepts only an empty folder
to prevent mixing results from different missions. A planning failure produces
`mission.json` and `planning_report.json`, without execution artifacts.

## Model and timeline

The snapshot at `t=0` contains the positions, headings, airspeeds and turn rates
of four aircraft. Without editor or command-line changes, they occupy **four
separate rings** of 25 m radius. Their centers are `(-40, -40)`, `(40, -40)`,
`(-40, 40)` and `(40, 40)` meters: neighboring centers are 80 m apart, leaving
at least 30 m between circumferences regardless of aircraft phase. Each
aircraft stays on its own ring until departure. For other drone counts, centers
are arranged on a recentered grid. `--loiter-spacing` must exceed twice
`--loiter-radius`; safety margins and departures are still checked by the
planner and do not follow from this geometric condition alone.
The default destination is `(140, 50)`; aircraft share a common altitude not
included in XY calculations. The planner also accepts a list of custom
`Snapshot` objects. `separate_loiter_snapshot()` builds the default scenario;
`ring_snapshot()` remains available for common-ring tests.

The generator receives no new telemetry. Aircraft continue their initial
loiter motion during a virtual activation delay (3 s by default), followed by
an additional wait drawn from the configured interval (0–22 s by default,
disabled with `--departure-delay 0 0`). This prototype delay is a simulation
parameter, not a measurement or guarantee of actual computation/transmission
time. No radio link is implemented; plans are assumed to be loaded before
activation.

Each candidate consists of circular loiter, two pseudorandom guidance points,
optional entry and alignment points for the assigned arrival side, passage
through a 6 m target disk and an exit. All aircraft remain simulated until the
common 90 s horizon: reaching the target neither removes nor stops them.

The model is kinematic, with constant airspeed per drone, 12 m/s by default.
On each primitive, turn rate varies linearly:

```text
heading(t+u) = heading(t) + yaw_rate(t)*u + 0.5*alpha*u²
yaw_rate(t+u) = yaw_rate(t) + alpha*u
position_dot = speed * [cos(heading), sin(heading)]
```

Positions are integrated using Fresnel integrals (exact circular arcs when
`alpha=0`), not straight segments between samples. Turn rate and its
acceleration are bounded to respect the model's bank and roll-rate limits:

```text
bank = atan(speed * yaw_rate / 9.81)
|yaw_rate| <= 9.81 * tan(max_bank) / speed
|alpha| <= 9.81 * max_roll_rate / speed
```

A separate check reconstructs the primitives, checks limits and verifies
arrival. This model includes neither full aerodynamics, geofencing,
takeoff/landing nor an energy model. The horizon is a time budget, not an
endurance validation. Variable speeds and continuation beyond the horizon
remain to be added.
Known static obstacles are supported as XY footprints in planar mode; see the
[obstacle guide](obstacles.md) for configuration and clearance checks.

## Coordination and scope of the guarantee

The generator uses `PCG64` with a seed derived from `(mission_seed, id)`.
Identical inputs, software versions and numerical conventions reproduce the
plans. The manifest contains configuration, initial states, plans and a SHA-256
fingerprint; individual files contain no state or plan belonging to another
drone. The fingerprint identifies content but does not authenticate it.
The PRNG provides diversity and reproducibility, not cryptographic
unpredictability.

The generator produces up to 28 candidates per drone within at most 90 attempts.
A joint backtracking search selects one candidate per drone, caching pairwise
compatibility. This is not a simple greedy selection. The search budget is a
node count independent of CPU time; a finite catalog is not complete in
continuous space.

For speed `v`, position error `epsilon` and temporal phase error `delta`, the
required nominal distance between two drones is:

```text
required = separation + epsilon_i + epsilon_j + v_i*delta_i + v_j*delta_j
```

Defaults are 8 m physical separation, 2 m spatial error per aircraft and
0.15 s temporal error per aircraft, giving 15.6 m required nominal separation.
The spatial error bound applies to deviation from the plan evaluated at local
time. Temporal phase may also represent a progress delay, provided that delay
is actually bounded by `delta`.

To avoid missing a collision between samples spaced by `dt`, the checker uses
the Lipschitz bound:

```text
continuous_min_distance >= sampled_min_distance - (v_i + v_j)*dt/2
```

Every selected combination is checked again. The guarantee is conditional on
these error bounds over `[0, horizon]`. Static obstacle clearance, when enabled,
is checked separately with its own margins. Calculations
use floating-point arithmetic and a numerical margin; this is not a formal
software proof or real-flight certification.

The displayed envelopes have radius `separation/2 + epsilon + v*delta` around
the nominal position. Their non-overlap represents the conservative criterion
used, not an aircraft's exact physical wingspan.

## Autonomous execution and disturbances

An `Executor` owns only its own plan, limits and local state. Guidance corrects
deviations from the time-indexed reference. Local sensors are assumed perfect.
Clock offsets change the time used for guidance, never the initial physical
position. Optional clock drift grows with time. Injected wind is deterministic,
bounded in norm and constant over each integration step.

The global observer collects positions for tests and display but sends nothing
back to controllers. It checks distances and deviations on a grid ten times
finer, using Lipschitz bounds between samples. The report distinguishes validity
of the plan's assumptions from separation actually verified in the disturbed
run: a violated assumption does not necessarily mean a collision occurred.

Corrective controls alone do not guarantee that errors will remain within the
tube for every future wind disturbance: this property is evaluated for each
run. Proving controller robustness is a later step. Contract violations are
reported; no improvised return or emergency stop is presented as safe.

The report also exports arrival times, traveled distances and detour ratios
including loiter. Advanced diversity/predictability metrics and statistical
experiments over many seeds remain to be extended.

## Diagnostics and tests

- `validated`: combination validated under the stated assumptions.
- `initial_separation_violation`: insufficient distance already at time zero.
- `initial_margin_not_certified`: the selected robust margin cannot be validated initially.
- `invalid_initial_state`: speed or turn rate incompatible with the model.
- `destination_unreachable_within_horizon`: even the direct distance exceeds the model's maximum time-limited reach.
- `no_candidate`: no complete candidate found for at least one drone.
- `catalog_exhausted`: no compatible combination in the generated catalog.
- `search_budget_exhausted`: search stopped by its budget, with no conclusion of infeasibility.

The last three statuses do not prove physical infeasibility. Exit codes: 0 for
a run with arrivals, tracking tubes and separation verified; 2 for rejected
planning; 3 for execution that was not fully validated.

```bash
./scripts/test.sh
```

Tests check integration against independent quadrature, reproducibility, seed
changes, primitive constraints, collisions between samples, isolated drone
tracking and disturbances, and several rejection diagnostics. The quadrotor
model and ring example remain accessible through their existing commands.
