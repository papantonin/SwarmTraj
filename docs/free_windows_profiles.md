# Free-window-guided generation and richer 3D profiles

The new features are opt-in. With them disabled, the historical single-level,
constant-setpoint candidate stream is retained. Four-aircraft tests compare
serial/spawn generation, immutable payloads, full-horizon reservations, physical
execution and actual attainment of the requested levels.
The default profile policy remains strict. The explicit
[mixed-profile policy](mixed_profiles.md) instead permits simple and rich families
within one adaptive catalog; its level count is a maximum, not a requirement.

## 1. Use reservation windows before generating alternatives

Enable `--reservation-policy grid --interval-guidance safe_windows`.
Unlike reservation filtering alone, adaptive catalog expansion now computes
free **departure** intervals before generating the next physical candidates.

For each expanded aircraft, use one existing valid parent plan and offline
reference plans for the other aircraft. During margin refinement these are the
current feasible incumbent; during feasibility expansion they are the first
accepted plan in each nonempty catalog. References are proposal forecasts,
not irrevocable reservations or assumptions about the final selected fleet.

For a shared cell, let the parent's moving occupancy interval be `[a,b]`, its
release time be `r`, and the other aircraft's reserved interval be `[c,d]`.
Uniformly shifting the moving parent by `new_release-r` would cause overlap iff

```text
new_release in [r+c-b, r+d-a]
```

Intersect these forbidden intervals with the configured release bounds, merge
them and take their complement. Closed occupied boundaries receive an extra
1e-6 s numerical exclusion guard. One free interval is chosen deterministically
from the aircraft's seeded RNG, then a release is drawn within it. The new
release is supplied to the geometric generator **before** rollout. Mandatory
approach gates, target, activation epoch and departure bounds remain intact.
Guided attempts alternate with ordinary exploration; every attempt counts
toward the same cumulative allowance. If no free interval is predicted or hint
construction exceeds `--interval-max-constraints 50000`, normal exploration
continues. No empty-window result proves physical infeasibility.

Crucially, changing release time also changes loiter phase, path geometry and
flight duration. Therefore this interval calculation is a **forecast**, not a
certificate that the regenerated plan respects the reference calendar. Every
new plan is rebuilt and independently physically/obstacle validated, then joint
search rechecks all pairs and cell/time reservations. No accepted trajectory is
retimed after its separation check. The known holding prefix is not assumed to
shift; it is included in the full-horizon validation as before.

A zero-width departure interval is never widened. With fixed simultaneous
departures, this temporal heuristic may have no useful degree of freedom;
geometric and profile alternatives can still be explored. This is not a full
SIPP search, continuous-space completeness guarantee, or solver for every
possible passage window. Initial catalogs still build independently; interval
guidance begins once nonempty reference catalogs exist.
The separate [intermediate passage heuristic](intermediate_passages.md) can fit
speed bumps without changing departure time. It is opt-in and does not replace
the independent physical/reservation checks.

Diagnostics in `planning_report.json`, per catalog:

- `interval_contexts`: parent candidate ID, reference IDs, free intervals,
  constraint count and hint-budget status;
- `interval_proposals`: candidate ID, chosen release interval and requested
  release, and whether individual validation accepted the generated plan.

Individual acceptance does not mean joint compatibility or final selection.
These reference IDs/calendar hints remain ground-side data and are not placed
in an individual aircraft's plan.

## 2. Multiple cruise levels

`--cruise-levels 2` requests two distinct cruise levels; values 1–3 are accepted.
The default is 1. Multiple levels currently require coupled 3D flight and an
automatic cruise range. `--cruise-level-hold 3` sets the proposed hold duration,
with a minimum of 2 s.

The geometric reference first reaches its primary cruise level. It then
performs rest-to-rest, rate/acceleration-bounded transitions to the additional
levels, holds there, and returns to the primary level before the original final
approach/descent. Proposed increments are between 5 and 15 m; all declared
levels must differ by at least 5 m and remain inside the requested cruise range.
Recovery still returns to the primary level by the common horizon.

Altitude is never assigned instantaneously. The existing grid-snapped vertical
transition primitives are integrated continuously, retimed into total airspeed,
then tracked by the coupled physical model. Insufficient space/time rejects the
proposal with `insufficient_time_or_range_for_cruise_levels`; it never silently
reduces the requested number of levels or expands the cruise range.

Independent validation checks the declared levels and requires actual ordered
attainment before arrival: at least one second of consecutive grid states within
0.5 m of each level and 0.2 m/s of zero vertical speed. Failure produces
`validation_cruise_level_not_reached`. Physical-envelope, dense integration,
obstacle, passage, final recovery and pair-distance checks are retained.
Attainment is numerical evidence with these tolerances, not an invariant-set
proof of an exact altitude hold.

Plans export immutable `cruise_levels_m` metadata. Reports include
`selected_cruise_levels_m`; the 3D legend shows the selected levels and the
altitude plot shows the integrated motion.

## 3. Smooth airspeed-reference variation

Enable `--speed-profile smooth`. `--speed-profile-range 10 14` gives the
reference bounds in m/s (also the launcher's default when smooth mode is
selected). All initial airspeeds must lie inside this range. Physical aircraft
speed limits remain the existing 8–16 m/s by default; the reference range is
**not** a new hard actual-speed envelope.

The profile keeps the initial setpoint throughout loiter, final approach and
recovery. On a pre-approach geometric-time interval `[a,b]`, it introduces

```text
V(tau) = V_initial + A sin²(pi (tau-a)/(b-a))
dt/dtau = geometric_speed(tau) / V(tau)
dV/dt = dV/dtau * V(tau) / geometric_speed(tau)
```

The bump starts four geometric seconds after release and ends 18 geometric
seconds before nominal passage. It is C1, with zero speed-reference derivative
at both joins; jerk is not constrained. Its sign and amplitude use a separate
PCG64 stream derived from mission seed, drone ID and attempt ID. The amplitude
is conservatively limited by `--speed-profile-acceleration 0.3` m/s² and the
requested reference bounds. A proposal with too little time/range for a
meaningful bump is rejected, not instantaneously accelerated or relabeled.

Geometry is retimed **before** flight rollout. Yaw and flight-path derivatives
are transformed with the same clock; bank/lift references use total airspeed.
Axial acceleration feedforward is applied through thrust/braking, and actual
states are integrated without clipping. Saturation or coupled vertical/stall
limits may make a proposed profile untrackable; ordinary independent validation
rejects it. Faster setpoints during climbs can therefore be rejected more often
than slower ones; no uniform distribution of accepted maneuvers is claimed.

Independent checks verify speed-reference shape, finite values, bounds,
initial/final setpoint, sampled reference-acceleration bound and meaningful
reference/actual speed variation. Actual physical-envelope checks remain
separate and unchanged. Requested acceleration bounds do not certify a jerk
bound or exact tracking under disturbances.

The optional immutable array `speed_reference_m_s` is exported in each enabled
plan. The executor still needs only its own integrated states/controls, local
clock and measurements; it never queries the fleet calendar. Legacy constant
profiles omit this new metadata. Actual airspeed is already plotted and
exported in telemetry.

## Running the combined four-aircraft scene

```bash
.venv/bin/python examples/trajectory_3d.py --num-drones 4 \
  --target 500 500 --horizon 240 --approach-spread 180 \
  --obstacles examples/scenarios/static_obstacles.json \
  --prepare-in-flight --activation-time 30 --planning-latency 5 \
  --reservation-policy grid --interval-guidance safe_windows \
  --cruise-levels 2 --speed-profile smooth --speed-profile-range 10 14 \
  --initial-candidates 2

# Same snapshot/constraints: baseline, windows only, profiles only, combined
.venv/bin/python scripts/benchmark_intervals_profiles.py

# Additional seeds and repeatable timing trials, without execution cost
.venv/bin/python scripts/benchmark_intervals_profiles.py --seeds 7 19 \
  --no-simulate --repeats 3
```

The preparation duration of 5 s is explicitly simulated; omit
`--planning-latency 5` to gate readiness on measured planning wall time.
Neither option guarantees a real network/computation deadline. Increasing the
example horizon to 240 s is explicit; the planner never extends it itself.

## Observations and limits

The first combined CLI run passed all nominal execution checks with four
aircraft, the unchanged large obstacles and a 180-degree approach spread:

- Minimum sampled separation: **51.17 m**.
- Minimum sampled obstacle-surface distance: **28.44 m**.
- Selected levels, in drone-ID order: `63.95/69.07`, `63.95/72.51`,
  `80.00/68.17`, `105.00/96.55` m.
- Actual minimum/maximum airspeeds across the fleet: approximately **10.16–13.33 m/s**.
- Fifteen interval-guided attempts, three individually accepted. These guided
  alternatives were not selected in this particular final incumbent.

Artifacts: `results/trajectory3d/20261025-233212-397989/`.
This is nominal model validation, not a guarantee under arbitrary disturbances.

The paired four-mode campaign on the same seed-7 snapshot also passed every
nominal execution check. Proposal counts were 17 (baseline), 17 (windows),
57 (profiles) and 59 (combined). Windows-only selected one interval-generated
alternative. Thus the features are exercised, but **this campaign does not
demonstrate fewer proposals or a consistent speedup**. Rich profiles broaden
the family of flyable paths at an additional validation cost. The profiles-only
run had a larger obstacle margin than combined, despite both being valid;
the existing objective prefers inter-aircraft margin, not obstacle-margin
optimality. Raw results:
`results/benchmarks/intervals-profiles-validation-20261026.json`.
Timings in that campaign overlap other local verification work and must not be
interpreted as isolated performance measurements.

A second seed-19 campaign compared baseline, windows and combined modes.
All three passed every nominal execution check. Proposal counts were
25 / 23 / 71 respectively: a small generation-work reduction for windows-only
in this scene, not a general performance guarantee. Raw results:
`results/benchmarks/intervals-profiles-seed19-20261026.json`.
The actual Tk replay with levels and varying airspeed was exercised as well:
`results/rich-profiles-gui-smoke/20261025-233747-144083/`.

With new features disabled, all four individual plans exactly matched the
historical combined-preparation scene in
`results/trajectory3d/20261025-221013-698944/mission.json`. New configuration
fields/diagnostics still legitimately change whole-mission manifests.

The full regression run passed 378 tests (four existing Gymnasium float32
warnings). A subsequently added disabled-guidance unit case and the focused
interval/CLI tests passed as well. All seven requested comparative nominal
executions passed. These checks establish the exercised behavior only, not
continuous-space completeness, real-aircraft certification or universal
disturbance robustness.
