# Arrival deadlines and mission horizon

The horizon is the **total simulated time from the frozen snapshot**, not the
planning computation budget and not just flight time to the destination.
It includes loiter, approach, target passage, pull-out, cruise recovery and
onward flight. Every drone remains present through the common horizon.
With [in-flight preparation](mission_pipeline.md), this same clock starts at
designation and includes modeled computation, loading and loiter; it is never
reset at activation.

## Independent controls

| CLI option | Default | Meaning |
|---|---|---|
| `--horizon` | 90 s | Total simulation and separation-validation interval |
| `--arrival-deadline` | Unset | Optional absolute latest target-passage time, measured from the snapshot |
| `--post-arrival-time` | 5 s | Minimum time remaining after passage; does not replace the full recovery check |

The editor exposes **Horizon (s)**, **Arrival deadline (s)** (arrival deadline)
and **After arrival (s)** (minimum post-passage time). Enter **Auto** or leave
the deadline blank to remove the separate deadline. Changes apply with
**Apply** or **Launch mission**; reset restores command-line values.

For horizon `H`, optional deadline `D`, post-arrival time `P` and clock bound
`epsilon_t`, each nominal arrival must satisfy:

```text
t_arrival <= min(D, H - P) - epsilon_t
# With no D: t_arrival <= H - P - epsilon_t
```

`D` must be positive and no later than `H`. `P` must be nonnegative. An
oversized `P` can leave no admissible arrival time; it is never silently reduced.
The deadline concerns nominal target passage with a clock guard. Execution
separately checks its first observed entry into the arrival sphere. Tracking
and numerical assumptions still apply; this is not a real-time proof.

**A five-second post-arrival setting does not promise a five-second pull-out.**
The complete altitude recovery must independently finish before `H`, even if
it needs much longer than `P`. Conversely, increasing `H` cannot override an
explicit earlier `D`. The full-horizon separation and obstacle checks remain.

```bash
# Four drones, arrival by 130 s, with at least 20 s left after passage.
# Physical cruise recovery may require more than these 20 seconds.
.venv/bin/python examples/trajectory_3d.py --num-drones 4 \
  --obstacles examples/scenarios/static_obstacles.json --approach-spread 180 \
  --target 500 500 --horizon 180 --arrival-deadline 130 --post-arrival-time 20
```

## Diagnostics

Coarse rejection categories remain compatible with earlier reports. Each
catalog additionally exports `rejection_details`, including:

- `target_not_reached_within_horizon`: geometric guidance did not reach the
  destination during the sampled horizon; the generator cannot infer a future
  arrival or prove that the path would ever converge.
- `post_arrival_window_exceeds_horizon`: arrival leaves insufficient observation
  time, including the clock guard.
- `arrival_deadline_exceeded`: the separate deadline fails, including after
  coupled-flight retiming.
- `climb_cannot_finish_before_arrival`: the initial altitude transition does
  not fit before arrival.
- `insufficient_time_before_descent` or `insufficient_time_before_target_altitude`:
  altitude transitions overlap before passage.
- `recovery_exceeds_horizon`: the proposed pull-out/return to cruise does not
  finish within the horizon.

`diagnostics.timing` in `planning_report.json` contains these counts, the
effective nominal arrival limit, per-drone observed horizon requirements,
advisory text and, on success, departure/arrival/recovery schedules.
`geometric_template` additionally distinguishes a conservative geometric
recovery-headroom limit from the coupled preflight model; see the
[parameter campaign](parameter_campaign.md). This advisory changes no inputs
and is not a physical infeasibility proof.
Recovery completion is descriptive grid-based metadata: the earliest
post-passage sample settled at cruise through the endpoint, within 0.2 m
and 0.1 m/s. This does not replace independent physical validation.

When temporal rejections occur and no solution is found, a heuristic suggests
a longer horizon, at least 1.5 times the current value and covering recorded
profile hints, rounded upward to ten seconds and the simulation grid.
It changes **nothing automatically**, never extends a deadline and is **not a
feasibility guarantee**. The hint is neither a lower bound on every possible
mission nor a validated estimate of its completion time.

`report.json` additionally contains `arrival_deadline_verified_in_execution`
and `post_arrival_time_verified_in_execution`. A failed check produces exit
code 3, just as other executed mission violations do.

## Four-drone regression scene

The saved 90-second case with target `(500,500,30)`, 180-degree approaches and
the enlarged static obstacles still correctly returns `no_candidate`. Detailed
counts identify 195 proposals not reaching the target, 55 leaving insufficient
post-arrival time and 110 whose recovery exceeds the horizon. The suggested
140-second retry is a heuristic, not a tested guarantee.

With horizon 180 s, deadline 130 s and post-arrival minimum 20 s, the same
four-aircraft scene passed all nominal execution checks. Observed arrivals were
approximately 90.04, 113.04, 99.92 and 80.38 s. Minimum sampled inter-drone
distance was 51.17 m; minimum obstacle-surface distance was 24.36 m.
These are model-based nominal measurements, not disturbed-flight guarantees.
