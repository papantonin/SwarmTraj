# Contact events, moving preparation and cell/time reservations

These three additions keep full-horizon physical, obstacle and pair-separation
checks. They introduce no inter-aircraft or ground telemetry during autonomous
execution. Existing planar/legacy modes remain available.

## 1. Consistent first-contact events

Coupled 3D planning and execution now use the same numerical first-entry
algorithm (`contact.py`). For destination center `D` and radius `r`, it seeks
the first event `||p(t)-D|| <= r` after activation. The search:

1. Prunes grid intervals using the supplied speed bound.
2. Subdivides possible intervals, including cases where both endpoints are
   outside but the path crosses the sphere between them.
3. Reintegrates held physical controls at interior times instead of linearly
   interpolating position or attitude.
4. Localizes entry to a time bracket no wider than `1e-6 s`.

The validator checks the existing **1-degree** flight-path-angle tolerance at
this contact and at the nominal closest-approach time. It additionally checks
the approach sector at contact and rejects contact before departure. The
observer reconstructs actual held controls, including asynchronous switches
and simulator wind; this ground-side reconstruction never feeds a controller.

The work limit is 4096 dense evaluations per event. Unresolved searches are
rejected (`validation_contact_event_unresolved` in planning; failed arrival and
`contact_resolution_failures` in execution). An initially inside state has a
zero-width event at the allowed start time. Tangencies and excursions shorter
than the time resolution remain numerical limitations, not certified roots.
RK4 still has no rigorous integration-error enclosure.

`report.json` distinguishes:

- `nominal_contact_times_s` and `nominal_contact_descent_angles_deg`;
- `arrival_times_s` and `actual_descent_angles_deg`, now localized at actual entry;
- historical `nominal_arrival_times_s` / `nominal_descent_angles_deg`, which still
  describe the stored closest-approach reference, not sphere entry;
- `contact_bracket_width_s`, `contact_detection`, `contact_resolution_failures`
  and the required `contact_events_verified` acceptance gate.

The previous obstacle/seed-19 candidate for drone 3 has an entry angle of
approximately **31.01778 degrees**. It is now rejected before selection rather
than passing the coarse nominal grid. Both unguided and guided four-aircraft
comparisons select alternatives and pass all nominal execution checks:
`results/benchmarks/contact-regression-20261026.json` (the filename is a label;
run timestamps elsewhere retain the actual local clock).

## 2. Preparation while aircraft remain in loiter

Enable `--prepare-in-flight`. Mission time zero is target designation, not the
end of computation. `--activation-time A` defines the agreed future activation
and radio-cutoff epoch on this same clock. No deadline or horizon is shifted.

The candidate generator predicts each aircraft's position at release from the
designation snapshot and its known loiter. Integrated plans include that whole
holding prefix; simulation starts at designation and never resets physical
positions after calculation or loading. Heading and position at activation
therefore differ from their designation values. Physical, obstacle and pair
checks include the entire preparation interval. The common activation time is
distinct from the configured per-drone extra-loiter interval.

Preparation models one shared upload link, in sorted ID order:

```text
upload_finished_k = planning_duration + (rank_k + 1) upload_delay
acknowledged_k = upload_finished_k + acknowledgement_delay
latest_ready = activation_time - clock_error - activation_guard
activate iff every acknowledgement exists and acknowledged_k <= latest_ready
```

The guard accounts for the earliest bounded clock-shifted departure. A late or
missing acknowledgement yields `preparation_not_ready`, exit code 2, no
executable plans and no individual payloads. The program never extends the
activation time, departure bounds or arrival deadline automatically. A planning
failure retains its original status even if preparation would also fail.

| Option | Default | Meaning |
|---|---|---|
| `--prepare-in-flight` | Off | Enable the simulated readiness gate |
| `--activation-time` | 3 s | Future activation relative to designation; normally increase explicitly for preparation |
| `--upload-delay` | 0.25 s | Per-plan sequential upload duration |
| `--ack-delay` | 0.1 s | Delay from upload completion to acknowledgement |
| `--activation-guard` | 0.5 s | Additional readiness guard |
| `--planning-latency` | Unset | Use measured planner wall time; an explicit value simulates a reproducible duration instead |
| `--missing-ack ID` | None | Simulate a lost acknowledgement, repeatable |

`Preparation` and `prepare_mission` are also exported by the Python API.
`planning_report.json` stores the complete readiness timeline under
`timings.preparation_timeline`. Predicted activation states, loading times and
readiness are included. With an explicit simulated planning duration, the
deterministic timeline also appears in `diagnostics.preparation`. Actual wall
durations remain outside the mission hash. With measured latency, activation
acceptance can legitimately depend on machine load; the physical plans remain
deterministic, but the readiness decision is not promised identical.

The 3D replay shades preparation in yellow and displays **In-flight preparation**
until activation. It replays a chronological simulation
after computation; it is **not a wall-clock live flight backend running in
parallel with the planner**. Upload/acknowledgement durations are modeled inputs,
not real networking or payload-size measurements. The all-or-none decision is
a simulator coordinator assumption, not a solution to distributed agreement
with unbounded packet loss. Rejecting activation is not a certified indefinite
holding or emergency policy beyond the checked horizon.

Faster [native physical construction](native_kernel.md) reduces measured
planning latency and can therefore change this fixed-activation readiness
decision. It does not move activation earlier, refresh the snapshot or shorten
the prediction horizon to an unchanged activation time. An explicitly supplied
`--planning-latency` also stays unchanged when switching numerical backends.
Earlier activation requires selecting a new activation time before planning and
revalidating the entire mission.

For example, with three aircraft and the default transfer/guard settings, the
readiness inequality is `activation_time >= planning_duration + 1.5 s`.
The repeated headless reference measurements (7.626 s Python / 0.199 s C++)
give arithmetic readiness thresholds of 9.126 s / 1.699 s. For the measured
plans' fixed 3 s activation, the C++ duration passes the simulated readiness
gate and the Python duration does not. These are modeled transfers using
observed planning durations, not guaranteed live network or real-time results;
the thresholds do not certify a different, uncomputed activation schedule.

## 3. Explicit full-horizon space/time reservations

Enable `--reservation-policy grid` (default `none` for compatibility).
Each independently validated physical candidate reserves cells on a regular
world-aligned XYZ grid (XY in planar mode), including loiter, altitude changes,
approach, recovery and onward flight. For sample `p_i(n dt)`, define

```text
inflation_i = separation/2 + position_error
              + V_bound_i clock_error + V_bound_i dt/2
time interval_n = [max(0,(n-1/2)dt), min(horizon,(n+1/2)dt)]
```

All cells intersecting the axis-aligned box `p_i +/- inflation_i` are reserved
over that interval. The box overapproximates the corresponding ball. Contiguous
intervals in each cell are merged. The `dt/2` term covers between-sample motion;
the other terms retain the center-to-center safety distance, tracking and clock
margins. Negative coordinates and exact time boundaries are supported.

Joint search allocates one complete candidate per drone only if reservations
of every selected pair have **no overlapping closed interval in a common cell**.
The same geometric cell can be reused at sufficiently different times. Cached
cell/time conflicts filter candidate combinations and drive adaptive expansion.
This allocates passage windows by choosing physically executable trajectories,
not by asking a fixed-wing aircraft to stop until a cell becomes available.
The module also provides safe-interval complements with occupied endpoints
excluded for future interval-directed planning.

Grid compatibility is an **additional hard constraint**. The existing robust
distance check is retained and independently repeated on selected trajectories.
All selected reservations are independently rebuilt and cross-checked before
acceptance. No geometry, speed or timing is changed after validation.

- `--reservation-cell-size 20`: cell edge in meters, minimum 5 m.
- `--reservation-max-entries 1000000`: per-candidate construction budget.
  Exhaustion returns `reservation_budget_exhausted`, not an impossibility proof.
- `diagnostics.search` counts reservation conflicts and records the first
  cell/time witness. `diagnostics.reservations` summarizes successful allocation.
- Each individual payload includes **only its own** `own_reservations` table,
  not the fleet calendar or another aircraft's plan.
- The observer checks sampled actual center adherence to the own reservations,
  reports `reservation_adherence_verified`, and rejects execution when it fails.
  Full physical-distance, obstacle, tracking and clock checks remain mandatory.

Optional [free-window guidance and richer profiles](free_windows_profiles.md)
now generate release proposals from forecast free intervals before rollout;
the reservations and final checks in this section remain mandatory.

This is resource allocation over a finite flyable catalog, **not a complete
SIPP implementation or a kinodynamic completeness guarantee**. Coarse grids
and inflated boxes can reject separated trajectories unnecessarily. No local
continuous repair, full safe-interval graph search or
automatic cell-size relaxation is implemented. A failure may mean insufficient
catalog coverage, conservative cells or exhausted budgets rather than absence
of a safe continuous-space solution.

## Combined four-aircraft example

```bash
.venv/bin/python examples/trajectory_3d.py --num-drones 4 \
  --target 500 500 --horizon 210 --approach-spread 180 \
  --obstacles examples/scenarios/static_obstacles.json \
  --prepare-in-flight --activation-time 30 --planning-latency 5 \
  --reservation-policy grid --initial-candidates 2

# Replace the simulated planning latency with a value too late for activation:
# --planning-latency 30
# Or simulate a missing acknowledgement:
# --missing-ack 2
```

The example uses an explicitly simulated 5 s computation duration. Omit
`--planning-latency 5` to test readiness against measured planning wall time.
Neither version implies a guaranteed real-machine computation deadline.

The first combined headless run passed every nominal execution check, with
**47.34 m** minimum sampled inter-drone distance and **43.16 m** minimum sampled
obstacle-surface distance. All plans were acknowledged by 6.1 s, before the
29.35 s guarded deadline. Allocated per-drone cell counts were 550/526/557/490.
Artifacts: `results/trajectory3d/20261025-221013-698944/`.
Two actual CLI fault runs using the same four-aircraft scene returned exit 2
and `preparation_not_ready`, without individual payloads or simulated execution:
30 s modeled planning latency (`results/trajectory3d/20261025-222004-402480/`)
and a missing acknowledgement from drone 2
(`results/trajectory3d/20261025-222006-269123/`).
Actual Tk replay with preparation shading and 180-degree approaches was also
checked: `results/preparation-gui-smoke/20261025-221936-200354/`.
Tests additionally compare serial and spawn planning, check movement through
preparation, recheck allocated intervals and compare event detection under
different observer resolutions. Finite successful trials are not proof of
robustness under arbitrary wind, navigation faults or missed activation.
