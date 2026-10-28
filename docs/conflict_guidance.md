# Witness-guided proposal generation

The CLI enables `--conflict-guidance witness`. Use `--conflict-guidance none`
to compare with the previous unguided catalog expansion. The Python API keeps
`Config(conflict_guidance="none")` for compatibility. Both modes retain the same
flight model, validation, departure bounds, approach sectors and safety margins.

This is an **offline, bounded proposal heuristic**, not Conflict-Based Search
(CBS), an online avoidance controller, or a complete continuous-space planner.
It extends [adaptive catalogs](adaptive_planning.md) beyond choosing which
aircraft needs more proposals: it also biases how new proposals are generated.

## From a pair check to a proposal

For integrated plans `i,j`, the joint search checks

```text
L_ij = min_n ||p_i(n dt) - p_j(n dt)|| - (V_bound_i + V_bound_j) dt/2
R_ij = d_min + 2 position_error + (V_bound_i + V_bound_j) clock_error
compatible iff L_ij >= R_ij + 1e-7
```

If this fails, the planner retains the worst tested candidate pair per aircraft
pair, ranked by slack `L_ij - R_ij`. Its witness contains candidate identifiers,
the nearest **sample's** time and positions, and both bounds. This sample is
not an exact collision time, nor does a negative robust slack prove a collision.
Only one witness per aircraft pair is retained; it may be a historical pair,
not a member of every currently failing combination.

When a catalog is expanded, the worst available witness involving that drone
provides a hint. Guided attempts cycle through available modes:

1. **Cruise altitude**, only when an automatic 3D cruise range is configured.
   Select opposite allowed endpoints according to the pair's altitude order
   at the witness. Equal altitudes break the tie by drone ID.
   The descent proposal's minimum cruise height still applies. Obstacle-derived
   cruise proposals can take precedence; the hint is not a hard reservation.
2. **Lateral detour**, only if the configured detour scale is positive. Project
   the aircraft's witness point onto the current start-to-entry segment:
   `f = clip(dot(p_w-start, delta)/dot(delta,delta), .15, .85)`.
   Move the nearer of the two optional waypoints to `start + f delta + s A normal`,
   where `A` is the existing candidate's detour scale, and `s` points away from
   the other witness position along that normal. Coincident projections use ID
   order. Mandatory entry gates, approach direction and destination are retained.
3. **Departure**, only if the allowed extra-loiter interval has nonzero width.
   Try its minimum for the lower-ID endpoint and its maximum for the other.
   A zero-width interval is never widened, including simultaneous departures.

These are freshly seeded proposals, not mutations of an already validated
plan. If a guided attempt fails, the next attempt is ordinary exploration.
Every proposal consumes the same cumulative per-drone attempt allowance. The
guided-mode counter persists between expansions. No new unbounded retry loop
is introduced. Changing expansion sizes can change the resulting sequence;
reproducibility requires identical planning settings and numerical environment.

For margin-preferring selection, hints instead come from the current feasible
incumbent's lowest-slack pairs. The incumbent is retained while alternatives
are generated and tested; a rejected repair cannot replace it.

## Validation and scope

```text
sampled pair witness -> bounded proposal bias -> fresh geometric path
 -> coupled physical rollout -> independent individual / obstacle validation
 -> joint separation search -> independent final pair / obstacle recheck
 -> individual immutable plans -> own-state-only execution
```

Changing geometry or altitude can change timing and introduce new conflicts.
For that reason no repair is assumed to resolve its source conflict, and joint
search rechecks all relevant pairs over the entire common horizon, including
loiter, target passage, recovery and onward flight. No instantaneous altitude
change, post-validation retiming, relaxed tolerance or hidden extra delay is
used. Spatial and timing constraints remain conditional on the model and
bounded tracking/clock errors; see [flight model limitations](flight_model_3d.md)
and [disturbed execution](robust_tracking.md).

The full witness graph is ground-side planning data only. Individual exported
plans contain neither these hints nor another drone's trajectory. Execution
requires no new communication.

`planning_report.json` records `conflict_guidance` and, per catalog,
`guided_proposals`: attempted candidate ID, mode, source witness and whether
**individual** validation accepted it. Acceptance there does not mean the
candidate was selected or jointly compatible. Unsuccessful attempts also
contribute to the existing rejection-stage counts.

## Reproducible four-aircraft comparison

```bash
# Guided proposals, simultaneous departures, 180-degree approach spread
.venv/bin/python examples/trajectory_3d.py --num-drones 4 \
  --target 500 300 --horizon 150 --cruise-range 40 110 \
  --approach-spread 180 --activation-time 0 --departure-delay 0 0 \
  --initial-candidates 1 --conflict-guidance witness

# Paired comparison on default, simultaneous-departure and obstacle scenes
.venv/bin/python scripts/benchmark_conflicts.py --seeds 7 19

# Repeated planning-only measurements (pool startup included)
.venv/bin/python scripts/benchmark_conflicts.py --scenarios simultaneous \
  --no-simulate --repeats 3
```

The benchmark always uses four aircraft and compares identical snapshots,
seeds, budgets and constraints. It records complete configurations, signatures,
rejections, timings and optional nominal execution checks in a new JSON file.
Neither mode must select the same trajectory as the other. Both must pass all
requested checks; exit code 2 preserves failures for analysis. A timing gain is
empirical and scene-dependent, not guaranteed. Modes alternate order across
repetitions to reduce ordering effects. No obstacles in the user-supplied map
are reduced or removed for this comparison.

### Observed validation results (2026-10-25)

For simultaneous departures, target `(500,300,30)`, 180° approach spread,
150 s horizon, 40–110 m cruise range and one initial plan per aircraft:

| Seed | Proposals, unguided / guided | Nominal separation lower bound, unguided / guided | Nominal execution |
|---|---:|---:|---|
| 7 | 10 / 9 | 25.44 / 35.93 m | Both passed every check |
| 19 | 10 / 10 | 21.93 / 26.65 m | Both passed every check |

The required nominal pair distance is 16.8 m; the values above are conservative
whole-horizon bounds, not sampled distances or additional slack. Only the
conflicting catalogs were expanded. Seed 7's guided altitude repair for drone 2
was individually accepted; drone 1's repair was rejected and ordinary
exploration supplied its alternative. Rejection was not bypassed.
Raw results: `results/benchmarks/conflicts-first-20261025.json`.

The default and large-obstacle scenes used both modes and seeds 7/19. Default
scenes and the seed-7 obstacle scene passed all nominal execution checks.
The seed-19 obstacle scene failed **in both modes**: drone 3 passed at
31.01452°, just outside the unchanged 30° ± 1° tolerance. Tracking, physical
limits, obstacles and pair separation still passed. That scene's planned
trajectories were individually valid, illustrating that planning acceptance and
execution acceptance are distinct gates. The benchmark correctly returned 2;
these are retained failures, not a successful campaign.
Raw results: `results/benchmarks/conflicts-scenes-20261025.json`.
That result predates reintegrated contact validation. The old seed-19 candidate
is now rejected before selection; both modes select alternatives that pass the
nominal execution checks. See [contact regression](mission_pipeline.md).

An additional actual CLI run used four drones, both large obstacles, target
`(500,500,30)`, 180° approach spread, 180 s horizon, default 30–120 m automatic
cruise, guided proposals and margin-preferring selection. All nominal execution
checks passed. Minimum sampled separation was 51.17 m, minimum sampled distance
to obstacle surfaces was 37.61 m, and the selected combination's minimum robust
slack improved from 9.41 to 31.18 m relative to its first feasible combination.
This is not an isolated comparison of guidance alone. Artifacts:
`results/trajectory3d/20261025-214805-485239/`.

The above execution campaigns overlapped other local test work. Their wall
times are retained for traceability, not presented as controlled speedups.

A separate planning-only comparison ran after the test processes finished,
on an Intel i3-1115G4 (2 physical cores / 4 logical processors), Python 3.12,
four requested workers, three repetitions per mode and alternating order:

| Simultaneous-departure seed | Median unguided | Median guided |
|---|---:|---:|
| 7 | 2.59 s | 2.51 s |
| 19 | 2.82 s | 2.97 s |

Pool startup is included; simulated execution is excluded. These small samples
and normal desktop load do **not** demonstrate a consistent speedup. The observed
benefit here is greater separation margin and occasionally fewer proposals,
not a universal time reduction. All 12 planning runs validated; nominal flight
checks for these same seeded scenarios are recorded in the earlier execution
campaign. Raw timings: `results/benchmarks/conflicts-timing-final-20261025.json`.

## Remaining limitations

Fresh proposals need not resemble the conflicting parent; there is no local
continuous optimization, reservation table, speed-profile optimization or
systematic branching over every conflicting interval. An altitude hint can be
irrelevant during loiter or descent, and a detour can create another conflict.
The heuristic can therefore be slower than unguided exploration, or fail where
it succeeds. `catalog_exhausted` and `search_budget_exhausted` still do not prove
continuous-space infeasibility. This feature does not solve unmodeled failures
or guarantee tracking under arbitrary wind or navigation errors.
