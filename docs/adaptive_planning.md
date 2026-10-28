# Adaptive planning and diagnostics

The trajectory CLI defaults to **adaptive catalogs** and **minimum remaining
values (MRV)** search order. The library keeps `Config(planning_strategy="fixed",
search_order="id")` for compatibility. Flight dynamics, arrival requirements,
separation margins and independent validation are unchanged.
The CLI additionally defaults to margin-preferring selection after the first
success; the library retains first-feasible selection. See
[margin selection](margin_selection.md). Benchmarks on this page use the
historical first-feasible policy unless stated otherwise.

## Workflow

1. Check the frozen initial snapshot and assign geometric approach sectors.
2. For each drone, generate up to `initial_candidates` independently validated
   plans, stopping earlier if its cumulative attempt budget is exhausted.
3. Search for a compatible combination using forward checking. With MRV, choose
   the drone with the smallest remaining domain, breaking ties by ID. Candidate
   order within each domain stays unchanged.
4. If no combination is found and budgets remain, rank expandable catalogs by
   domain wipeouts, then observed pair conflicts, then size and ID. Enrich at
   most two catalogs per round, adding up to `initial_candidates` plans each.
5. Repeat search with accumulated plans and a persistent pair cache.
6. When margin selection is enabled, refine the combination and optionally
   extend catalogs belonging to low-margin pairs under the same cumulative caps.
7. Independently recheck selected pairs over the entire common horizon.

The CLI also uses sampled conflict witnesses to bias new altitude, detour or
departure proposals (`--conflict-guidance witness`). It does not modify already
validated plans or reserve target-passage windows. See
[witness-guided proposals](conflict_guidance.md) for the method and its limits.
`--conflict-guidance none` restores catalog-level targeting only; the library
keeps that historical default. MRV does not rank candidates by
least-constraining value or guarantee fewer nodes on every problem.
Optional [mixed profile families](mixed_profiles.md) start simple and unlock
richer proposals during conflict/margin expansion or consecutive rejections.
They require explicit opt-in and never weaken strict profile requests.
Optional [own-rejection feedback](parameter_campaign.md) also operates inside
initial catalogs: `--candidate-guidance rejections` explores longer settling
holds or wider visibility routes after repeated matching rejections. All
existing independent physical and joint checks are retained.

## Settings

```bash
# Default adaptive planning, four drones, with visualization
.venv/bin/python examples/trajectory_3d.py --num-drones 4

# Explicit adaptive settings, headless execution
.venv/bin/python examples/trajectory_3d.py --num-drones 4 --no-gui \
  --planning-strategy adaptive --initial-candidates 4 --search-order mrv \
  --approach-spread 180 --horizon 150

# Historical full-catalog behavior and identifier-order search
.venv/bin/python examples/trajectory_3d.py --num-drones 4 --no-gui \
  --planning-strategy fixed --search-order id --selection-policy first --approach-spread 180 --horizon 150
```

- `--candidates 28`: maximum accepted plans per drone across all rounds.
- `--attempts 90`: maximum proposals per drone across all rounds.
- `--initial-candidates 4`: initial accepted-plan goal and expansion increment,
  capped at the maximum candidate count. A difficult drone may require all
  attempts even to meet the initial goal.
- `--max-search-nodes 30000`: cumulative node budget across rounds, including
  revisited branches; never reset after catalog expansion.
- `--workers 0`: automatic process count. Each round uses a fresh spawn pool,
  so very small batches may incur noticeable overhead.
- `--conflict-guidance witness`: bias expanded catalogs from sampled pair
  witnesses; `none` disables proposal-level guidance without changing validation.

Each drone's PCG64 stream resumes from its exact state, including draws made by
rejected proposals. Attempt IDs stay unique; earlier plans remain unchanged.
Catalogs with exhausted attempt or candidate budgets are not regenerated.
Worker errors propagate. With the same inputs, strategy and numerical
environment, serial and parallel execution produce identical plans and
deterministic diagnostics. Switching strategy or search order may select
different plans; the manifest records these settings.

An empty catalog has used its entire attempt allowance and produces
`no_candidate`. Nonempty incompatible catalogs produce `catalog_exhausted`
when no catalog can expand. Exhausting the cumulative node budget produces
`search_budget_exhausted`. None proves continuous-space infeasibility.
Adaptive search may use its node budget before reaching fixed-mode catalog
sizes, or succeed without generating most of those catalogs. Neither mode
universally dominates.

## Diagnostic outputs

Every CLI planning run exports **`planning_report.json`**, including failures
after valid CLI input. It contains status, reason, deterministic diagnostics
and non-deterministic timings. Preflight rejection may have empty diagnostics
but still includes total planning time.

`mission.json` includes `planning_diagnostics` in its reproducible manifest;
**timings are excluded from the manifest and its hash**. Successful execution
also includes diagnostics and timings in `report.json`. Individual drone
payloads do not receive the global conflict graph or other drones' plans.

Diagnostics include:

- Expanded drone IDs, catalog sizes, cumulative attempts and search nodes per
  round, and whether a solution was found.
- Accepted/rejected counts per drone and attempt/candidate budget flags.
- The first failed generation or validation stage for each rejected proposal:
  geometric arrival/horizon, vertical profile, rollout envelope, refined
  envelope validation, target/sector, descent angle or cruise recovery.
  These identify stages, not every underlying physical cause. Legacy validation
  failures use the broader `individual_validation` category.
- Finer `rejection_details` distinguish pre-arrival transitions, arrival deadlines
  and post-passage recovery. `diagnostics.timing` gives schedules and advisory
  horizon suggestions; see [mission timing](mission_timing.md). Suggestions are
  never applied automatically and do not guarantee a feasible solution.
- Accepted obstacle-routing proposal families per catalog; the CLI explores
  lateral and altitude-level alternatives, described in [obstacles](obstacles.md).
- Per-catalog guided proposals, including mode, source candidate-pair witness
  and individual acceptance. These are not joint-safety or optimality claims.
- Unique pair checks, cache hits and domain wipeouts.
- For each explored drone pair: tested candidate pairs, conflicts, worst
  observed nominal distance lower bound, the corresponding required margin
  and closest sample time for that candidate pair. This is not a formally
  localized continuous-time collision.

A conflict means failure of the conservative robust criterion, not necessarily
an actual physical collision. Counts cover only explored candidates; they are
not collision probabilities, and an unexplored pair must not be assumed safe.

Timings distinguish catalog wall time (including process startup/transfers),
search, final joint verification and total planning. Per-drone generation and
independent-validation durations are accumulated inside workers. Parallel
durations cannot be summed to infer total wall time. Total time also includes
preflight and bookkeeping not covered by the other phase measurements.

## Four-drone evaluation

```bash
# Three scenarios, all using four aircraft; execution checks enabled
.venv/bin/python scripts/benchmark_adaptive.py

# Multiple seeds and bounded execution perturbations
.venv/bin/python scripts/benchmark_adaptive.py --seeds 7 19 \
  --wind 0.05 --clock-offset 0.05

# Planning-only measurements with repetitions
.venv/bin/python scripts/benchmark_adaptive.py --no-simulate --repeats 3
```

The benchmark compares `fixed` (ID order), `adaptive_id` and `adaptive_mrv` on
the default 90 s, 180°/150 s and 360°/240 s scenarios. Each uses four aircraft,
12 m/s airspeed, initial altitudes 30/55/80/105 m, automatic cruise within
30–120 m, and a 30° descending airborne pass followed by recovery. Each mode
uses identical disturbance settings. Execution time is excluded from planning
measurements.

`--reference path/to/benchmark.json` optionally checks seed-7 fixed-mode result
signatures against an earlier benchmark. Adaptive plans may differ and are
validated rather than required to match historical selection. Results go to a
new JSON file under `results/benchmarks/`; existing files are not overwritten.
Exit code 2 indicates a failed planning, execution or reference check, with
results retained for analysis. `execution_failures` names the failed execution
checks; separation success alone is not treated as complete validation.

Single runs are illustrative, not a distribution or deadline guarantee. Repeated
seeds and disturbed simulations provide empirical evidence, not a proof of
tracking under every disturbance. Safety scope remains that of the
[flight model](flight_model_3d.md).

## Initial measurements and limitations

On the Intel i3-1115G4 machine (2 physical cores / 4 logical processors), a
planning-only comparison with four workers and seed 7 measured:

| Scenario | Fixed catalogs, ID order | Adaptive catalogs, MRV | Speedup |
|---|---:|---:|---:|
| No sector, 90 s horizon | 13.47 s | 9.30 s | ×1.45 |
| 180°, 150 s horizon | 27.11 s | 5.25 s | ×5.16 |
| 360°, 240 s horizon | 41.02 s | 9.87 s | ×4.16 |

These are single measurements, not controlled-laboratory results: desktop load
and CPU conditions varied. A separate run with flight simulation enabled found
adaptive planning times of 4.51–6.03 s, illustrating this variability. In all
three scenarios the reduced catalogs solved in the first round and four search
nodes. The main gain is generating fewer candidates, not MRV. Full-catalog
result signatures matched the historical reference exactly. Raw data:

- `results/benchmarks/before-adaptive-4-drones.json`: pre-change reference run.
- `results/benchmarks/adaptive-4-drones-timing.json`: planning-only comparison.
- `results/benchmarks/adaptive-4-drones-comparison.json`: three modes, all nine
  nominal executions fully validated; minimum sampled pairwise distances were
  37.11 m, 28.19 m and 43.32 m for the three scenarios.

A separate campaign used seeds 7 and 19, 0.05 m/s synthetic wind and clock
offsets distributed from −0.05 to +0.05 s. All six executions reached the target
and passed tracking-tube and separation checks. However, only the two 360°
executions passed every check: some other runs violated physical/vertical limits
or the 1° descent-angle tolerance. These are recorded failures, not relaxed
thresholds or robust-flight guarantees. The campaign returned exit code 2.
Raw data: `results/benchmarks/adaptive-4-drones-perturbations.json`.
That result describes the historical executor. The new constrained executor
passes all six original small-disturbance trials on identical nominal plans,
without changing thresholds. Broader wind/sensor trials still expose limits;
see [disturbed tracking and validation](robust_tracking.md).

An additional four-aircraft end-to-end run requested simultaneous departures,
target `(500,300,30)`, 180° approach spread, cruise range 40–110 m and an initial
catalog of one plan per drone. The first search identified a conflict between
drones 1 and 2. Only their catalogs were extended; the second search succeeded
with sizes `(1,2,2,1)` and six cumulative search nodes. All nominal execution
checks passed, with 28.64 m minimum sampled separation. This exercises actual
conflict-driven expansion, rather than only first-round success. The observed
3.78 s planning time was measured while the test suite was running and is not
used in the timing comparison above. Artifacts:
`results/trajectory3d/20261025-162801-830040/`.
