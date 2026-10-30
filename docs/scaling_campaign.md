# Centralized planning campaign: one to five aircraft

`scripts/benchmark_scaling_campaign.py` measures the existing coupled 3D
point-mass planner without modifying its physical model or acceptance checks.
It does not launch PX4 or Gazebo. See [flight model](flight_model_3d.md),
[adaptive planning](adaptive_planning.md), [obstacles](obstacles.md) and
[richer profiles](free_windows_profiles.md) for the mechanisms under test.

## Running a campaign

Use the existing project environment. Each output directory must be new.

```bash
.venv/bin/python scripts/benchmark_scaling_campaign.py \
  --output results/scaling-campaign --timeout 120

# Resume interrupted work using the saved matrix and identical source hashes.
.venv/bin/python scripts/benchmark_scaling_campaign.py \
  --output results/scaling-campaign --resume

# Audit artifacts and produce PDF, Markdown, CSV and graphical reports.
.venv/bin/python scripts/report_scaling_campaign.py results/scaling-campaign

# Run these probes after the first campaign to avoid CPU contention.
.venv/bin/python scripts/benchmark_planning_limits.py \
  --reference results/scaling-campaign --output results/scaling-limits
.venv/bin/python scripts/report_scaling_campaign.py results/scaling-campaign \
  --follow-up results/scaling-limits
```

The default matrix contains **319 planning jobs**:

| Suite | Jobs | Purpose |
|---|---:|---|
| Core | 180 | 12 families × 5 fleet sizes × 3 planner seeds |
| Random | 50 | 10 deterministic scene IDs × 5 fleet sizes |
| Ablation | 60 | 6 policy variants × 5 fleet sizes × 2 planner seeds |
| Replay | 10 | Serial and two-worker repeats at each fleet size |
| Controls | 19 | Explicitly unreachable, invalid, obstructed or collocated inputs |

Core families cover the baseline, 180°/360° approach spreads, static obstacles,
short horizon, explicit arrival deadline, simultaneous release, larger
separation, denser same-altitude loiters, different individual airspeeds,
strict smooth-speed/multilevel profiles and grid reservations with free-window
guidance. Changes between families are saved explicitly; cross-family results
are not controlled single-parameter comparisons. For example, approach-spread
families also have longer horizons.

Random scenes draw target, target altitude, heading rotation, spatial offset,
loiter radius/spacing, individual speed/altitude, horizon, departure-delay
window, descent angle, approach spread and separation. Global mission draws
remain comparable across fleet sizes for each scene ID. These scenes do not
include randomly generated obstacles. Configured physical limits are unchanged.

The baseline uses an adaptive initial batch of two candidates, MRV search,
margin-preferring selection, conflict witnesses and spatial obstacle routing.
This is an explicit benchmark configuration, not every library/CLI default.
It retains 28 candidate/90 attempt caps per aircraft, 30000 search nodes,
2000 margin-search nodes, dt=0.2 s, 2 m position error and 0.15 s clock error.

Ablations share the spread180 family inputs and cumulative candidate/attempt
caps. They compare first-feasible versus margin selection, fixed catalogs,
conflict guidance, search order and own-rejection feedback. Fixed and adaptive
strategies intentionally do different amounts of work within those caps.

For accepted baseline, obstacle and heterogeneous-speed jobs at the first core
seed, the worker additionally executes nominal, gust/sensor and outside-clock
profiles. The outside-clock trial is expected to be rejected. All other
profile failures remain visible as unmet expectations, even when planning
itself succeeds. There are at most 45 execution trials; missing executions
after rejected plans/timeouts are not counted as passed trials.

The optional follow-up adds 99 jobs: 40 candidate/attempt budget comparisons,
25 search-node cap probes, 24 explicit horizon variations and 10 mixed-profile
trials. Candidate and attempt caps vary together; this does not isolate their
individual contributions. The search-cap trials use first-feasible selection
and identical spread180 inputs. Four one-node multi-aircraft probes explicitly
expect `search_budget_exhausted`. Horizons and permission to use mixed profile
families are explicit mission changes, not silent relaxation after failure.

For an execution requiring investigation, an observer-only replay can trace
the original envelope predicate and the actual asynchronous boundary bindings:

```bash
.venv/bin/python scripts/diagnose_campaign_execution.py \
  results/scaling-campaign core-baseline-n5-s7 --profile gust_sensor \
  --output results/execution-diagnostic.json
.venv/bin/python scripts/report_scaling_campaign.py results/scaling-campaign \
  --follow-up results/scaling-limits --diagnostic results/execution-diagnostic.json
```

The diagnostic requires the exact original stored plan arrays and preserves
all predicate results. It never upgrades the original rejected execution or
alters a controller, physical limit or validation threshold.

## Measurements and failure semantics

Jobs run one at a time. Planning uses one process except explicit two-worker
replays. CPU-heavy catalog parallelism uses the existing spawn mechanism.
BLAS/OpenMP thread counts are fixed to one in each child process. A desktop
host is not isolated against other workloads, CPU-frequency changes or heat.

`planning_wall_s` measures only `plan_mission`; the planner also records its
internal generation, validation, joint-search and optimization times.
`process_wall_s` includes imports, export, compression and optional execution.
Per-process peak RSS is recorded separately from the largest child's peak;
neither is a simultaneous aggregate memory measurement. CPU measurements
include worker imports and completed child processes but precede export and
optional execution.

The timeout applies to the whole worker process. Completed planning is written
to a checkpoint **before execution**. A later timeout therefore preserves its
planning result and completed execution profiles, but cannot count unfinished
profiles as passed. A planning timeout has no planner status: it is an
externally censored observation, not an infeasibility result. Completed-time
percentiles exclude these censored runs and must be read with timeout counts.

`no_candidate`, `catalog_exhausted` and `search_budget_exhausted` retain their
original meanings. None proves infeasibility in continuous space. Expected
negative controls are reported separately from exploratory success rates.
Exceptions stop the campaign after the failing worker's evidence is retained;
partial worker output is never accepted as a successful mission.

## Comparing physical and geometric generators

Replay saved inputs rather than regenerating scenes when comparing the opt-in
[`physical_mission` generator](phase2_kinodynamic.md) with the historical results:

```bash
.venv/bin/python scripts/benchmark_scaling_campaign.py \
  --reference results/scaling-campaign-20261028/manifest.json \
  --suites controls core random replay --generator-mode physical_mission \
  --output results/physical-generator-comparison

.venv/bin/python scripts/report_generator_replay.py \
  results/physical-generator-comparison
```

This selects 259 jobs: 230 exploratory missions, 19 negative controls and ten
serial/spawn repeats. Only the generator mode changes in the saved mission
inputs; snapshots, seeds, limits, margins, candidate/search budgets and the
original per-process timeout remain fixed. Physical replays also request
execution of accepted strict-rich and grid jobs at the first core seed, in
addition to baseline, obstacles and heterogeneous speed. This means at most
75 execution trials, inside the same worker timeout. The manifest records this
extended execution coverage; planning and execution outcomes remain separate.

The comparison reporter audits original artifact hashes and inputs, then
independently revalidates saved individual plans, full-horizon pair distances,
reservations and physical pull-out connections. Negative checks corrupt final
commands and place obstacles at post-arrival endpoints. These numerical checks
are separate from journal consistency and do not certify integration error.
The report lists retained, gained and lost planning successes, worker completion
status, and every missing, duplicated or unexpected execution job/profile.

`analysis/index.html`, `generator-comparison.json` and `completion.json`
supplement the standard PDF/CSV reports. A successfully generated report does
not mean that the new generator is suitable as the default. Its promotion
decision also requires complete coverage, no lost historical successes, all
negative controls and exact spawn comparisons, complete execution profiles
with matching expectations, and no unresolved timeouts or revalidation errors.
The reporter never changes defaults. Fresh coupled 3D defaults are documented
in [default selection](planner_defaults.md); historical campaign cases keep
their explicit geometric/Python baseline.

See the [completed saved-input comparison](phase2_kinodynamic.md#completed-saved-input-comparison)
for the historical measured outcomes, hardware, execution coverage and censored
jobs. The subsequent [migration checks](generator_migration.md) complete those
five inputs below the same timeout and compare one/two-process variants on an
explicit subset; they do not replace the full historical journal.

## Reproduction and artifacts

The immutable `manifest.json` contains all requested jobs, complete effective
configurations and snapshots, hardware/runtime metadata, timeout and source
hashes. Source changes abort execution/resume. Worker streams are kept under
`logs/`; gzip-compressed original mission documents, including full selected
states and controls, are under `missions/`. Each finished/censored job is
flushed to `journal.jsonl`. Interrupted attempts are preserved before retry.
There is no automatic retry with relaxed constraints or larger budgets.

The reporter checks coverage, unique IDs, canonical mission hashes, result
signatures, input agreement and selected-plan counts. This is a **consistency
audit**, not independent physical trajectory revalidation. Exact replay
signatures compare status, selected arrays, counts, search work and distance
values while excluding timings and requested worker count.

`analysis/` contains `report.pdf`, `report.md`, plots, `missions.csv`, optional
`executions.csv`, `summary.json` and `audit.json`. Running a report while the
campaign is incomplete deliberately returns code 2 and lists missing jobs.
All generated data stay under ignored `results/` directories.

The test distribution is deliberately broad and contains stress cases; its
success fraction is not an operational reliability estimate.
Fleet size changes the centered grid, initial phases and altitude distribution;
measured scaling includes those scenario changes, not just search complexity.
Three core seeds, two ablation seeds and one extra replay timing per variant
are small samples; timings are observations, not guaranteed speedups.
Numerical acceptance remains conditional on the model, envelope, tracking tubes, clock
bounds and integration accuracy. No physical calibration, formal collision
freedom, PX4 separation certificate or real-aircraft safety claim follows.
