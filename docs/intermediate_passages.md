# Intermediate-cell passage-window guidance

This geometric passage-fitting feature is retired from production. The source
and experimental results below describe the historical implementation in Git
at `300de37`. Current physical C++ planning rejects this request explicitly;
see [production and archive policy](planner_defaults.md).

## Scope and explicit permissions

Enable `--generator-mode geometric --passage-guidance safe_windows` with grid reservations and smooth speed
permission. The default is `none`. This opt-in requires coupled 3D flight;
strict constant-speed requests are rejected as incompatible, not silently
changed. It works independently of departure-window guidance, including when
the departure interval is fixed to `0 0`.

The new heuristic changes speed progressively **before physical rollout** to
move an intermediate cell visit into a forecast free interval. It never hovers,
extends the departure bounds, modifies an accepted plan, or removes validation.
It is not a complete SIPP/kinodynamic planner or a real-aircraft safety proof.

## Offline algorithm

1. Retain the geometric reference for each individually accepted candidate in
   its ground-side catalog. This cache is enabled only for passage guidance.
   It is not a flyable certificate and is never exported to an aircraft.
2. On catalog expansion, use existing reference aircraft plans: the first plan
   per nonempty catalog for feasibility, or the feasible incumbent for margin
   refinement. These are provisional references, not fleet commitments.
3. Find shared inflated grid cells whose occupancy intervals actually overlap.
   Restrict own visits to at least 16 s after departure and at least 20 s before
   nominal arrival. From eligible cells choose the visit nearest the temporal
   midpoint between departure and arrival, with deterministic cell/time ties.
4. Merge other aircraft's blocked intervals for that cell and compute their
   complement on the common horizon. Exclude closed blocker boundaries with a
   1e-6 s guard. The existing `interval_max_constraints` bounds hint construction;
   overflow abandons this hint, not the whole mission or safety constraints.
5. Match the parent's physical visit location to its saved geometric reference.
   Fit a bounded C1 speed bump before this point, retaining the original
   departure and geometry. The bump ends at least two geometric seconds before
   the matched point and no later than 18 geometric seconds before arrival.
6. Roll out a completely new coupled trajectory. Run ordinary independent
   physical/target/obstacle/recovery validation. Rebuild inflated cell occupancy
   and require **every** visit to the requested cell to lie in the selected free
   interval. A missed cell, extra revisit or window overrun rejects the attempt.
7. Append only accepted new plans. Joint search still rechecks every pair and
   full-horizon reservations, followed by independent final checks. The selected
   fleet need not equal the provisional reference fleet.

This procedure can reject useful physical alternatives because the chosen cell
or free window is too restrictive. Such rejection does not prove infeasibility.
Conflicts during holding or final passage/recovery are not eligible for this
local maneuver. Ordinary geometry/profile exploration remains available.

## Speed fitting

Let `tau` parameterize the saved geometric reference, `q(tau)` its total
geometric speed and `V0` the original speed setpoint. On `[a,b]`:

```text
V_A(tau) = V0 + A sin²(pi (tau-a)/(b-a))
dt/dtau = q(tau) / V_A(tau)
|A| <= a_ref (b-a) V0 / (pi V_high)
```

Outside `[a,b]`, retain `V0` and zero reference-speed derivative. The allowed
amplitude interval is also intersected with the requested speed-reference
bounds. Since these geometric references have `q >= V0`, the amplitude bound
conservatively bounds `|dV/dt|`. Physical acceleration/actuator/stall checks are
independent and unchanged. Speed is not assigned instantaneously.

The reference arrival time at the matched point decreases monotonically with
`A`. Evaluate the two amplitude endpoints to obtain an achievable reference
time range. Intersect it with free windows eroded by a conservative forecast
half-dwell time, choose a deterministic feasible center, and use 40 bisection
steps to fit the amplitude. Reject unachievable windows, insufficient duration
or bumps smaller than 0.2 m/s. Actual dwell/clock changes are checked afterward
on the integrated trajectory, not assumed from this forecast.

## Budgets, determinism and mixed families

Guided attempts can occupy at most every other proposal slot; ordinary
exploration follows a rejected guided attempt. Each actual attempt consumes
the existing cumulative allowance. Identical deterministic fits for the same
parent/window context and profile requirements are tried only once, regardless
of success. Cached trial keys survive catalog resumption and spawn transfer.
This prevents duplicate physical plans and repeated unreachable-window attempts.

With [mixed profiles](mixed_profiles.md), fitting is used only when the current
unlocked proposal family allows smooth speed and has the same level count as
the saved reference. A constant family is not upgraded behind the user's back;
other proposal families and strict level requirements retain their semantics.
The cache adds memory and process-transfer overhead. No general runtime gain,
schedule completeness, battery/endurance or arbitrary-disturbance guarantee is
claimed. The clocks/tubes and numerical limitations of the
[flight model](flight_model_3d.md) still apply.

## Commands and diagnostics

```bash
# Four aircraft, static obstacles, strict two-level/smooth profiles.
.venv/bin/python examples/trajectory_3d.py --num-drones 4 --seed 19 \
  --target 500 500 --horizon 240 --activation-time 30 --approach-spread 180 \
  --obstacles examples/scenarios/static_obstacles.json \
  --reservation-policy grid --interval-guidance safe_windows \
  --generator-mode geometric --passage-guidance safe_windows --initial-candidates 2 \
  --cruise-levels 2 --speed-profile smooth --speed-profile-range 10 14

# Explicitly permitted mixed families; still four aircraft.
.venv/bin/python scripts/benchmark_intervals_profiles.py \
  --modes combined passages mixed_passages --seeds 7 19

# Different, explicitly obstacle-free simultaneous-departure experiment.
.venv/bin/python scripts/benchmark_intervals_profiles.py \
  --modes combined passages --seeds 7 19 --departure-delay 0 0 \
  --activation-time 0 --target 500 300 --horizon 210 --cruise-range 40 110 \
  --cruise-levels 1 --initial-candidates 1 --without-obstacles
```

Add `--no-gui` to the first command for headless execution. Preparation/loading
gating is a separate option as before. Changing the horizon or departure bounds
in these commands is explicit; the planner never makes these changes itself.

Ground-side `planning_report.json` contains `passage_contexts` (parent/reference
IDs, selected cell, anchor, free intervals and budget status) and
`passage_proposals` (candidate ID, fitted amplitude/window, reconstructed visit
intervals, acceptance/rejection). Individual aircraft receive only their own
physical plan/reservations and shared configuration, never these reference
calendars or other-aircraft telemetry. Timings remain outside mission hashes.

The regression corridor uses four curated straight geometric references to
isolate intermediate timing from route sampling. All aircraft have the same
destination, a fixed departure epoch and physically integrated speed profiles.
Initial references conflict; two guided candidates resolve the conflict. No
validator is mocked. Serial and spawned extensions agree, independent full
pair/grid checks pass, and nominal execution passes every acceptance criterion.

## Four-aircraft observations and retained failures

All final positive trials below passed every nominal execution criterion.
The strict paired modes retain identical physical constraints, safety margins
and budgets; the crossing pair differs only in `passage_guidance`.

| Scene / seed | Unguided attempts | Passage-guided attempts, total | Fitted / individually accepted / selected |
|---|---:|---:|---:|
| Large obstacles, variable departures / 7 | 59 | 59 | 0 / 0 / 0 |
| Large obstacles, variable departures / 19 | 71 | 54 | 2 / 1 / 0 |
| Obstacle-free crossing, fixed departures / 7 | 103 | 103 | 0 / 0 / 0 |
| Obstacle-free crossing, fixed departures / 19 | 106 | 26 | 3 / 1 / 0 |

The obstacle scene retains the existing large solids, target `(500,500,30)`,
240 s horizon, activation at 30 s, two strict levels, 180-degree spread and
smooth-speed permission. Its unguided signatures/counts are from the previous
saved strict comparison. The unused seed-7 guidance retained that exact result
signature. The crossing scene uses the explicit command above: no obstacles,
target `(500,300,30)`, 210 s horizon, activation/departure at zero and one level.
All four aircraft in that scene actually depart at zero. Its sampled pair
minima are approximately 49.15 m in the unguided trials and 49.15/41.99 m in
the guided seed-7/19 trials. Safety margins were not reduced.

The accepted guided alternatives were not selected in these final incumbents.
Inserting an attempt changes subsequent deterministic candidate IDs/profile
sampling, so reduced work is not proof that the fitted trajectory directly
resolved those full scenes. The separate curated corridor regression does
select both guided candidates and tests their actual conflict resolution.
Some scenarios use no eligible intermediate hint at all. No general speedup
or larger-margin guarantee follows. Timings in these trials overlap other
local checks and are **not isolated performance measurements**.

Mixed-profile compatibility was also exercised with four aircraft and the large
obstacles (seed 19): 36 attempts, one fitted/accepted alternative, none selected;
every nominal execution check passed. A full strict CLI launch including
simulated planning/loading acknowledgement gating passed as well. The four
individual payloads contain only their own physical plan/reservations, without
geometric caches or global passage diagnostics.

Retained artifacts:

- `results/benchmarks/passage-unique-fits-20261026.json`: final large-obstacle trials.
- `results/benchmarks/passage-crossing-final-20261026.json`: final paired fixed-departure trials.
- `results/benchmarks/passage-mixed-20261026.json`: mixed-profile compatibility.
- `results/trajectory3d/20261026-124839-688427/`: preparation/CLI/payload validation.
- `results/benchmarks/passage-fixed-departure-20261026.json`: four negative
  trials with the large-obstacle scene, fixed departure at 30 s and two strict
  levels. All return `no_candidate` in an initial catalog, before intermediate
  hints can help. This is finite-catalog failure, not continuous infeasibility;
  neither margins nor profile requirements were silently relaxed.
- `results/benchmarks/passage-variable-departure-20261026.json`: early prototype
  before fit deduplication. Seed 19 spent 164 attempts, including 58 fitted
  attempts with repeated deterministic alternatives. These observations led to
  the trial cache; they are not final-version results.
- `results/benchmarks/passage-crossing-interrupted-20261026.json`: incomplete
  exploratory console records. Editing the catalog class between active spawn
  rounds caused a version-mismatch exception; it propagated, not a partial
  acceptance. The final comparison was rerun in fresh processes with fixed code.

These are nominal numerical validations. No additional disturbed rich-profile
campaign, integration-error certificate or real-network guarantee is claimed.

The full regression suite passed **413 tests**, with four existing Gymnasium
float32 warnings. The focused passage/CLI/mixed-profile run passed 64 tests.
Coverage includes unreachable windows, closed boundaries, construction budgets,
strict-profile permission, duplicate-fit suppression, unchanged fixed departure,
actual inflated-cell visits, process-transfer immutability and the four-aircraft
corridor's serial/spawn equality and independently checked physical execution.
