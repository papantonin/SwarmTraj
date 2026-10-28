# Explicit adaptive mixed-profile catalogs

## Purpose and configuration

Rich altitude/speed profiles cost more to generate and validate. Rather than
imposing them on every proposal, `--profile-policy adaptive_mixed` explicitly
permits a mixture of simple and richer candidates. This is an offline proposal
heuristic, not a new flight model or collision-avoidance controller.

The default remains `strict`: requesting two levels and smooth speed still
requires every accepted plan to satisfy both, even after repeated failures.
There is no automatic switch from strict to mixed mode.

Only with the explicit mixed policy:

- `--cruise-levels N` becomes a **maximum**, not a required minimum.
- `--speed-profile smooth` permits, but does not require, speed variation.
- `--speed-profile constant` never permits smooth-speed families.
- `--cruise-levels 1` never permits multi-level families.
- The cruise range, speed-reference bounds and acceleration cap remain intact.
  Actual speed remains subject to the independent physical envelope.

Mixed mode requires adaptive planning and coupled 3D flight. If both profile
settings remain at their simple defaults, its only family is baseline; the
policy alone does not enable extra capabilities.

## Deterministic staged exploration

For two allowed levels and smooth speed, the ordered menu is:

```text
baseline -> levels_2 -> speed -> levels_2_speed
```

The initial catalog starts with baseline (one cruise level, constant reference).
When joint search asks to extend a conflicting aircraft's catalog, unlock the
next family for that aircraft only. A low-margin extension also unlocks one
family. With three allowed levels, constant families `levels_2`, `levels_3`
precede `speed`, then combined families follow in increasing level count.

An initially difficult catalog must not exhaust its whole attempt allowance
without trying other permitted families: after `max(2, initial_candidates)`
consecutive rejections, unlock the next family. A successful acceptance or new
unlock resets this rejection streak. This threshold is a deterministic work
heuristic, not a physical feasibility criterion.

After each unlock, try the newest family first, then cycle all unlocked families
in reverse menu order. Simple candidates remain available. Every proposal,
including rejected rich profiles and guided proposals, consumes the original
cumulative per-aircraft attempt budget. Accepted candidates consume the same
catalog cap as before. No extra planning/search budget or horizon is introduced.

The stage, cursor, rejection streak, attempt ID and PRNG state survive catalog
resumption and spawn-worker transfers. Family selection does not draw extra
random numbers. Generating different profiles can consume different subsequent
draws, so mixed mode may legitimately select different plans from strict mode.
Wall-clock timings do not enter the mission hash.

## Independent validation and execution

Each mixed plan declares an immutable `profile_family`. The independent
validator checks that this declaration is allowed, resolves the corresponding
strict profile requirements, and runs the existing integrated-physics checks.
Unknown/missing labels fail; declared multi-level/speed profiles must still
provide and attain their required metadata and physical motion.

All original dynamics, actuator, stall, altitude, obstacle, target-passage,
recovery and full-horizon checks remain. Joint selection still checks physical
pair-distance bounds and, when enabled, cell/time reservations. No validated
plan is retimed, reshaped or upgraded after selection. Earlier catalog plans
are retained unchanged. A permitted family or individually accepted candidate
does not establish joint safety.

The executor uses only its own plan, clock and measured state, as before.
Other-aircraft reference windows and catalog search metadata stay ground-side.
Mixed mode retains numerical/model-conditional guarantees only: no certified
integration-error bound, arbitrary-disturbance guarantee, completeness or
real-aircraft certification is added.

## Use and comparison

```bash
.venv/bin/python examples/trajectory_3d.py --num-drones 4 \
  --target 500 500 --horizon 240 --approach-spread 180 \
  --obstacles examples/scenarios/static_obstacles.json \
  --activation-time 30 --reservation-policy grid \
  --interval-guidance safe_windows --initial-candidates 2 \
  --profile-policy adaptive_mixed --cruise-levels 2 \
  --speed-profile smooth --speed-profile-range 10 14

# Identical snapshot, physical constraints and budgets; four aircraft only.
# Strict families and explicitly allowed mixed families are different requests.
.venv/bin/python scripts/benchmark_intervals_profiles.py \
  --modes windows combined mixed_windows --seeds 7 19
```

The first command opens the existing scenario editor and 3D playback; use
`--no-gui` for headless execution. Preparation acknowledgement gating is optional
and unchanged; see [mission pipeline](mission_pipeline.md).

`planning_report.json` records each catalog's `profile_events` (unlock reason
and attempt ID) and `profile_proposals` (attempt ID, family, individual acceptance).
Execution reports give `profile_policy` and `selected_profile_families`.
Strict historical individual plans omit the optional family field.

The benchmark retains failures, configuration, deterministic signatures,
proposal counts, family counts and timings. Mixed catalogs need not improve
runtime, margin or diversity on every scene. In particular, success in the
initial simple catalogs may make richer profiles unnecessary. Compare workload
and acceptance criteria alongside runtime, not as equivalent trajectory requests.

## Four-aircraft observations

On the Intel i3-1115G4 host (two physical cores, four logical CPUs), with four
catalog workers, seeds 7 and 19, unchanged large obstacles, 180-degree spread,
240 s horizon and grid/window guidance, all six nominal executions passed every
execution acceptance check. Each cell below gives **attempts / planning seconds**:

| Seed | Simple only (`windows`) | Rich required (`combined`) | Mixed permitted (`mixed_windows`) |
|---|---:|---:|---:|
| 7 | 17 / 4.65 | 59 / 12.75 | 18 / 4.84 |
| 19 | 23 / 6.53 | 71 / 16.76 | 63 / 14.50 |

These are single measured runs, not timing distributions or real-time promises.
The benchmark was not run concurrently with this change's test suite, but
external desktop load was uncontrolled. In these scenes, mixed mode reduced
work relative to globally imposed rich profiles, but simple-only was still
cheaper. The allowed trajectory families differ; this is not a speedup while
requiring identical profiles or identical selected paths.

Seed 7 selected three baseline plans and one two-level plan; minimum sampled
separation was 43.61 m and obstacle-surface distance 35.68 m. Seed 19 selected
three baseline plans and one speed-profile plan; corresponding minima were
51.17 m and 33.41 m. Seed 19 exercised actual conflict-driven unlocks, including
individually accepted combined level/speed alternatives. Its 63 proposals
comprised 25 baseline, 17 level, 11 speed and 10 combined attempts.
Numerical margins are not globally optimal: the seed-7 rich-required combination
had a larger pair margin than mixed, and the objective does not optimize
obstacle margin.

Raw results: `results/benchmarks/mixed-profiles-validation-20261026.json`.
The four strict-mode result signatures (two seeds, simple/rich modes) exactly
matched the prior saved benchmark signatures, including every individual plan.
New configuration/diagnostic fields still legitimately change mission manifests.

The complete regression suite passed **399 tests**, with four existing Gymnasium
float32 warnings. Focused tests cover strict non-fallback, targeted conflict
unlocks, cumulative budgets, forbidden declarations, physical validation,
immutable process transfers and serial/spawn reproducibility. A separate
headless launch with simulated preparation/acknowledgements also passed:
`results/trajectory3d/20261026-120513-462282/`. Its four exported payloads contain
only their own plan/reservations, not global catalog diagnostics. These checks
exercise nominal execution; no new disturbed rich-profile campaign is claimed.
