# Phase 2 — physical recovery and complete individual mission construction

Production now runs only the C++ implementation of these physical algorithms.
The Python proposal builder and geometric production path have been removed;
Python remains the independent audit/observer. Historical comparison tables,
intermediate rollout decisions and commands below describe the migration study,
not current backend choices. See [production and archive policy](planner_defaults.md).

New coupled 3D missions default to `generator_mode="physical_mission"` and
`numerical_backend="cpp"`; [selection and archive rules](planner_defaults.md)
preserve explicit historical requests. The physical generator constructs a complete individual
mission directly through the seven-state point-mass model. It does not build a
legacy vertical trapezoid or convert a pull-out altitude loss into a global
vertical-acceleration coefficient. Independent physical, contact, obstacle and
joint-separation validation remain mandatory and unchanged.

This implements the first two development stages: an explicit physical
recovery primitive and bounded complete individual mission construction.
Control-space optimization with multiple shooting is a separate future stage.
Neither mode is a calibrated aircraft model or a real-aircraft certificate.

## Stage 1: recover from the actual boundary state

`kinodynamic_recovery.pullout_from_state(state, own_config,
max_duration_s=...)` starts at the supplied `[x,y,z,V,psi,gamma,phi]` state.
Position, altitude, airspeed, heading and slope are never reset. This first
primitive supports an unbanked descending entry; banked or unresolved fleet
physics are rejected, rather than silently flattened or ignored.

The result contains read-only states and held physical controls, duration,
observed altitude loss and explicit `entry_state`, `exit_state` and `dt`.
Pickle/process transfer reconstructs read-only arrays. The time budget rejects
an incomplete recovery; it never extends the mission horizon. Thrust, braking,
load and the actual evolving speed follow the existing RK4 dynamics. Held
commands are checked at both ends and three interior points with refined
integration. `validate_pullout` independently reintegrates the stored primitive.
The local check does not establish obstacle clearance or whole-mission safety.

`simulate_pullout` remains an isolated laboratory boundary fixture. The old
`recovery_seed_acceleration` function remains available for historical numerical
comparisons only; mission generation no longer calls it. In particular, a
physical altitude loss is no longer mapped to a coefficient shared by unrelated
climbs, descent entry and recovery transitions.

## Stage 2: complete individual missions

`physical_generation.py` constructs these phases on the original mission clock:

1. Continue the supplied initial loiter until the requested release boundary.
2. For rich requests, physically fly and hold the ordered cruise levels and
   a bounded smooth speed bump, then recover the primary level and setpoint.
3. Fly a physically integrated transit to an upstream approach gate while
   reaching the selected cruise altitude and accumulating a settled cruise hold.
4. Align with the approach and physically roll the remaining bank to zero.
5. Establish the requested descent slope using physical forces; for level
   passage, reach the target altitude through a physical altitude transition.
6. Continue through the target neighborhood on a settled approach.
7. Start the explicit pull-out from the actual post-passage state.
8. Recover cruise altitude and continue flying to the complete horizon.

Every stored state is propagated from its actual predecessor with a held
physical command. Phases never teleport, impose an airspeed after integration,
or splice different endpoint states. The approach length and cruise headroom
are proposed using unbanked physical rollouts. XY gates and optional visibility
waypoints are routing preferences only; the actual aircraft follows bounded
physical dynamics. A waypoint route is not an obstacle-clearance certificate.

The Python reference generator's repeated pitch-down loss and terminal-distance
proposals reuse bounded scalar caches within each planning process. Keys include
the complete resolved own configuration, airspeed and (for distance) cruise
altitude. Unhashable caller
configurations use the uncached path. These caches contain no aircraft states,
commands, RNG state or acceptance decisions. Actual candidate integration and
all independent validators still run; warm and cold caches must produce the
same complete plans, candidate counts and joint-search results. They only avoid
recomputing identical proposal-sizing fixtures.

The optional [C++ physical planner](native_planner.md) computes these bounded
sizing fixtures directly in compiled code and retains full candidate integration
and independent validation. It also ports proposals, catalog extension and joint
selection; one worker uses a complete standalone mission call.

A candidate is independently checked by the existing `validate_individual`
and contact-event code, including the requested angle at first sphere entry,
nominal arrival, cruise-before-arrival and final cruise recovery. Catalogs and
joint selection use the original full-horizon checks and margins. Each aircraft
still executes only its own immutable plan, state and clock.

### Supported and bounded scope

- Coupled 3D point-mass flight, one to three cruise levels, constant or smooth
  airspeed **reference**.
  Actual airspeed evolves dynamically. Layered and common-XYZ targets, level
  and descending passages are supported.
- Fixed cruise altitude or selection inside the explicitly configured range.
  No range, deadline, departure bound or physical limit is widened.
- Assigned approach sectors, bounded departure proposals and per-aircraft
  profiles. Catalog generation resolves own physics with `Config.for_drone`.
- Multiple-level requests use actual held physical controls while flying a
  bounded turn. Every declared level must be distinct and in range, and must
  be physically attained and held before arrival. They never fall back to a
  single level. Explicit mixed-family declarations remain supported.
- Smooth speed uses a C1 sine-squared bump on the real mission clock, starting
  four seconds after release and lasting 28 seconds. Its amplitude respects
  the requested reference range and acceleration bound. Actual speed evolves
  through thrust/braking with feedforward. The full initial loiter, final
  approach, pull-out and recovery retain the original setpoint. No reference
  retiming or instantaneous speed assignment is used.
- Altitude guidance anticipates braking with a distance-dependent vertical
  speed request; it does not wait until cruise altitude to decelerate. The
  same physical and independent dense envelope checks remain authoritative.
- Automatic levels interleave low recovery-cost levels, the original layer
  and seeded range exploration. Every eighth ordinary attempt explores the
  earliest legal release; conflict and interval hints may override it inside
  the unchanged departure bounds. Approach lead lengths also vary.
- Obstacle-derived above/below levels and lateral visibility routes are
  proposal alternatives. The post-passage route also avoids obstacle footprints.
  Full-horizon integrated obstacle and joint checks remain mandatory; a
  vertically cleared prefix never certifies the climb, descent or recovery.
- With obstacles and an unrestricted approach, attempts after the first five
  also explore deterministic heading offsets. This can move a long terminal
  transition away from a blocked join. Assigned approach sectors are preserved;
  all proposals still consume the original attempt budget and require independent
  physical, obstacle and joint validation.
- Geometric intermediate-passage fitting is still unsupported and rejected
  explicitly at configuration, without using the geometric generator. It is
  absent from the original 230 exploratory requests. No optimal spatial-routing
  or completeness claim is made.
- Existing candidate/attempt budgets bound exploration. Alignment, headroom,
  envelope, timing or recovery failure can exhaust the finite family. Such
  failure does not prove continuous-space infeasibility.

The earlier experimental name `physical_recovery_seed` is retained as a
compatibility alias for the **new complete physical generator**. Its semantics
therefore differ from the superseded scalar-seed prototype. It performs no
fallback to geometric generation. Source fingerprints distinguish old runs.

## Running and inspecting

```bash
# A complete individual mission at 14 m/s, with the original physical limits.
.venv/bin/python examples/trajectory_3d.py --no-gui \
  --generator-mode physical_mission --num-drones 1 --airspeed 14 \
  --altitude 80 --target 500 0 --horizon 120 --cruise-range 70 100 \
  --max-descent-rate 8 --departure-delay 0 0 --workers 1 \
  --candidates 1 --attempts 3

# Local primitive/ODE comparisons and complete-mission regressions.
./scripts/test.sh tests/test_kinodynamic_recovery.py tests/test_physical_generation.py

# Paired historical/physical individual trials; retain rejected plans as well.
.venv/bin/python scripts/benchmark_physical_missions.py \
  --output results/physical-mission-bench --execute

# Historical local-loss comparison, not an executable mission validation.
.venv/bin/python scripts/phase2_pullout_bench.py

# Replay the exact original 230 requests, 19 controls and 10 spawn repeats.
.venv/bin/python scripts/benchmark_scaling_campaign.py \
  --reference results/scaling-campaign-20261028/manifest.json \
  --suites controls core random replay --generator-mode physical_mission \
  --output results/physical-profiles-campaign
```

The study originally exposed generator/backend switches. They are removed from
current preparation: production is fixed to physical C++; archived settings
remain unchanged with launch disabled. See [production policy](planner_defaults.md).

## Initial single-level verification

Before physical rich profiles were added, the complete repository suite passed
**850 tests**. The four warnings are Gymnasium float32 box-bound conversions in
the separate PyBullet backend. Fifteen original geometric campaign jobs replayed
with exactly matching result signatures. The underlying dynamics, controllers,
contact resolution, independent validators and separation/search code were not
changed.

That revision's paired individual bench completed 24 planning jobs: 12 physical and
12 geometric, with identical inputs and budgets. It covers 12/13/14 m/s,
initial altitudes 30/80 m, targets `(140,50,30)` and `(500,0,30)`, horizon
120 s, one candidate and eight attempts. Physical construction validated 12/12;
geometric construction validated 4/12. All 24 nominal and bounded gust/sensor
execution trials of the physical plans passed. This is a small individual
integration bench, not a success-rate estimate for arbitrary fleet missions.

The saved-plan audit independently revalidated all 16 selected plans, checked
12 exact pull-out boundary connections, and rejected 12 corrupt final commands
and 12 obstacles placed at post-arrival endpoints. A two-aircraft planning
smoke also passed the existing joint selection, with a full-horizon distance
lower bound of 32.86 m against 16.80 m required; it is not a fleet scaling study.
Local reports, source hashes and logs are under ignored
`results/physical-mission-development/`.

## Extended physical-profile verification

The extended generator passed the complete repository suite: **871 tests** in
590.71 s, with the same four Gymnasium float32 warnings. Regression cases
exercise actual two/three-level holds, constant/smooth references, strict and
explicit mixed profiles, serial/spawn equality, read-only transfer, insufficient
range/time, and rejection of false level/speed metadata. A rich individual plan
also passed nominal and bounded gust/sensor execution trials.

The original seed-7 five-aircraft baseline now passes joint planning with its
unchanged 90 s horizon, 28-candidate/90-attempt limits and original separation,
tracking and clock margins. Additional cases cover overflight, underflight,
an obstacle after target passage, and rich two-aircraft grid allocation with
independent full-horizon pair and reservation rechecks. All 230 exact saved
exploratory configurations, including the 15 strict-rich requests, decode
without reducing their requirements.

These checks do not substitute for the complete replay. The observer-corrected
campaign is preserved under `results/physical-profiles-observer-campaign-20261029/`.
It was interrupted after 84 jobs to investigate two short-horizon planning
timeouts. Its saved selections independently revalidated 180 individual plans,
240 pairs and 180 pull-out connections without numerical errors; this partial
coverage is not a qualification result. The scalar-cache revision uses a new
campaign, `results/physical-cached-campaign-20261029/`.
Its immutable manifest changes only the saved generator mode and records the
extended execution coverage: first-seed validated baseline, obstacle,
heterogeneous, strict-rich and grid families receive nominal, bounded
gust/sensor and deliberately out-of-clock trials. The original 120 s
per-process limit still includes optional execution; any unfinished execution
remains explicit, even when planning completed successfully.

## Scalar proposal caching and finalization

The observer-corrected scalar-cache revision passed **880 tests** in 642.48 s,
with the same four Gymnasium float32 warnings. The focused physical-generation,
recovery, observer-boundary and campaign/report checks passed **80 tests** in
108.77 s. Regressions compare complete plans, candidate counts and search work
with cold, warm and disabled caches, and retain support for unhashable caller
configurations. These tests do not replace the full saved-input campaign.

The proposal-sizing microbenchmark replays the exact saved
`core-short_horizon-n1-s7` request. Before caching it took 30.60 s, including
90 repeated headroom rollouts and 90 terminal-distance requests with 45 distinct
inputs. After caching it took 25.00 s, with one headroom rollout and the same
90 candidate attempts and exact result signature. The observed 18.3% reduction
is a single paired desktop measurement, not a guaranteed speedup. Evidence and
source fingerprints are under `results/physical-cached-finalization-20261029/`;
the pre-cache evidence remains under `results/physical-finalization-20261029/`.

## Completed saved-input comparison

`results/physical-cached-campaign-20261029/` records all **259 requested
outcomes**: 230 exploratory missions, 19 negative controls and ten serial/spawn
replays. Its manifest preserves the historical snapshots, seeds, physical
limits, margins, mission requirements, candidate/search budgets and 120 s
process limit. Only `generator_mode` changes in the saved mission inputs.

| Aircraft | Historical geometric | Current physical |
|---|---:|---:|
| 1 | 43/46 | 43/46 |
| 2 | 36/46 | 37/46 |
| 3 | 34/46 | 34/46 |
| 4 | 35/46 | 39/46 |
| 5 | 32/46 | 39/46 |
| Total | **180/230** | **192/230** |

All 180 historical planning successes are retained, with 12 gains and no lost
successes. Gains comprise the seed-42 five-aircraft baseline, all three
five-aircraft deadline cases, all six four/five-aircraft heterogeneous-speed
cases, and random cases `n2-s1003` and `n4-s1005`. All 15 strict-rich requests
validate without simplifying their level or speed requirements. No request is
reported as an unsupported configuration. The other 38 exploratory outcomes
remain explicit: 15 `no_candidate`, six `catalog_exhausted`, 12
`initial_margin_not_certified` and five `planning_timeout`.

All 19 negative controls match their expected statuses. All ten serial/spawn
replays match exact result signatures. The 72 requested execution trials meet
their expectations: 24 nominal and 24 bounded gust/sensor trials are accepted;
24 deliberately outside-clock trials are rejected. No trial is missing or
duplicated. Planning failures have no selected plan to execute and are never
counted as passed execution trials.

Separate numerical revalidation checks **600 individual plan records**, 803
full-horizon pairs, 51 reservation pairs and 537 exact pull-out connections.
The plan count includes the repeated selections from the ten replay jobs.
All 600 corrupt final-command trials and all 600 post-arrival endpoint-obstacle
trials are rejected. Input/hash audits and numerical revalidation report zero
errors. In addition, all 82 completed pre-cache results retain their exact
signatures after caching, and 15 historical geometric jobs replay unchanged.

In that campaign, five jobs hit the unchanged 120 s process limit: short-horizon
five-aircraft cases at seeds 7 and 19, and grid three-aircraft cases at all
three seeds. These are externally censored planning outcomes, not continuous
infeasibility claims. All 259 outcomes are present, but only 254 workers
finish normally. The report's promotion decision remains **ineligible solely
because of these five timeouts**; at that historical stage `geometric` remained
the default. Current production policy is physical C++ only.

A subsequent [migration check](generator_migration.md#measured-migration-checks-29-october-2026)
completed those five saved inputs within 49–64 s after rollout allocation and
envelope batching optimizations. Their outcomes are bounded catalog failures,
not new planning successes. This does not rewrite the historical 259-job journal;
the migration guide also records the separate native startup blocker.

The run used an Intel Core i3-1115G4 (two physical cores, four logical CPUs),
one BLAS/OpenMP thread per worker, serial jobs and one planning process except
the explicit two-worker replays. Summed worker wall time is 106.2 minutes,
compared with 34.1 minutes for the same historical 259-job subset and 112.4
minutes for the earlier physical revision. These are different source
revisions measured at different times on an uncontrolled desktop, with 72
execution trials now versus 39 historically. They do not isolate the cache's
effect or establish a general speedup; physical generation remains more costly
in this experiment. Timing statistics exclude censored planning durations and
must be read alongside the five timeouts.

The comparative HTML, PDF, CSV, audit and numerical checks are in the campaign's
`analysis/` directory, particularly `index.html`, `generator-comparison.json`
and `completion.json`. Full-suite logs, source snapshots and the cache
signature comparison are in `results/physical-cached-finalization-20261029/`.
See [campaign reproduction and reporting](scaling_campaign.md#comparing-physical-and-geometric-generators).
Those are point-mass numerical experiments; no new PX4/Gazebo flight or
interactive GUI smoke test was run for that cached-revision finalization.
The later migration checks linked above record separate native experiments.

## Historical scalar-prototype measurements

Before these two stages, the supplied scalar-seed patch passed its local tests
and reproduced the 13/14 m/s recovery figures. However, a 15-mission paired
sample validated 12 missions geometrically and only two with the scalar seed;
ten historical successes were lost and none gained. Its coefficient affected
all vertical transitions, including pre-arrival phases. Those results remain
under `results/phase2-validation-20261028/`; they must not be attributed to the
new physical construction or regenerated silently with the compatibility alias.
The pre-change files are also preserved locally under
`results/physical-mission-development/before/`.

The original 418-job campaign remains the broader reference. Changing a local
primitive or passing the single-aircraft bench is insufficient to change the
default. The complete saved-input replay above now includes the original
negative controls. Further work includes resolving the timeout-prone search
families without relaxing their constraints, bounded trajectory optimization,
broader fleet comparisons, and heterogeneous-airframe and obstacle/disturbance
sweeps. Existing
PX4/Gazebo experiments are separate and inherit no point-mass timing or safety
certificate. RK4 and sampled checks still have no formal integration-error bound.

The first physical scaling campaign used frozen source files from before
rich-profile support and completed all 259 requested jobs (230 exploratory,
19 controls and 10 replays). It validated 162/230 exploratory missions against
180/230 historically, with 15 unsupported requests, 25 lost historical
successes and seven gains. Its saved-plan audit checked 483 individual plans
and 619 pairs without numerical revalidation errors. Those measurements
describe the earlier generator, not the extension above. Its inputs and
journal remain under `results/physical-scaling-campaign-20261028/` and are not
replaced or mixed with new results. The extended revision's separate,
source-fingerprinted replay is reported above. Configuration support does not
imply that every constrained mission is physically possible
or that bounded catalog search finds a joint selection. Stress-test failures,
timeouts and execution failures must remain visible. The geometric default is
retained pending the [migration qualification gates](generator_migration.md#gates-before-changing-the-default).
