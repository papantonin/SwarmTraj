# Native physical-planner migration

The execution checkpoint is Git `9887248`. This migration moves the remaining
physical-planning algorithms into the standalone C++20 library, while keeping
Python defaults, historical modes and an explicit Python validation oracle.
The point-mass planner is separate from PX4/Gazebo dynamics and controllers.

## 1. Stable execution checkpoint

`9887248` records the previously qualified construction/execution kernels,
documentation and regressions. Generated reports and benchmark telemetry remain
ignored under `results/`.

## 2. Independent individual validation

`validation.hpp` and `validation.cpp` implement full-horizon independent replay:
nominal integration, compatibility projections, both sides of held-command
switches and the original five refined samples, actual level/speed attainment,
target/sector tests, bounded first-entry localization and robust obstacle
clearance. Construction acceptance is never used as a validation result.
The original tolerances, check ordering and first rejection reasons are kept.
An unresolved 4096-evaluation contact search still fails validation.

The Python boundary continues to bind immutable aircraft identities, exact IDs
and declared profile families before resolving own physical parameters. Select
the reference explicitly with `validate_individual(..., backend="python")`;
`backend="cpp"` selects the native independent replay. The default follows
`Config.numerical_backend`. Missing native validation capability raises an error.

Targeted verification: 68 existing construction/contact regressions and 16
differential native-validation tests pass. Three standalone C++ tests pass.
The saved campaign replay checks **600 accepted plans**, rejects **600 corrupted
final commands** and **600 post-arrival endpoint obstacles**, and reports no
acceptance/rejection difference. First-contact times match exactly on this
sample. Evidence and frozen source/binary hashes are recorded in
`results/native-planner-20261030-085245/validation-replay.json`.
The complete integrated qualification is recorded in stage 5 below.

## 3. Native proposals and resumable catalogs

`random.hpp` implements the NumPy-compatible SeedSequence and PCG64 algorithms,
including 128-bit state, the pending uint32 cache, Lemire bounded integers and
draws that are subsequently overridden by proposal rules. The implementation
uses portable 64/32-bit arithmetic, without a compiler-specific 128-bit type.
Golden and interleaved tests compare exact results and resumed states against
the installed NumPy 2.5.4. Algorithm references are the tagged primary sources:
[SeedSequence](https://github.com/numpy/numpy/blob/v2.5.4/numpy/random/bit_generator.pyx),
[PCG64](https://github.com/numpy/numpy/blob/v2.5.4/numpy/random/src/pcg64/pcg64.h) and
[bounded distributions](https://github.com/numpy/numpy/blob/v2.5.4/numpy/random/src/distributions/distributions.c).
This is an independent implementation; SeedSequence and PCG are credited to
Melissa O'Neill and NumPy compatibility follows the referenced algorithms.

`proposals.cpp` performs seeded level/departure/speed choices, proposal-only
visibility routing, physical construction and compatibility projections.
`catalog.cpp` preserves candidate IDs, cumulative attempts, mixed-family
unlocking, hint alternation, own-rejection adjustments and independent validation.
Accepted trajectories are immutable and remain unchanged across extensions.
The Python adapter binds aircraft identities and public read-only arrays.
Existing spawn-based catalog orchestration is retained; no thread pool is added.

Verification: six exact RNG tests, seven differential catalog tests and the
44 backend/RNG/individual-validation regressions pass. Four standalone C++
tests pass. The catalog comparisons include multiple resumes, guided departure
windows, overwritten draws, strict/rich/mixed profiles, obstacle routing and
own-rejection feedback. Selected native catalog plans are revalidated by Python.

## 4. Bounded joint search and independent final verification

`search.cpp` preserves catalog order, ID/MRV tie ordering, cumulative node
budgets and stable leximin branch-and-bound margin refinement. Pair compatibility
keeps the original `required + 1e-7` threshold. Witness and expansion priorities
retain their deterministic ordering and remain proposal guidance only.

`reservations.cpp` preserves full-horizon inflated cell/time occupancy, closed
interval intersections, construction budgets, guarded departure windows and
nextafter-excluded safe-interval endpoints. Explicit budget errors propagate
without turning a finite construction limit into physical infeasibility.

`joint_validation.cpp` recomputes selected full-horizon pair bounds, obstacle
clearance and fresh reservations independently of search caches. Python still
owns spawn-process scheduling when requested; the native search adapter owns
only immutable candidate copies and never receives aircraft telemetry.
Changed candidate geometry under a cached identity is explicitly rejected.

Verification: 11 native-search differential/oracle tests pass, including an
exhaustive finite-catalog margin optimum, both orderings, shared budget limits,
pair thresholds, grid conflicts, departure windows, immutable cache identity and
independent final rejection. The combined backend/catalog/search run passes 39
tests; five standalone C++ tests pass. Complete integrated qualification follows.

## 5. Complete native mission planner

`mission.hpp` exposes `plan_mission(vector<Designation>,
vector<PlanningSettings>)` without Python objects, callbacks or Python runtime
dependencies. `approach.cpp` implements the square shortest-augmenting-path
assignment, retaining SciPy's column/tie ordering. `mission.cpp` performs input
preflight, sorted-ID assignment, catalog extension, adaptive search, bounded
margin refinement, timing advice and independent final full-horizon checks.
Physical parameters and sphere radii remain specific to each aircraft. Horizon,
arrival deadline and minimum post-arrival duration remain distinct.

The Python adapter resolves immutable profiles and returns public read-only
trajectories. `Config(numerical_backend="cpp", generator_mode="physical_mission",
dimensions=3, planning_workers=1, ...)` uses one complete native mission call.
The existing CLI options select the same path:

```bash
./scripts/build_native.sh
.venv/bin/python examples/trajectory_3d.py --no-gui \
  --generator-mode physical_mission --numerical-backend cpp --workers 1 \
  --execution-backend cpp --output results/full-native-example
```

Requested multiple workers keep Python's spawn-process orchestration and native
catalogs, assignment, search and final verification. Within the native backend,
one-worker and spawned outputs must have identical signatures. The default
numerical and execution backends remain Python. Older extensions must be rebuilt:
`planning_api=1`, together with source/compiler/FP/binary identities, identifies
the complete planner. CLI, editor, profile loading, serialization, visualization
and reports remain Python. The execution backend is an independent runtime choice;
global execution observation remains Python with either backend.

### Integrated qualification, 30 October 2026

Evidence is retained in `results/native-planner-20261030-085245/`:

- **1,080 pytest tests pass in 591.55 s**, with four existing Gymnasium
  float32-bound warnings. Two subsequent packaging regressions pass separately.
- All **six standalone C++ tests** pass in Release and with ASan/UBSan.
  The standalone mission executable links standard C/C++ libraries only.
- The saved campaign completes **259/259 jobs**: 230 exploratory missions,
  19 negative controls and 10 serial/spawn replays. Statuses, candidate counts
  and search-node counts match the previous construction-only C++ checkpoint
  for all 259. All 192 exploratory successes are retained; the remaining 38
  cases retain their original failure classifications.
- All **72 execution expectations** pass with C++ local execution and the
  independent Python observer, including deliberately out-of-clock rejections.
  All 10 native serial/spawn comparisons have exact signatures.
- The reporter explicitly calls the **Python** individual validator: **600
  accepted plans**, **803 full-horizon pairs**, **51 reservation pairs** and
  **537 pull-out connections** pass. It rejects all **600 corrupted final
  commands** and **600 post-arrival endpoint obstacles**. There are no input,
  journal-audit or numerical revalidation errors.
- Comparing all 600 selected plans against that previous checkpoint gives
  maximum state-component difference `9.10e-13`, control-component difference
  `4.08e-14` and speed-reference difference `1.78e-15`, within the predeclared
  `1e-9` state / `1e-10` command tolerances. Candidate IDs and phases match.
- Pure Python and native wheels, plus a native wheel rebuilt from the source
  archive, import and plan outside the editable checkout. Packaged native plans
  also pass Python validation and retain the compiled numerical source identity.

The original full-suite/campaign source freeze and compiled binary remain
unchanged. Wheel qualification found a setuptools absolute-source-path error;
the subsequent `setup.py` correction only makes source/dependency paths relative.
Its two regressions and actual wheel/archive builds pass. The original failing
wheel log and pre-fix setup file are preserved alongside separate packaging
evidence; old manifests are not rewritten. `runtime-qualification-source-freeze.json`
records the runtime freeze, `distribution-verification.json` records packaged
imports, and `saved-array-comparison.json` records selected-plan equivalence.

No GUI widget or PX4/Gazebo controller was changed or newly flight-qualified
during this planner migration. This remains numerical validation conditional on
the point-mass model and tracking/clock assumptions, without a certified RK4
integration-error bound or a real-time flight guarantee.

### Paired planning measurements

`paired-final/` uses three fresh serialized processes per backend/case, identical
saved three-aircraft inputs, seeds, margins and budgets, one planning worker and
one BLAS/OpenMP thread. Hardware: Intel Core i3-1115G4, two physical cores/four
logical CPUs; Linux x86-64, CPython 3.12, GCC 13.3, strict float64 FP.
Means below come from this experiment; desktop load is uncontrolled.

| Saved case | Python planning | Complete C++ planning | Wall-time ratio | Planning process CPU reduction | Process peak RSS, Python / C++ |
|---|---:|---:|---:|---:|---:|
| Baseline | 5.887 s | 0.0586 s | 100.5x | 99.00% | 79.0 / 77.1 MiB |
| Strict rich profile | 7.162 s | 0.0832 s | 86.1x | 98.84% | 83.8 / 81.5 MiB |
| Grid reservations | 60.350 s | 0.6635 s | 91.0x | 98.89% | 95.4 / 89.6 MiB |

All 18 trials complete and preserve statuses, candidate counts and search nodes.
The grid request retains `catalog_exhausted`, with 3,265 nodes; faster completion
does not turn that finite failed search into a validated plan. Maximum compared
plan-component difference is `1.20e-12`. Native serial/spawn equivalence is
qualified separately by the complete campaign. Retained public array storage
is identical: 205,512 bytes for baseline and 504,336 bytes for rich plans.
Peak RSS includes Python/imports and retained arrays; the observed small
differences do not establish a substantial whole-process memory reduction.
Three repeats are not a percentile, worst-case time or real-time deadline proof.

### Complete CLI through static 3D rendering

`pipeline/` measures 18 fresh serialized CLI launches: three repeats, two
three-aircraft configurations, and three planning/execution combinations in
alternating order. It includes imports, planning, full-horizon execution with
the independent Python observer, JSON/CSV exports and a static 3D PNG.
These CLI fixtures are separate from the saved planning-only inputs above.

| Mean whole process | Python planning + Python execution | C++ planning + Python execution | C++ planning + C++ execution |
|---|---:|---:|---:|
| Reference, 90 s horizon | 13.048 s | 5.375 s | 3.346 s |
| Rich, 210 s horizon | 18.035 s | 10.032 s | 5.110 s |

The measured full-pipeline ratios with both native backends are **3.90x / 3.53x**
versus Python. Worker CPU decreases from 12.578 to 3.106 s and from 17.516 to
4.770 s. Peak RSS remains about 140 / 152 MiB: the retained arrays, interpreter,
observer and renderer remain allocated. Native planning itself takes about
0.075 / 0.107 s in these CLI trials; local execution plus Python observation
takes 0.613 / 1.389 s. Static rendering takes 1.481 / 1.590 s and
startup/exports/other work 1.176 / 2.025 s. Those stages now dominate the pipeline.

All 18 launches pass planning and execution and produce a 3D image. Maximum
plan-component and CSV telemetry differences are `1.20e-12` and `2.96e-12`.
Selecting either runtime execution backend preserves the native mission hash.
Frozen manifests, per-run stage measurements, reports and images remain under
the artifact directory; `measurements.md`, `measurements.json` and
`performance.png` / `performance.pdf` summarize the evidence.

These measurements do not cover interactive animation FPS, Gazebo, electrical
energy, networking or live drone state estimation. Faster computation does not
change the fixed designation snapshot, simulated horizon, controller update
schedule or tracking/clock margins, and does not establish an activation deadline.

No departure bound, margin, deadline, physical envelope, validation sample or
search budget may be weakened during migration. Runtime timings remain outside
canonical mission fingerprints.
