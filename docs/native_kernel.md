# C++20 production engine

C++ is the only production backend for physical 3D planning and constrained local
point-mass execution. It covers dynamics, RK4, deterministic proposals, routing,
resumable catalogs, independent validation, joint selection and final rechecks.
Python retains the interface, deterministic spawn orchestration, immutable public
objects, independent audit equations, global observation, plots and reports.
See [production policy, installation and archive migration](planner_defaults.md).

```bash
bash scripts/setup.sh --trajectory
.venv/bin/python examples/trajectory_3d.py --no-gui
# Rebuild and run standalone native checks in an existing environment:
./scripts/build_native.sh
# Every ordinary wheel now includes the required extension:
.venv/bin/python -m pip wheel . --no-deps --no-build-isolation --wheel-dir results/native-wheels
```

The direct CPython buffer binding needs no pybind11 or scikit-build dependency.
Builds require C++20 and Python development headers; the CMake checker requires
CMake 3.20+. Linux x86-64 / CPython 3.12 / GCC 13.3 is the qualified environment.
Missing capabilities fail explicitly; no Python production fallback exists.
A single native call plans a serial mission. Parallel catalogs use Python spawn
scheduling around native proposal/catalog/search operations. Native output owners
keep transferred NumPy arrays immutable. The low-level proposal adapter wraps
C++ `physical_proposal`, including exact PCG64 state transfer, rather than
constructing a second Python recipe.

The standalone library remains available independently of Python:

```bash
cmake -S native -B build/native-check -DSWARM_BUILD_PYTHON=OFF -DSWARM_SANITIZE=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build/native-check --parallel 2
ctest --test-dir build/native-check --output-on-failure
```

The numerical contracts below remain applicable. Python-versus-C++ timings and
differential qualification below are historical measurements of the migration;
the retired Python production implementations remain in Git at `300de37`, not
in the installed package. Current regressions use stored historical negatives,
native determinism/transport checks and independent physical audit replay.

## Numerical and ownership contract

States are `[x, y, z, total_airspeed, heading, flight_path_angle, bank]` and
controls are `[thrust_acceleration, brake_acceleration, load_factor, roll_rate]`.
Units and equations match [the point-mass model](flight_model_3d.md). There are
seven doubles per state and four per command. RK4 scratch uses fixed stack
arrays; histories use contiguous vectors with reserved capacity. The binding
copies validated inputs before releasing the GIL and returns an immutable bytes
owner for NumPy views. It makes one output copy; this is not a zero-copy API.
Existing public `Trajectory` copies and compatibility projections remain intact.

The retained low-level construction adapter crosses the language boundary twice
per candidate: through the pull-out, then through full-horizon recovery/onward
flight. Python derives any
obstacle exit route from the actual pull-out state between these calls. There
are no Python callbacks or per-integration-step crossings. The rollout API also
accepts a complete held-command history in one call. Checked nominal rollouts
require zero wind and the reference two-substep integration; unchecked fixtures
can explicitly supply wind and another subdivision count.

The complete serial planner instead crosses once per mission, after Python has
resolved own-aircraft settings. Proposal routing, pull-out and onward routing,
catalog extension, individual replay, joint search and final rechecks all run
inside the library. Requested multi-process catalogs retain the Python scheduler
and cross at catalog/search operations. Neither path calls Python from C++.

Both backends preserve nominal two-substep endpoints, independently reintegrated
four-substep samples at fractions 0, 0.25, 0.5, 0.75 and 1, and the nominal endpoint
envelope check. An independent validator still runs for every candidate,
including full-horizon contact, obstacle and actuator-switch checks. Joint pair
and reservation checks remain unchanged. Loaded own-aircraft profiles retain
their effective values and enclosing radii. Native histories are bounded to
1,000,000 steps; exceeding this implementation bound raises an error.

Both build paths disable fast-math and floating-point contraction. Float64
scalar C++ and NumPy calculations can differ in the last bits. Differential
tests compare physical states within `1e-9` and controls within `1e-10`, while
requiring the same acceptance, candidate counts, search nodes and phase structure.
Within the native backend, saved serial/spawn result signatures must match
exactly. Native mission documents record source hash, ABI, compiler, FP policy
and the actual extension binary's SHA-256 in their canonical identity. Rebuild
after editing native sources; a successful import alone does not rebuild them.

No native thread pool has been added. Existing deterministic spawn-based catalog
workers remain available. Fast native candidates can make pool startup dominate;
measure `--workers 1` before requesting more processes.

## Required local execution and own references

`simulate(plans, config)` uses C++ control and motion with physical 3D production
plans. Native `execution_api=1` and `reference_api=1` are required; missing
capabilities fail with rebuild instructions. Python/legacy execution requests
are rejected. The editor has no backend checkbox. PX4 controllers and Gazebo
physics remain separate.

An executor copies only its own immutable seven-state history and four-command
history into compact native storage once. At each actuator update C++ samples
the reference using the original two-substep RK4 and averages held feedforward
commands across the same local-clock interval, including trims before/after the
reference horizon. Python retains own-clock event scheduling and sensor draws;
it no longer integrates reference dynamics in the control path. The native
controller then updates the own-motion estimate, computes nominal/measured-state
guidance and applies protection. No peer state or true simulator wind enters it.
The independently implemented Python sampler remains available to audit these
native windows and evaluate global execution.

Protection retains the same actuator/bank clipping, **21 inclusive samples**,
**four RK4 subdivisions**, envelope tolerance **1e-8**, verified trim fallback
and **ten** blend trials as Python. Failure to verify a fallback still
invalidates execution; no physical state is clipped. This is sampled protection,
not a certified invariant set or emergency collision avoidance.

A separate native simulator call integrates each requested sample duration
independently from the interval's initial state, with four RK4 subdivisions and
the simulator-only disturbance. The Python scheduler still owns asynchronous
clock switches, sensor RNG draws and both incoming/outgoing boundary records.
Python retains full-horizon pair/obstacle checks, contact reintegration, envelope
diagnostics, tube/clock assumptions and final acceptance. The native kernel does
not choose its own acceptance criteria or bypass observer checks.

Execution reports record `execution_backend` and, for C++, the actual binary and
source identity. This runtime choice does not rewrite a saved plan or its
canonical mission hash. The extension ABI remains 1 for existing construction
operations; `execution_api=1` identifies the added execution operations.

Replay identical saved plans in fresh serialized processes with:

```bash
.venv/bin/python scripts/benchmark_native_execution.py \
  --missions results/my-run/mission.json --repeats 3 \
  --output results/native-execution-paired
# Or reuse the saved campaign's missions that have execution expectations:
.venv/bin/python scripts/benchmark_native_execution.py \
  --campaign results/native-development-20261029-185306/scaling \
  --output results/native-execution-campaign
```

This runner verifies canonical mission hashes and exact plan reconstruction,
freezes current sources and binary identity, and rejects changed plans. It
compares every discrete report decision/event and numeric telemetry within
absolute `1e-8` (report floats also allow relative `1e-10`). It checks historical
acceptance/failure lists where available and preserves errors/timeouts. Its
single-aircraft absent-pair distance is explicitly `+inf`; only this matching
sentinel is allowed, while nonfinite physical states or multi-aircraft distances
fail comparison. `--reanalyze --output <existing-run>` can recheck saved
measurements after changing only the analysis runner. It verifies unchanged
runtime sources, binary, mission inputs and journal/worker agreement, records
old/new runner hashes and telemetry hashes, and writes a separate
`reanalysis.json` while preserving the original summary. Its
timing covers full `simulate()`, including the unchanged Python observer;
imports, artifact export and plotting are outside that timer. Peak RSS covers
the whole fresh Python process and retained result arrays, not native scratch
alone. Common-airframe saved campaigns are supported; per-ID profile replay is
explicitly rejected by this benchmark rather than discarded by its decoder.
Dedicated execution regressions cover resolved own-aircraft profiles.

## Verification and paired measurements

```bash
./scripts/test.sh tests/test_native_backend.py tests/test_native_execution.py \
  tests/test_execution_boundaries.py tests/test_native_execution_benchmark.py
./scripts/test.sh
.venv/bin/python scripts/benchmark_native.py \
  --output results/native-paired --repeats 3
```

Replay the full saved mission matrix with the native backend using:

```bash
.venv/bin/python scripts/benchmark_scaling_campaign.py \
  --reference results/scaling-campaign-20261028/manifest.json \
  --suites controls core random replay --generator-mode physical_mission \
  --numerical-backend cpp --planning-workers 1 --execution-backend cpp \
  --output results/native-scaling
.venv/bin/python scripts/report_generator_replay.py results/native-scaling
```

The manifest records the explicit backend override and binary identity; worker
and reporter guards reject changed sources or a different native binary. Reports
keep exact recovery comparisons separate from C++/Python comparisons within the
declared absolute tolerances. The reporter explicitly uses the independent
Python physical validator, including corrupted-command and late-obstacle
negative checks. Historical execution expectations used the recorded runtime backend. Current
production always uses C++; omitting a compatibility flag also uses C++.

The production capability tests require the native extension and fail if it is
absent. Historical optional-kernel test snapshots could skip native tests; those
skips are not evidence for the current required engine. The standalone C++ tests also
cover a manufactured trim, heading wrap, envelope rejection and pull-out.

The initial construction-port snapshot passed **925 pytest tests** (660.26 s, four
existing Gymnasium float32-bound warnings), with frozen source hashes unchanged
during the run. Standalone release and AddressSanitizer/UndefinedBehaviorSanitizer
C++ tests passed. Pure Python and native wheels were built and imported from
extracted distributions; a native wheel rebuilt from the source archive also
planned a validated mission outside the checkout's import path. A four-aircraft
headless native CLI run passed planning and Python execution validation. No
Gazebo or interactive GUI qualification was performed for this numerical port.

The first paired run is preserved in
`results/native-development-20261029-185306/paired/`: fresh subprocesses,
one/two workers, identical saved inputs, seeds, budgets, constraints and 120 s
caps, BLAS/OpenMP threads limited to one. Hardware: Intel Core i3-1115G4,
two physical cores/four logical CPUs. These are single observations under
uncontrolled desktop load, not guaranteed gains or repeated-run percentiles.

| Three-aircraft physical planning | Python, 1 worker | C++, 1 worker | Observed ratio | Python/C++ peak RSS |
|---|---:|---:|---:|---:|
| Baseline | 7.66 s | 0.214 s | 35.7 | 79.44 / 79.23 MiB |
| Strict rich profile | 9.51 s | 0.343 s | 27.7 | 83.50 / 84.09 MiB |
| Grid reservations | 64.93 s | 3.458 s | 18.8 | 95.50 / 95.46 MiB |

All 12 paired trials completed, with identical statuses, candidate counts and
search nodes; serial/spawn signatures match within each backend. The largest
selected-array difference was `1.20e-12`. Grid remains `catalog_exhausted` in
both backends; a faster rejected search is still a rejected search. With two
workers the native times were 1.30, 1.43 and 12.05 s, illustrating spawn costs.
RSS includes interpreter/imports and allocator state; there is no demonstrated
whole-process memory reduction. Public retained array sizes are unchanged.

### Historical headless pipeline with construction-only C++

A subsequent experiment in `results/system-impact-20261029-195118/` measures
the existing CLI through planning, Python execution/observation, JSON/CSV
exports and a static 3D PNG. It uses three fresh processes per backend/scenario,
serialized in alternating backend order, with the same saved three-aircraft
inputs, one worker and one BLAS/OpenMP thread. Lightweight timing wrappers leave
the planner/executor source unchanged; process wall time includes startup and
shutdown. The earlier table remains the original planning-only experiment.

| Mean of three runs | Reference Python | Reference C++ | Rich Python | Rich C++ |
|---|---:|---:|---:|---:|
| Planning | 7.626 s | 0.199 s | 9.659 s | 0.234 s |
| Python execution and observation | 2.660 s | 2.775 s | 6.316 s | 6.319 s |
| Static 3D image | 1.576 s | 1.515 s | 1.665 s | 1.639 s |
| Startup, exports and other | 1.177 s | 1.165 s | 2.062 s | 2.051 s |
| Entire process wall time | 13.039 s | 5.655 s | 19.702 s | 10.244 s |
| Worker user + system CPU time | 12.583 s | 5.354 s | 19.023 s | 9.869 s |
| Mean process peak RSS | 140.14 MiB | 139.93 MiB | 152.33 MiB | 152.24 MiB |

The whole headless pipeline improves by **2.31x / 1.92x**, while planning in
these repeated observations improves by 38.3x / 41.2x. CPU-seconds decrease by
57.4% / 48.1%; total RSS remains comparable. Unported execution and rendering
dominate the native pipeline. Desktop load is uncontrolled, three repeats are
not a deadline/percentile qualification, and differences from the earlier single
observations must not be interpreted as algorithm changes.

All 12 runs pass planning and execution, preserve saved inputs and matching
decisions/counts/search nodes. Maximum plan-array and execution-telemetry
differences are `1.20e-12` and `2.96e-12`. The interactive `index.html`, PNG/PDF
chart, `summary.json` and frozen manifest retain the evidence. Interactive
animation FPS, Gazebo, networking and electrical energy were not benchmarked.
The simulated physical horizons remain 90 s and 210 s in both backends.

The full native replay in
`results/native-development-20261029-185306/scaling/` completed **259/259 jobs**:
230 exploratory missions, 19 negative controls and 10 serial/spawn replays.
It preserves the saved Python physical inputs and 120 s cap. Exploratory outcomes
are 192 validated, 17 `no_candidate`, 12 `initial_margin_not_certified` and nine
`catalog_exhausted`; all 192 previous Python physical successes and all 180
historical geometric successes are retained. All 72 nominal, gust/sensor and
deliberately out-of-clock execution expectations passed using the Python tracker.

All 225 uncensored previous physical outcomes retain status, counts and search
nodes. A separate saved-array comparison covers 570 selected plans: maximum
state difference `5.23e-12`, maximum command difference `3.67e-13`, with identical
smooth speed references. The five previously censored inputs now terminate in
0.53–0.57 s (two `no_candidate`) and 2.90–3.39 s (three `catalog_exhausted`).
These remain failed finite searches; no margin, deadline or budget was relaxed.
Historical timing records remain separate from the paired measurements above.

The generated comparative report independently reloads and validates **600
plans**, checks **803 full-horizon pairs** and **51 reservation pairs**, and
rejects all 600 corrupted final commands and all 600 post-arrival endpoint
obstacles. It checks 537 pull-out connections: two exactly and 535 within the
declared C++/Python tolerances. Artifact/input audits have no errors, all 19
negative controls match and all 10 native serial/spawn comparisons match exactly.
See `scaling/analysis/index.html`, `scaling/analysis/generator-comparison.json` and the separate
`saved-array-comparison.json` in the development directory. These checks retain
the then-current geometric/Python defaults and do not establish PX4/Gazebo qualification.

The kernel removes Python small-array allocation churn in the hot loop. It
does not remove interpreter/process memory, combinatorial search complexity or
the bounded proposal catalog's limitations. Execution tracking now has the
separate optional kernel described above; the geometric converter remains
Python. No RK4 error certificate, real-aircraft guarantee or
PX4/Gazebo timing/separation claim follows from this implementation.

## Local execution qualification (30 October 2026)

Evidence is under `results/native-execution-20261030-084101/`. The complete
suite passed **992 tests in 958.23 s**, with the four existing Gymnasium
float32-bound warnings. Five focused benchmark-analysis tests passed after the
analysis-only correction described below. Both standalone C++ tests passed in
Release and under ASan/UBSan. Pure Python and native wheels were built and
imported outside the checkout; the native wheel also built from the source
archive with matching numerical source identity.

The saved 24-mission execution matrix was replayed through both backends without
replanning: **72 pairs / 144 executions**, with 24 nominal acceptances, 24
gust/sensor acceptances and 24 outside-clock rejections per backend. All discrete
report decisions, event identities and historical acceptance/failure lists
match. The maximum component difference across compared arrays is `4.10e-12`,
below the predeclared `1e-8` threshold. Safety-filter tests additionally exercise
verified blends, failed trim fallback, boundary classifications, own-aircraft
profiles, fleet/alone isolation and both hidden actuator-switch sides. No
margin, sample count, tolerance, clock schedule or physical limit was relaxed.

The original campaign summary is retained as a failed **analysis**: it treated
the single-aircraft absent-pair `+inf` distance as invalid numerical telemetry.
`campaign/reanalysis.json` fixes only that sentinel handling, records both runner
hashes and telemetry hashes, and verifies unchanged runtime sources/binary,
missions and journal/worker agreement. `campaign/measurement-runner.py` matches
the original immutable manifest hash exactly. Physical NaNs/infinities and
multi-aircraft nonfinite distances still fail. A fresh three-profile
single-aircraft paired run also passes with the corrected runner.

The separate `paired-final/` experiment uses three fresh serialized processes
per backend/case, alternating order, after the test suite and campaign finish.
Hardware is the same Intel Core i3-1115G4, two cores/four logical CPUs, Linux
x86-64, CPython 3.12, GCC 13.3, with strict FP and one BLAS/OpenMP thread.
Both backends receive the exact same saved three-aircraft plans, disturbances,
sensor seeds and clock settings. These means cover **full `simulate()` including
the independent Python observer**, excluding imports/export/plotting:

| Saved plan / execution profile | Python mean | C++ mean | Wall-time ratio | CPU-time reduction | Process peak RSS, Python / C++ |
|---|---:|---:|---:|---:|---:|
| Reference, nominal, 90 s horizon | 3.507 s | 0.809 s | 4.33x | 76.7% | 87.5 / 88.2 MiB |
| Reference, gust/sensor/clock | 5.754 s | 1.142 s | 5.04x | 80.0% | 88.9 / 89.3 MiB |
| Rich, nominal, 210 s horizon | 8.077 s | 1.843 s | 4.38x | 77.3% | 103.3 / 103.7 MiB |
| Rich, gust/sensor/clock | 12.904 s | 2.698 s | 4.78x | 79.1% | 106.4 / 107.4 MiB |

All 12 pairs match reports and arrays (maximum `1.82e-12`). Retained evaluation
array bytes are identical: 2,232,496 for the reference and 5,208,496 for rich
plans. RSS is observed after simulation and compressed-array export in the fresh
process; interpreter/imports, saved plans and observer data remain allocated.
There is no demonstrated whole-process memory reduction. Native held-control
scratch avoids Python temporary-array churn, but the public histories retain
their existing shape. Desktop load is uncontrolled and three repeats are means,
not percentile/deadline guarantees. Horizons and controller update schedules
remain unchanged; this speeds offline computation, not simulated flight time.

### Complete CLI through static 3D rendering

`pipeline/` adds 18 fresh serialized CLI launches: three repeats of two saved
three-aircraft configurations and three backend combinations. The CLI performs
planning, full-horizon execution/observation, JSON/CSV export and static 3D PNG
rendering. Mean entire-process times in this experiment are:

| Scenario | Python construction + Python execution | C++ construction + Python execution | C++ construction + C++ execution |
|---|---:|---:|---:|
| Reference, 90 s horizon | 17.730 s | 7.377 s | 4.650 s |
| Rich, 210 s horizon | 24.764 s | 12.814 s | 6.963 s |

With both kernels the full pipeline improves by **3.81x / 3.56x** versus Python,
or **1.59x / 1.84x** over C++ construction with the Python executor. Worker CPU
time decreases from 17.346 to 4.394 s and from 24.405 to 6.600 s. Mean process
peak RSS stays comparable: 140.0 to 140.1 MiB and 152.1 to 152.2 MiB. Static
rendering and startup/export are now substantial remaining costs.

All 18 runs retain validated planning/execution, identical inputs and matching
candidate counts/search nodes. Maximum plan-array and CSV telemetry differences
are `1.20e-12` and `2.96e-12`. C++ construction missions have exactly the same
canonical hash with either runtime execution backend. The frozen source/binary
manifest, per-stage timing wrappers, reports and images are retained alongside
`summary.json`. These runs do not measure interactive animation FPS, Gazebo,
networking, electrical energy or a real controller deadline. Differences from
the historical construction-only experiment above reflect separate timing
sessions; ratios here use paired measurements from this session.

## Complete physical-planner qualification (30 October 2026)

The subsequent [full-planner migration](native_planner.md#5-complete-native-mission-planner)
ports proposals, independent replay, catalogs, assignment and joint selection
in addition to construction. It records 1,080 full-suite tests, two subsequent
packaging regressions, six standalone/sanitizer tests, 259 saved campaign jobs,
72 execution expectations and independent Python revalidation of 600 plans.
Its paired planning and complete CLI measurements supersede the
construction-only measurements above for the current implementation. Defaults
were Python/geometric at that checkpoint; the detailed guide separates planning gains from the
remaining Python execution observation, serialization and static rendering.
