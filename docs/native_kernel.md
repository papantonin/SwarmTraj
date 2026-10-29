# Optional C++20 physical construction kernel

The `physical_mission` generator can use an explicit C++ numerical backend.
Python remains the default and the independent validation reference. The
standalone library covers seven-state flight dynamics, held-control RK4,
construction envelope samples, local construction guidance, terminal fixtures,
pull-out and complete physical candidate construction. Proposal sampling,
per-aircraft random streams, routing hints, catalogs, joint selection, execution,
visualization and reports remain in Python.

## Build and run

Use the existing Python 3.12+ environment. A C++20 compiler, CMake 3.20+ and
Python development headers are required for the local extension build:

```bash
./scripts/build_native.sh
.venv/bin/python examples/trajectory_3d.py --no-gui \
  --generator-mode physical_mission --numerical-backend cpp --workers 1 \
  --output results/native-example
```

The build script compiles in ignored `build/native/`, puts the extension beside
the Python package and runs the standalone C++ tests. For a distributable wheel:

```bash
SWARM_BUILD_NATIVE=1 .venv/bin/python -m pip wheel . --no-deps \
  --no-build-isolation --wheel-dir results/native-wheels
```

An ordinary installation or wheel build keeps the pure Python package. No new
Python build dependency is required: the small binding uses CPython's buffer API
directly. pybind11 and scikit-build-core can be considered if a larger binding
surface justifies them. Linux x86-64 / CPython 3.12 / GCC 13.3 is the tested
platform; other compiler/platform branches have not been qualified.

Library callers select `Config(generator_mode="physical_mission",
numerical_backend="cpp", ...)`. A missing/incompatible extension fails explicitly
before catalog generation. There is no automatic fallback. The option requires
coupled 3D physical generation; geometric, planar and legacy modes reject it.
Shared PX4 editor scenarios now save the backend explicitly in v3 records.
Historical v1/v2 records select Python; v3 requires both generator and backend.
The editor's **C++ construction (off: Python)** checkbox is enabled only for
physical coupled 3D generation. A missing extension or incompatible selection
is rejected without switching backends silently. Native PX4 controllers and
Gazebo aerodynamics remain unchanged.
The subsequent [editor/startup qualification](generator_migration.md#c-editor-and-ordered-startup-qualification-30-october-2026)
records the 960-test suite, readiness/load checks and one single plus three
four-aircraft native trials for this integration.

Build the standalone library without Python, or run sanitizer checks, with:

```bash
cmake -S native -B build/native-check -DSWARM_BUILD_PYTHON=OFF \
  -DSWARM_SANITIZE=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build/native-check --parallel 2
ctest --test-dir build/native-check --output-on-failure
```

The public C++ interface is `native/include/swarmtraj/flight.hpp`. Its numerical
functions expect finite, physically valid inputs and resolved own-aircraft
parameters. The Python boundary validates buffer sizes, values and configuration;
it accepts native float64 C-contiguous buffers without implicit casting. The
Python convenience wrapper performs its conversions explicitly.

## Numerical and ownership contract

States are `[x, y, z, total_airspeed, heading, flight_path_angle, bank]` and
controls are `[thrust_acceleration, brake_acceleration, load_factor, roll_rate]`.
Units and equations match [the point-mass model](flight_model_3d.md). There are
seven doubles per state and four per command. RK4 scratch uses fixed stack
arrays; histories use contiguous vectors with reserved capacity. The binding
copies validated inputs before releasing the GIL and returns an immutable bytes
owner for NumPy views. It makes one output copy; this is not a zero-copy API.
Existing public `Trajectory` copies and compatibility projections remain intact.

Construction crosses the language boundary twice per candidate: through the
pull-out, then through full-horizon recovery/onward flight. Python derives any
obstacle exit route from the actual pull-out state between these calls. There
are no Python callbacks or per-integration-step crossings. The rollout API also
accepts a complete held-command history in one call. Checked nominal rollouts
require zero wind and the reference two-substep integration; unchecked fixtures
can explicitly supply wind and another subdivision count.

Both backends preserve nominal two-substep endpoints, independently reintegrated
four-substep samples at fractions 0, 0.25, 0.5, 0.75 and 1, and the nominal endpoint
envelope check. The Python independent validator still runs for every candidate,
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

## Verification and paired measurements

```bash
./scripts/test.sh tests/test_native_backend.py
./scripts/test.sh
.venv/bin/python scripts/benchmark_native.py \
  --output results/native-paired --repeats 3
```

Replay the full saved mission matrix with the native backend using:

```bash
.venv/bin/python scripts/benchmark_scaling_campaign.py \
  --reference results/scaling-campaign-20261028/manifest.json \
  --suites controls core random replay --generator-mode physical_mission \
  --numerical-backend cpp --output results/native-scaling
.venv/bin/python scripts/report_generator_replay.py results/native-scaling
```

The manifest records the explicit backend override and binary identity; worker
and reporter guards reject changed sources or a different native binary. Reports
keep exact recovery comparisons separate from C++/Python comparisons within the
declared absolute tolerances. The independent physical validator is unchanged.

Native-specific numerical tests skip when the extension is absent. Build it
before using the full suite as native validation. The standalone C++ tests also
cover a manufactured trim, heading wrap, envelope rejection and pull-out.

The final development snapshot passed **925 pytest tests** (660.26 s, four
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

### Complete headless pipeline

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
the geometric/Python defaults and do not establish PX4/Gazebo qualification.

The kernel removes Python small-array allocation churn in the hot loop. It
does not remove interpreter/process memory, combinatorial search complexity or
the bounded proposal catalog's limitations. Execution tracking and the geometric
converter are future ports. No RK4 error certificate, real-aircraft guarantee or
PX4/Gazebo timing/separation claim follows from this implementation.
