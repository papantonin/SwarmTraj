# C++ production backend

C++20 is the only production backend for coordinated trajectory planning and
local point-mass flight execution. Python provides the editor, configuration,
immutable plan objects, deterministic process scheduling, plots and reports.
Independent Python physics replay and the global observer remain separate
checks; they cannot generate a replacement mission or control an aircraft.

| Component | Implementation |
|---|---|
| Physical proposals, catalogs and bounded joint selection | C++ |
| Candidate validation and independent final joint rechecks in the planner | C++ |
| Local control, wind estimation, protection and simulated motion | C++ |
| Independent audit equations, target events and execution observation | Python |
| Editor, scenario files, visualization and reports | Python |
| Optional PX4/Gazebo | Its own native controllers and physics |

## Install and run

Use Python 3.12+, a C++20 compiler, Python development headers and CMake 3.20+.
Every normal installation and wheel build compiles the required extension.
The setup script also runs standalone C++ checks and the environment diagnostic:

```bash
bash scripts/setup.sh --trajectory
.venv/bin/python examples/trajectory_3d.py
.venv/bin/python examples/trajectory_3d.py --no-gui
```

Rebuild an existing environment without reinstalling dependencies:

```bash
./scripts/build_native.sh
.venv/bin/python scripts/doctor.py
```

No backend switch is needed. The editor has no generator/backend checkbox.
Explicit `--numerical-backend cpp`, `--execution-backend cpp` and
`--generator-mode physical_mission` remain hidden compatibility arguments.
Python, geometric, planar/legacy flight and legacy-controller requests fail
before planning or simulation output. Missing/incompatible native extensions
fail with build instructions; pure Python production builds and fallback are
retired.

`plan_mission(snapshots)` uses coupled 3D physical C++ planning. Use
`Config()` (or explicitly `Config(dimensions=3)`) for production configuration.
`Config` can still represent explicitly recorded historical modes for inspection
and independent validation; that does not enable a retired production backend.

One worker calls the complete native planner once. Explicit `--workers 2` or
`--workers 0` retains spawn-based scheduling: proposals, catalogs, selection and
final checks remain native. Failures propagate and transferred plans stay
immutable.

## Archived scenarios and explicit migration

Shared v1/v2/v3 editor files retain their recorded meaning: v1 geometric/Python,
v2 its generator with Python, v3 both explicit fields. Loading never converts
them. The editor displays incompatible archived settings with launch disabled.
Direct planning/PX4 preparation rejects them before output creation or Docker.

Create a new production request explicitly:

```bash
mkdir -p results
.venv/bin/python scripts/migrate_scenario.py \
  examples/scenarios/px4_four_loiter.json \
  --output results/my_fleet_cpp.json
```

The output parent must exist. Neither the destination nor its `.migration.json`
sidecar may exist. Snapshots and constraints are preserved; the sidecar records
the source hash/schema and changed fields. Geometric-to-physical migration
changes generator semantics. No trajectory, planning success, timing or
separation validation transfers; the new request must be planned and checked.
Historical implementations remain in Git at revision `300de37`.

Existing explicit native examples need no migration:

```bash
.venv/bin/python examples/px4_gazebo.py --num-drones 4 \
  --scenario examples/scenarios/px4_four_loiter_cpp.json \
  --edit-scenario --run --gui --timeout 1200
```

`px4_single_loiter_cpp.json` is the single-aircraft example. Archived examples
and results remain unchanged.

## Validation scope

Physical limits, budgets, clock/tracking margins, full-horizon checks, obstacle
clearance, contact tolerance and failed-fallback rejection remain unchanged.
Python `validate_individual` defaults to independent audit replay; its
implementation selector is a verification tool, not a production backend.
Observers never feed peer state or true disturbance to controllers.

This remains a conditional numerical point-mass model without a certified RK4
error bound. PX4/Gazebo does not inherit timing/separation claims. See the
[flight equations](flight_model_3d.md) and
[native implementation and historical benchmarks](native_kernel.md).

## Retirement verification (2026-10-30)

The integrated suite passed 1,009 tests. Six standalone C++ tests passed in both
Release and AddressSanitizer/UndefinedBehaviorSanitizer builds. Additional CLI
and search checks passed after the final help/documentation cleanup.

Five saved physical/C++ planning cases (single aircraft, rich profiles, fleet,
reservations and PX4 fleet references) matched the pre-removal states and controls
within absolute `1e-9`. Selected plans passed the independent Python validator;
the four point-mass execution cases also matched their previous reports and
arrays within that tolerance. A separate four-aircraft serial/spawn trial gave
identical signatures and zero array difference. Nominal and bounded gust/sensor
execution passed; deliberately excessive clock error remained rejected.

An actual Tk editor smoke test prepared a native fleet and blocked archived
launches. The built wheel was extracted and exercised outside the checkout:
native planning/execution passed, and the five deleted proposal/generator
modules were absent. Explicit archive migration and subsequent fleet preparation
also passed without modifying the source scenario.

The unedited single-aircraft PX4/Gazebo bench completed takeoff, reference flight
and landing using its new physical/C++ reference. Spatial error was 1.85 m at
the 95th percentile and 35.76 m maximum; the existing acceptance criterion is
the 25 m percentile threshold. This is a single-aircraft spatial check, not a
fleet timing/separation qualification. No new real four-aircraft Gazebo flight
was performed for this retirement change.

Local manifests, comparisons and logs are under
`results/cpp-only-production-20261030/` (ignored by Git). These checks qualify
the backend removal on the tested cases; they do not establish a new speedup
or memory-reduction factor.
