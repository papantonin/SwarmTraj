# Default physical C++ planning

New coupled 3D missions use `physical_mission` and the C++ numerical planner.
Python continues to provide the CLI, editor, profile binding, serialization,
visualization and reports. PX4/Gazebo uses its own native controllers and physics.

## Selection policy

| Input | Effective generator | Numerical backend | CLI worker default |
|---|---|---|---|
| New coupled 3D mission | `physical_mission` | `cpp` | 1 |
| Explicit `--numerical-backend python` | `physical_mission` | `python` | Automatic |
| Explicit `--generator-mode geometric` | `geometric` | `python` | Automatic |
| Planar or legacy flight | `geometric` | `python` | Automatic |
| Saved scenario | Recorded/historical choice | Recorded/historical choice | Existing saved/factory setting |

`Config(dimensions=3)` resolves omitted generator/backend fields to concrete
`physical_mission` / `cpp` values in `__post_init__`. `Config()` remains the
planar Python API. Explicit selections are never overwritten; incompatible
requests such as geometric/C++ are rejected. `dataclasses.replace` preserves
the already resolved choices: when changing to an incompatible mode, explicitly
change generator and backend together. No `None` or automatic-selection token
is exported in a mission or v3 scenario. The library defaults to one worker;
explicit `--workers 0` requests automatic spawn workers, including with C++.

The editor starts in **Mission** view. Under **Advanced**, **Physical generator**
and **C++ construction (off: Python)** are checked for fresh coupled 3D scenes.
These are the existing labels;
the C++ selection now covers the complete physical numerical planner. Uncheck
the C++ option before switching to the geometric generator. **Reset** restores
the original effective selections and scene. A loaded Python scenario remains
Python until explicitly changed.

## Build and launch

The existing environment needs a C++20 compiler, CMake 3.20+, Python 3.12+
development headers and the trajectory dependencies. The full `scripts/setup.sh`
builds and checks the native extension after installing the project. In an
existing environment, rebuild without reinstalling dependencies:

```bash
./scripts/build_native.sh
# Fresh editor and point-mass simulation; physical C++ planning is the default.
.venv/bin/python examples/trajectory_3d.py
# Headless equivalent:
.venv/bin/python examples/trajectory_3d.py --no-gui
# Explicit physical Python reference:
.venv/bin/python examples/trajectory_3d.py --numerical-backend python --workers 1
# Historical geometric generator:
.venv/bin/python examples/trajectory_3d.py --generator-mode geometric
```

A missing or outdated native extension fails explicitly before catalog
generation with build/rebuild instructions. It never silently selects Python.
An ordinary pure Python wheel remains supported for explicitly selected Python
and historical modes. Use `--numerical-backend python` to open the trajectory
editor from such an installation: the default C++ startup check otherwise
reports the missing extension before opening that editor.
A native wheel is built with `SWARM_BUILD_NATIVE=1`:

```bash
SWARM_BUILD_NATIVE=1 .venv/bin/python -m pip wheel . --no-deps \
  --no-build-isolation --wheel-dir results/native-wheels
```

The point-mass execution backend remains independently selected by
`--execution-backend python|cpp`, with Python as its default. This selection
does not change the saved plan or the PX4 controller.

## PX4/Gazebo editor

Fresh scenes select physical/C++ without overrides. Use the new v3 fixtures to
retain the illustrated obstacles and initial conditions:

```bash
# One aircraft, fresh editor:
.venv/bin/python examples/px4_gazebo.py --edit-scenario --run --gui --timeout 1200
# Four aircraft, explicit v3 example with obstacles:
.venv/bin/python examples/px4_gazebo.py --num-drones 4 \
  --scenario examples/scenarios/px4_four_loiter_cpp.json \
  --edit-scenario --run --gui --timeout 1200
```

`px4_single_loiter_cpp.json` is the corresponding single-aircraft v3 fixture.
The original `px4_single_loiter.json`, `px4_four_loiter.json` and other saved
files are preserved. V1 still means geometric/Python, v2 retains its generator
and means Python, and v3 requires concrete generator/backend values. Missing or
null v3 selections are rejected. Historical saved campaign decoders likewise
materialize missing mode/backend fields as geometric/Python.

The original standalone PX4 benchmark without `--scenario` or `--edit-scenario`
keeps its historical geometric/Python reference and landing behavior. New
edited/shared scenarios retain their simulation-only remove-at-target default,
with acknowledgement and verified absence. The planner still validates the full
original horizon. See [editor](gazebo_editor.md) and [fleet](gazebo_fleet.md).

## Qualification and limits

The promotion changes selection defaults, not the model, RK4 subdivisions,
validation samples, margins, budgets, deadlines or fixed designation snapshot.
The previous [complete planner qualification](native_planner.md) remains the
numerical reference. Python reference fixtures and historical comparison scripts
now select their original modes explicitly so they continue to test the same
algorithms. The independent Python validator remains available with
`validate_individual(..., backend="python")`.

Default-selection tests cover fresh API/CLI/editor paths, explicit Python and
historical selections, missing native capability, concrete serialization,
strict saved schemas, worker overrides and an actual native plan revalidated
by Python. Generated qualification evidence is under
`results/cpp-default-20261030/`; measurements from earlier checkpoints remain
unchanged. Numerical validation does not establish a certified integration
error bound, continuous PX4 separation or a real-time deadline.

### Default-promotion qualification, 30 October 2026

The frozen implementation passes **1,108 pytest tests in 854.47 s**, with four
existing Gymnasium float32-bound warnings, and all **six standalone Release
C++ tests**. Two initial failures exposed reference fixtures still relying on
implicit geometric/Python selection; those fixtures now state their historical
modes explicitly, preserving their original assertions. The initial interrupted
run remains in the evidence and is not counted as a pass.

The standard headless four-aircraft trajectory CLI, with no generator/backend
or worker override, passes planning and execution. Its canonical mission is
exactly identical to a run with explicit physical/C++/one-worker options.
Native and pure Python wheels import outside the checkout: native default
planning succeeds, missing native capability fails explicitly, and explicit
Python planning succeeds from either package. The setup script passes shell
syntax checks; the existing environment was retained.

A visible TkAgg editor passes initial-state, Python/geometric switching and
Reset checks using Matplotlib mouse events; rendered states are preserved.
The desktop accessibility provider could not enumerate this Tk window, so this
is not a claim of verified native OS mouse input.

Fresh headless PX4/Gazebo runs use the new v3 examples without generator/backend
overrides, with the original pinned image, isolation, gates and thresholds:

| Native trial | Aircraft | Spatial error p95 | Largest target-contact distance | Minimum observed pair distance |
|---|---:|---:|---:|---:|
| Single | 1 | 5.07 m | 5.79 m | Not applicable |
| Shared world | 4 | 4.50–5.46 m | 5.97 m | 155.32 m |

All five aircraft pass actual/estimated initialization and spatial checks,
retaining the 6 m target radius, 25 m spatial p95 threshold and 30 m fleet
observation threshold. Each own-model removal is acknowledged and absence
verified. Pair evaluation covers each pair's active overlapping interval.
Both runs complete and their containers are stopped and removed. The original
point-mass plans retain their full horizons and compiled numerical identity.

After the full suite, one report-only HTML sentence was corrected to stop
claiming that the global default was still geometric; all 20 campaign-tool
tests pass again. The pre-correction reporter is archived. Runtime planner,
execution, native sources and binary remain identical to the full-suite freeze;
old manifests are not rewritten. `verification.json` records this distinction,
source identities, package checks and native outcomes in the artifact directory.
