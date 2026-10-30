# Agent guide

## Scope and project purpose

These instructions apply throughout this repository unless a more specific
`AGENTS.md` overrides them.

This is a Python drone-swarm simulation and trajectory-planning project.
The main development path is coordinated fixed-wing trajectory generation:
take a shared initial snapshot, generate reproducible candidate trajectories,
select a compatible combination centrally, and execute each plan using only
that aircraft's own state and clock. No ground-station or inter-drone telemetry
is available to controllers during execution.

Keep documentation in English. The existing GUI contains French labels; when
documenting them, retain the actual label and provide its English meaning.
Do not translate the interface as an incidental documentation change.

## Repository map

- `src/swarm_sim/trajectory2d/`: the native 3D production trajectory system and archive data types. The package
  name is historical; do not rename it just because the default launcher is 3D.
  - `planning.py`: configuration, candidate catalogs, geometric approach-sector
    assignment, bounded joint search and plan serialization.
  - `dynamics.py`: planar primitives, initial snapshots and limits.
  - `vertical.py`: independent analytical descent checks and archive profile math.
  - `flight3d.py`: independent Python equations/envelope for audits and observation;
    production dynamics and RK4 live in `native/src/flight.cpp`.
  - `aircraft_profiles.py`: strict versioned fixed-wing JSON, immutable parameter
    binding and canonical identity. Common or complete per-ID assignments;
    v2 enclosing spheres, no physical overrides or ignored fields.
  - `coupled_planning.py`: independent Python full-horizon audit replay, not a
    proposal generator. Keep its separately implemented physical checks.
  - `execution.py`: local tracking, disturbances and global evaluation.
  - `tracking.py`: independent held-control audit. Native execution owns wind
    estimation and protection; failed fallback verification invalidates acceptance.
  - `perturbations.py`: simulator-only wind/position sensors. Never feed true
    disturbance or observer state to a controller.
  - `execution_safety.py`: observer-only envelope events and shared acceptance
    criteria; preserve both sides of asynchronous actuator switches.
  - `parallel.py`: deterministic, spawn-based catalog generation.
  - `catalog.py`: resumable per-drone catalogs, rejection stages and timings.
  - `search.py`: compatibility import for the sole native joint-search adapter.
  - `conflict_guidance.py`: offline sampled pair witnesses and bounded proposal
    hints; never treat a repair or individually accepted proposal as jointly safe.
  - `obstacles.py`: static solids and independent full-horizon clearance bounds;
    routing proposals are native. `obstacle_view.py` renders solids.
  - `mission_timing.py`: arrival/recovery schedules and advisory-only horizon
    suggestions. Explicit arrival deadlines must never be silently extended.
  - `contact.py`: shared reintegrated first-entry events; unresolved bounded
    searches must fail validation, never widen the passage-angle tolerance.
  - `preparation.py`: designation-clock loading/acknowledgement timeline and
    simulated all-or-none activation gate; not a live distributed protocol.
  - `reservations.py`: conservative cell/time resource allocation; preserve the
    independent distance check and full-horizon reservation recheck.
  - `interval_guidance.py`: immutable native departure-window forecast records;
    references are not fleet commitments or safety proofs.
  - `profile_policy.py`: explicit mixed-family requirements; strict user requests
    must never fall back to simpler profiles, and mixed plans require a valid declaration.
  - `native_planning.py`, `native_backend.py`: typed adapters to the required C++
    planner/control engine; no Python numerical fallback.
  - `production.py`: native-only production gate. Archived recorded modes remain
    readable; unsupported requests fail without automatic conversion.
  - `scenario_editor.py`: preparation interface without backend/generator switches.
  - `visualization.py`, `visualization3d.py`: playback and plots.
- `examples/trajectory_3d.py`: preferred trajectory entry point; delegates to
  `examples/trajectory_2d.py`, now a compatibility alias for the same 3D path.
- `src/swarm_sim/px4_scenario.py`: strict shared single-aircraft editor schema,
  loiter metadata, proposal-only landing-pattern screening and experiment gates.
  See `docs/gazebo_editor.md`; preserve the distinction between geometric
  screening and actual swept-body/temporal safety validation.
- `src/swarm_sim/px4_fleet.py`: opt-in four-aircraft shared-world experiment,
  strict fleet schema, joint planning, immutable own-plan projections, startup
  barriers and observer-only pair measurements. See `docs/gazebo_fleet.md`.
  Preserve independent PX4 instances, IDs, localhost ports and parameter/log
  directories. Keep single-aircraft mode unchanged. Never transfer the
  point-mass timing/separation certificate to spatial PX4 execution or call
  independent landing-pattern screens a joint return safety proof.
- `src/swarm_sim/px4_camera.py`: GUI-only free overview and keyboard camera
  helper, launched with the container's system Python for native Gazebo bindings.
  Preserve the firewall between camera services and aircraft control: no MAVSDK,
  world pose changes or actuator commands. Keep mouse control and user-selected
  views; do not force a follow target every frame. Camera navigation is not a
  change to reference geometry, sensor range or flight validation.
- `src/swarm_sim/px4_termination.py`: fingerprinted, simulation-only completion
  policy, derived native reference prefix, own observed-target distance and
  exact-own-model Gazebo removal with acknowledgement AND verified absence.
  Never count a missing/invalid scene reply as successful disappearance. Keep
  historical landing mode and the planner's original full-horizon validation.
- `src/swarm_sim/px4_reference.py`, `px4_gazebo.py` and
  `examples/px4_gazebo.py`: optional isolated, single-aircraft PX4/Gazebo Cessna
  spatial experiment. No remote/real-hardware interface, model certificate
  transfer, timed follower, live snapshot initialization or swarm flight.
  Preserve pinned image/SDK identity, network isolation and original PX4
  readiness/mission/landing checks. The original plan is immutable; derived
  waypoints explicitly lose its timing and safety claims. Do not increase the
  spatial-error experiment threshold to hide a failed flight.
  Explicit `--gpu` may expose only `/dev/dri/renderD128` for rendering, without
  privileged mode or serial/flight-controller device access. Preserve the
  default software fallback and record requested GPU mode separately from
  observed renderer/performance. Never claim hardware use from a flag alone.
  `examples/gazebo/` inherits the image's official airframe with a downward
  lidar and extends the flat rendered ground for terrain-dependent landing;
  retain both sensor/ground geometry and disclose these fixture adaptations.
  `px4_scene.py` derives a per-run SDF from the immutable mission. Optional JSON
  obstacles participate in reference planning and have matching Gazebo collisions;
  PX4 waypoint rounding and separate takeoff/landing are not obstacle-certified.
  Gold target annotations are non-colliding visibility-bit-1 visuals, excluded
  by the fixture lidar mask; never let them become fictitious landing terrain.
  The opt-in `--airborne-start` bench is a warm initialization, not a cold in-air
  boot. Shared edited scenarios support only explicitly bounded initial states,
  not arbitrary airframe trim/checkpoint initialization. Preserve ground readiness/arm,
  mission feasibility, single pose change + single mass-proportional impulse,
  actual/estimated-state gates and documented one-time native controller reset.
  The bootstrap is observer-only afterwards; never add sustained motion commands
  or silently claim the immediate-state gate proves stable/full-flight tracking.
- `src/swarm_sim/simulator.py`: separate quadrotor backend using
  gym-pybullet-drones/PyBullet.
- `src/swarm_sim/fixed_wing.py`: separate, simpler fixed-wing ring simulator;
  not the coupled trajectory-planning engine.
- `native/src/`: the sole production proposals, catalogs, joint search, independent
  final checks, dynamics and local execution. Preserve initial loiter, terminal
  requirements and actual profile attainment; wider routing/longer settling
  proposals must never change physical limits or validation margins.
- `src/swarm_sim/scenario_migration.py`: explicit new-file shared-scenario migration
  with source hash and changed-field provenance; never transfer old validation.
- `tests/`: pytest tests for dynamics, planning, execution, GUI, CLI and backends.
- `scripts/benchmark_planning.py`: reference-versus-optimized planning benchmark.
- `scripts/benchmark_adaptive.py`: four-aircraft fixed/adaptive comparisons and
  optional disturbed execution checks.
- `scripts/benchmark_selection.py`: four-aircraft first-feasible versus
  margin-preferring selection using a saved scene.
- `scripts/benchmark_tracking.py`: four-aircraft disturbance sweeps on identical
  plans, historical signatures and deliberate out-of-bounds rejection trials.
- `scripts/benchmark_conflicts.py`: paired unguided/witness-guided four-aircraft
  planning and optional execution checks, preserving identical constraints.
- `scripts/benchmark_intervals_profiles.py`: baseline/windows/rich/mixed/passage
  four-aircraft comparisons, including explicitly configured fixed departures;
  keep failed trials and distinguish individual acceptance from final selection.
- `scripts/validate_scenarios.py`: incremental four-aircraft parameter/disturbance
  campaigns with source guards; retain failed expectations and propagate errors.
- `scripts/audit_campaigns.py`: standalone, read-only journal consistency and
  completeness audit. Never infer missing selections from observed trials or
  describe journal consistency as independent trajectory revalidation.
- `scripts/validate_vertical_flight.py`: isolated manufactured-solution physics
  fixtures and an independently written unbanked ODE comparison. No planning,
  guidance calls or executable plan/control export; keep held-command refinement
  distinct from integration-subdivision checks and retain negative checks.
- `scripts/validate_flight_envelope.py`: boundary classification, analytical
  steady-equilibrium checks and broader isolated speed/time-step experiments.
  Loaded profiles select parameter-relative boundary/grid fixtures; preserve
  the historical reference fixtures when no profile is supplied.
  Keep coupled boundary exclusions, classification agreement, numerical precision
  and failed dynamic requests distinct; never relabel failed requests as negatives.
- `docs/`: trajectory guides, flight equations, assumptions and performance data.
- `examples/aircraft/`: complete illustrative reference/training and approximate
  public Rascal case-study profiles, not calibrated aircraft or mesh/6-DoF data.
- `results/`: generated run artifacts; ignored by Git.

Do not describe the current trajectory engine as Gazebo, PX4, ROS 2 or 6-DoF.
The optional single-aircraft adapter uses real PX4/Gazebo separately; it does
not change the planner's physics or establish multi-aircraft execution safety.
It uses custom dynamics, NumPy/SciPy and Matplotlib. PyBullet is used by the
separate quadrotor and ring examples.

## Environment and commands

Use Python 3.12 or newer and the project-local `.venv`. Prefer the existing
environment; do not recreate it or reinstall dependencies unnecessarily.

```bash
# Full simulation setup on a fresh installation
bash scripts/setup.sh

# Trajectory development dependencies without the separate PyBullet backend
.venv/bin/python -m pip install -e '.[dev,trajectory2d]'

# Full test suite; isolates pytest from externally sourced ROS plugins
./scripts/test.sh

# Focused checks
./scripts/test.sh tests/test_flight3d.py tests/test_cpp_production.py tests/test_native_mission.py
./scripts/test.sh tests/test_parallel_planning.py tests/test_trajectory_cli.py

# Default coupled 3D scenario, without GUI
.venv/bin/python examples/trajectory_3d.py --no-gui

# Native parallel catalogs (all numerical work remains C++)
.venv/bin/python examples/trajectory_3d.py --workers 2 --no-gui

# Ten-aircraft scenario, initial altitudes 30–120 m
.venv/bin/python examples/trajectory_3d.py --no-gui --num-drones 10 \
  --altitude-spacing 10 --approach-spread 180 --horizon 150

# Performance comparison; can take several minutes
.venv/bin/python scripts/benchmark_planning.py
```

The setup script pins the upstream gym-pybullet-drones commit. Preserve that
pin unless intentionally updating and testing the dependency.

## Modeling and safety invariants

- Use SI units. World Z points upward; heading 0 points toward +X, π/2 toward
  +Y. Flight-path angle is not pitch attitude.
- In coupled 3D mode, speed means **total airspeed**, not horizontal speed.
  Wind contributes to ground velocity. Keep legacy speed semantics distinct.
- Treat the initial snapshot as fixed planning input. Individual exported
  plans and controllers must not depend on another aircraft's state or plan.
  Full-state observers are for evaluation and display only.
- Validate integrated trajectories before joint separation selection. Changing
  geometry, timing or flight dynamics after a separation check invalidates it.
- Retain separation margins for tracking error and clock uncertainty, and
  bounds covering motion between samples. Spatial error in 3D is a Euclidean
  norm, not an independent allowance on each axis.
- Check the entire common horizon, including loiter, altitude transitions,
  target passage, pull-out and onward flight. Aircraft do not disappear or stop
  when they reach the target. A target passage is not a landing procedure.
  Exception in the explicitly requested Gazebo-only `remove-at-target` mode:
  delete each own simulated model on observed target contact, with acknowledgement
  and verified scene absence, instead of returning/landing. The ORIGINAL planner
  still validates its entire horizon unchanged; only the derived native/display
  prefix and runtime completion change. No real-aircraft termination/disarm.
  Preserve `land` mode. Only this fresh simulated instance may explicitly use
  `MIS_TKO_LAND_REQ=1` (require takeoff, not landing), recorded/read back; retain
  other parameters, native feasibility/readiness/arm gates and failure propagation.
  Pair evaluation after deletion uses each pair's active overlapping interval,
  never extrapolation or truncation at the first aircraft's arrival.
- Keep total horizon, optional absolute arrival deadline and minimum
  post-arrival time separate. Full recovery must fit even if it takes longer
  than the configured post-arrival minimum; see `docs/mission_timing.md`.
- Never silently relax safety margins, envelope limits, arrival requirements,
  approach spread or departure-delay bounds to obtain a successful result.
- Distinguish invalid input or a necessary infeasibility condition from failure
  of a finite candidate catalog or bounded search. `no_candidate`,
  `catalog_exhausted` and `search_budget_exhausted` do not prove infeasibility
  in continuous space.
- Numerical validation is conditional on the model, speed bounds, tracking
  tubes and clock assumptions. RK4 has no certified integration-error bound
  here. Do not claim formal collision freedom or real-aircraft certification.
- Physical defaults are illustrative, including the optional airbrake. Do not
  imply aircraft calibration, battery endurance or fault tolerance that the
  model does not provide.
- Loaded aircraft profiles apply to coupled 3D only and are authoritative for
  physical fields; mission/environment settings stay separate. Verify profile
  identity against effective physical values before export and hash the full
  record with the mission. Per-ID assignments must cover exactly the mission IDs;
  resolve own physics with `Config.for_drone` throughout planning and execution.
  Never pass the fleet map into a controller or individual payload. Include v2
  sphere radii in pair, obstacle and reservation checks. V1 radius zero preserves
  legacy point-aircraft semantics, not a real-aircraft geometry guarantee. No hot reload.
- Planning uses one fixed designation snapshot. Optional in-flight preparation
  includes loiter motion on the same mission clock and gates activation on
  loading/acknowledgement times. This is chronological offline replay, not a live
  network or distributed-commit implementation. Activation delay is never a
  real-time computation or transmission guarantee.

Read `docs/flight_model_3d.md` before changing coupled dynamics or safety checks.
Read the relevant trajectory guide before changing planner or execution behavior.

## Determinism and performance

Preserve per-drone seeded random streams, deterministic candidate order and
joint-search order. Different seeds alone do not ensure collision avoidance.
Use processes, not an unverified thread conversion, for existing CPU-bound
catalog parallelism. Keep worker functions spawn-safe and entry points guarded
by `if __name__ == "__main__":`. Preserve read-only plan arrays after process
transfer and propagate worker failures rather than validating partial output.

C++ is the only production planning/execution backend; `Config()` defaults to
coupled 3D physical C++. Python retains independent verification and orchestration.
Historical planar/geometric/legacy inputs retain their recorded metadata for
inspection, but cannot launch production. Explicit migration writes a new record.
The library and CLI default to one worker. See `docs/planner_defaults.md`.
Performance-only changes must retain validation points, budgets and margins.
Use the benchmark's result signatures to compare identical scenarios, and
report actual hardware, settings and measured timings rather than guaranteed
speedups. Keep wall-clock timings out of the reproducible mission manifest.

## Change and verification workflow

Inspect the working tree before editing; preserve unrelated and untracked user
work. Keep changes scoped to the request. Do not commit generated telemetry,
plots, benchmark outputs, virtual environments or caches.

Add focused regression tests for behavior changes. Run relevant tests first;
for changes to dynamics, safety, serialization or parallel planning, also run
the full suite when practical. Use headless scenario runs for end-to-end checks.
GUI changes additionally need an interactive smoke test when a display is
available; report if it was not possible.

Keep CLI help, exported schemas, examples and documentation consistent with
behavior changes. Preserve historical data meanings and public imports unless a deliberate
migration is part of the request. Retired planar/geometric/Python production
requests must fail clearly; do not restore them as fallback. Never mask a failing regression
by weakening safety assertions.

Report what changed, what was actually tested, observed outcomes and remaining
limitations. Documentation-only changes need link/code-block checks, not an
unnecessary long simulation. Do not present unrun tests as passing.
