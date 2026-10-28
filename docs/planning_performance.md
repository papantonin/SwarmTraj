# Planning time and parallel execution

This page describes the fixed-catalog optimization and its historical benchmark.
The CLI now defaults to adaptive catalogs and MRV search; see
[adaptive planning and diagnostics](adaptive_planning.md). Use
`--planning-strategy fixed --search-order id --selection-policy first` to reproduce full-catalog CLI
behavior. `scripts/benchmark_planning.py` still uses fixed catalogs through the
library defaults, retaining its original comparison semantics.

Catalog generation is independent for each drone. The launcher uses **separate
processes** for this stage, followed by a joint search that remains deterministic
and sequential. No in-flight communication between drones is introduced.
These are not Python threads:
[ProcessPoolExecutor](https://docs.python.org/3.12/library/concurrent.futures.html#processpoolexecutor)
runs CPU-bound loops on multiple processors without depending on the GIL.

## Settings

```bash
# Automatic: min(number of drones, available logical CPUs)
.venv/bin/python examples/trajectory_3d.py --approach-spread 180 --horizon 150

# Choose 2 or 4 processes, depending on the machine
.venv/bin/python examples/trajectory_3d.py --approach-spread 180 --horizon 150 --workers 2
.venv/bin/python examples/trajectory_3d.py --approach-spread 180 --horizon 150 --workers 4

# Same numerical optimization, but sequential execution
.venv/bin/python examples/trajectory_3d.py --workers 1

# Pre-optimization reference path: sequential, checking every step
.venv/bin/python examples/trajectory_3d.py --workers 1 --rollout-check-interval 1
```

The **launcher** defaults to `--workers 0` (automatic). The **Python API** keeps
`Config(planning_workers=1)` to avoid implicitly creating processes in a notebook
or host application. Setting `planning_workers=0`, `2` or `4` in a script with
an `if __name__ == "__main__":` guard enables parallel execution.
The requested count is capped by available CPUs and the number of drones.
An interactive session that cannot be imported falls back to sequential
execution with a warning.

Processes start using **spawn**, avoiding inherited Tk, Matplotlib or numerical
library locks from the preparation window. Worker errors are propagated: a
partial mission is never presented as validated. Processes are closed at the
end of each planning call.

## What is accelerated, and what stays unchanged

Each drone uses exactly its own PCG64 sequence derived from the seed and its
identifier. Results are collected in drone order, regardless of task completion
order. Backtracking is unchanged. Plan arrays remain read-only, including
after transfer between processes.

Flight-envelope checks during point-mass generation are batched in blocks of
**16 primitives** (`rollout_check_interval=16`) to amortize small NumPy calls.
**Both endpoints of every primitive are still checked with that primitive's
own control**, including both sides of a control change and the final partial
block. Nothing is subsampled. Immediate guards prevent integration from
continuing with an out-of-domain speed or flight-path angle. An invalid proposal
may be rejected later within the block, but cannot be accepted without passing
all checks.

Independent validation of the trajectories admitted to each catalog is unchanged:
dynamics reconstruction, refined internal checks, target passage, angle, exit
and return to cruise altitude. Separation checks and margins are neither removed
nor reduced. Integration step, horizon, candidate count and attempt budget are
unchanged.

The comparison uses a SHA-256 fingerprint of selected plans, status, catalog
sizes, search nodes and distance bounds. Changing performance parameters may
change the complete manifest's fingerprint (its configuration differs), but not
the expected trajectories within the same software environment.

## Reproducing the measurements

```bash
.venv/bin/python scripts/benchmark_planning.py

# Multiple repetitions; variant order alternates between repetitions
.venv/bin/python scripts/benchmark_planning.py --repeats 3
```

The three scenarios match the initial measurement: 4 aircraft, 12 m/s airspeed,
default initial positions and altitudes (30, 55, 80, 105 m), seed 7, target
(140,50,30), automatic cruise altitude within 30–120 m, a 30° descent, and
departures configured at 3 s plus a delay in [0,22] s. Only spread and horizon
change:

- No assigned approach sector, 90 s horizon.
- 180° spread, 150 s horizon.
- 360° spread, 240 s horizon.

The script compares the reference, batched sequential execution, then 2 and 4
processes. Timing covers **only `plan_mission`**, including creation, transfers
and shutdown of a fresh pool. It excludes visualization, post-planning flight
simulation, hashing and export. Results are exported to a new JSON file under
`results/benchmarks/`.

On a machine with 2 physical cores / 4 logical processors, 4 processes do not
promise a ×4 speedup. Load balancing depends on the number of rejected proposals
per drone; the longest task and process startup limit the gain. Small missions
may run faster sequentially. There is still no strict wall-clock time bound:
budgets limit attempts, candidates and nodes, not a deadline in seconds.
Numerical-validation and physical-model limitations remain those described in
the [flight model guide](flight_model_3d.md).

## Measurements on this machine

Intel i3-1115G4 CPU, **2 physical cores / 4 logical processors**. One run per
variant/scenario, with no other heavy tests running concurrently. Times include
startup of a fresh pool. The reference was remeasured on the unmodified code,
then the same scenarios were compared with the optimizations:

| Scenario | Reference | Batching, 1 process | Batching, 2 processes | Batching, 4 processes | Total speedup with 4 |
|---|---:|---:|---:|---:|---:|
| No sector, 90 s horizon | 31.94 s | 21.97 s | 13.81 s | 12.81 s | ×2.49 |
| 180°, 150 s horizon | 60.49 s | 39.20 s | 23.18 s | 21.32 s | ×2.84 |
| 360°, 240 s horizon | 82.62 s | 55.49 s | 36.43 s | 31.34 s | ×2.64 |

Every variant found a validated combination with exactly the same result
fingerprint as the reference. Candidate counts remain `(16,28,28,3)`,
`(23,28,28,22)` and `(24,28,28,17)`, respectively, with 4 search nodes in each
case. Total gains combine batching and parallel execution; the 1-process column
isolates the effect of batching.

These are indicative measurements, not a statistical distribution or a guaranteed
deadline. Reference times differ from the first measurements mentioned in the
conversation (26/50/72 s) because they were remeasured; they vary with CPU load,
frequency and thermal state. Detailed data from this run is stored in
`results/benchmarks/reference-before-parallel.json` and
`results/benchmarks/optimized-parallel-comparison.json`.

The launcher displays generation time and adds it to the report as
`planning_wall_time_s`, along with the parallel-execution settings. This time is
not included in the reproducible mission manifest.
