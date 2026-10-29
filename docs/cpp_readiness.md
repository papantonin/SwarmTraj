# CPU/memory audit and checkpoint before a C++ backend

The first native component should be the complete physical rollout loop:
seven-state dynamics, held-control RK4 and the sampled envelope checks used by
construction. Keep the Python planner, proposal RNG/order, independent validator,
interfaces and reports as the initial reference. PX4 startup reliability is a
separate integration issue and need not block this numerical work.

This audit changes no simulation behavior, parameters, budgets or safety checks.
At the time of this audit, no C++ backend had been implemented or benchmarked.
The subsequent Python stabilization checkpoint is `c930826`; the optional
[native kernel](native_kernel.md) now implements the first boundary below.
The Python baseline uses
the geometric default and an explicitly selected physical builder.

## Current-source measurements

Read-only diagnostics are under
`results/cpp-readiness-audit-20261029-173913/`. The local `profile_current.py`,
`audit-manifest.json`, source hashes, `.prof` files and per-run JSON records retain
the exact commands and inputs. All **11 completed diagnostic runs** reproduce
their previously recorded result signatures. The nominal execution also passes.
The source files match the frozen revision that passed **900 tests**; the full
suite was not rerun for this analysis-only documentation change.

Measurements used the existing Python 3.12 environment on the Intel Core
i3-1115G4 (two physical cores, four logical CPUs), BLAS/OpenMP threads set to one.
Cases come from the saved scaling manifest: `core-baseline-n3-s7`,
`core-rich_strict-n3-s7` and `core-grid-n3-s7`. CPU profiles use one planning
process so child work is not mistaken for unobserved CPU time. Each pass runs in
a fresh interpreter. Traced allocations and process/IPC diagnostics run separately.
Desktop load is uncontrolled; instrumentation durations are not benchmark timings.

| Physical planning profile | Baseline | Strict rich | Grid |
|---|---:|---:|---:|
| Total instrumented planning | 13.32 s | 16.11 s | 110.37 s |
| `physical_generation._step` | 83.75% | 83.38% | 83.03% |
| `flight3d.propagate` (RK4) | 75.58% | 73.14% | 73.74% |
| `flight3d.derivative` | 49.64% | 49.46% | 49.30% |
| Construction/validation `envelope_ok` | 14.18% | 13.81% | 13.89% |
| Independent `validate_individual` | 2.06% | 2.03% | 1.83% |
| Joint `solve` | 0.01% | 0.02% | 3.20% |

These are **nested cumulative times**, not additive categories. `_step` includes
integration and envelope work. The independent validator additionally calls the
same numerical helpers; its percentage describes that final independent pass,
not all checking performed while constructing candidates.

The grid case completes as `catalog_exhausted`, with 162 attempts and 82
individually accepted candidates, but no selected joint plan. It makes 79,868
construction steps and 179,473 RK4 calls. Its 1,368,337 vector RHS calls and
904,424 scalar RHS calls each construct a new result array: over 2.27 million
RHS output arrays by source/call-count accounting, before RK4 intermediate arrays.
This is allocation churn, not evidence of a leak or millions of retained arrays.

Unprofiled diagnostic planning times were 4.66 s for geometric baseline,
10.75 s for physical baseline, 13.31 s for physical rich and 87.93 s for physical
grid. Do not compare these fresh desktop observations to historical timings as
an isolated speedup experiment. No planning cap or mission constraint was changed;
the diagnostic profiler has its own external guard and is not a qualification run.

The geometric baseline has a different distribution: `convert_candidate` takes
58.34% cumulatively, physical RK4 31.53%, guidance 24.02% and planar propagation
23.58%, with overlapping call trees. Porting only `coupled_planning.py` would
miss the current `physical_mission` construction path.

The separate nominal three-aircraft execution profile takes 4.54 s under
cProfile. RK4 accounts for 64.02% cumulatively; `tracking.held_control_ok` for
32.56%, including RK4. The same native numerical core could therefore benefit
execution campaigns later, without rewriting their observer/controller isolation.

## Memory findings

| Observation | Measurement | Interpretation |
|---|---:|---|
| Serial physical baseline process peak RSS | 80.88 MiB | Includes interpreter, libraries and allocator state |
| Baseline traced allocation peak during planning | 2.75 MiB | Separate, slower traced run; includes lazy imports |
| Ten retained baseline candidates: array buffers | 0.65 MiB | Excludes object headers, caches and temporary arrays |
| Selected three baseline plans: array buffers | 0.20 MiB | Retained public arrays only |
| 82 grid candidates: array buffers | 8.92 MiB | Recorded while catalogs are alive, despite no selection |
| Serial grid process peak RSS | 97.70 MiB | Separate unprofiled run |
| Two-worker baseline process tree: peak sampled RSS sum | 244.14 MiB | Shared pages are counted repeatedly |
| Same process tree: peak sampled PSS sum | 173.22 MiB | Sampled proportional attribution, not a memory guarantee |

The two-worker diagnostic observes up to four live processes including helpers.
Its protocol-5 serialization estimates are 0.28 MiB of task payloads and
0.93 MiB of returned catalogs over two generation batches. These are diagnostic
serializations, not actual pipe-traffic measurements, and their instrumentation
must not be treated as ordinary planning performance.

Specific memory costs in the source:

- `physical_generation.py` builds lists of separate state/control arrays, then
  materializes dense arrays, projections and immutable `Trajectory` copies.
  Preallocated native state/control buffers can remove small-object and temporary
  allocations. Ownership isolation must remain intact.
- `Trajectory.__post_init__` intentionally copies arrays and makes them read-only.
  For a constant-speed coupled plan with N steps, its stored array payload is
  `8 * (19*N + 13)` bytes. The canonical seven-state/four-control history alone
  is `8 * (11*N + 7)` bytes. This identifies duplicated compatibility projections,
  not permission to remove public fields or weaken immutability.
- `Trajectory.positions` rebuilds an XYZ array with `column_stack` on each 3D
  access. The grid profile calls it 8,344 times, but it represents only 0.12%
  of current CPU time. A native kernel can read XYZ through the canonical state
  layout; a Python cache would trade retained memory for fewer temporary copies.
- `parallel.generate_catalogs` opens a new spawn pool on each multi-task call.
  The grid scene uses 27 batches/42 tasks. Its deterministic batch structure
  would create 14 pools with two requested workers; single-task batches remain
  serial. Existing catalogs are sent and returned with each relevant task.
  From the recorded counts, numerical array payload alone would total about
  39.61 MiB sent and 45.70 MiB returned over those multi-task batches, excluding
  framing/metadata. These are derived estimates for the two-worker structure,
  not measured traffic from the serial CPU profile.

Compiling RK4 alone will not eliminate Python worker imports or process memory.
A compact standalone native engine could address that later. First measure a
sequential extension under the existing spawn architecture; do not combine
processes and a new native thread pool without separate oversubscription and
determinism tests.

## Recommended native boundary

1. Implement a standalone C++ numerical library using double precision, a fixed
   seven-value state and four-value control, reusable RK4 scratch storage and
   preallocated trajectory buffers. Small fixed arrays suffice initially; Eigen
   is not required for these scalar equations.
2. First expose a complete held-command rollout for reference physics fixtures.
   Then expose construction of one complete candidate, including local guidance,
   phase progression and construction envelope checks. Crossing Python/C++ at
   every derivative or step would leave substantial dispatch/allocation work.
3. Keep proposal sampling/RNG and catalog/search order in Python initially.
   Native parameters must be the complete resolved own-aircraft values, with
   explicit recipes and diagnostics. Replacing NumPy RNG with a different C++
   generator while porting would change the experiment.
4. Use explicit NumPy buffer shapes, dtype, strides and ownership contracts.
   Avoid implicit conversions/copies and preserve read-only outputs after
   process transfer. pybind11 supports buffer exchange, but zero-copy behavior
   requires a deliberate interface and correct lifetime management.
   See [official NumPy/buffer documentation](https://pybind11.readthedocs.io/en/stable/advanced/pycpp/numpy.html).
5. Release the GIL only during autonomous native computation, after acquiring
   and validating inputs, with no Python-object access or callbacks in that
   region. pybind11 does not release it automatically.
   See [official GIL documentation](https://pybind11.readthedocs.io/en/stable/advanced/misc.html#global-interpreter-lock-gil).
6. Retain Python independent validation and its reference numerical path for
   every native candidate initially. Preserve both switch sides, refined and
   nominal endpoints, all held-command samples and full-horizon checks. Freeze
   floating-point/heading-wrap semantics and tolerances before comparison;
   disable fast-math and initially avoid contraction/reassociation changes.
7. Benchmark isolated rollout, complete candidates, catalogs and whole planning.
   Require positive and negative acceptance agreement and deterministic repeated
   native runs. An isolated kernel gain does not establish a whole-planner gain.

Priorities: P0 physical numerical rollout/construction; P1 storage and catalog
transport after measurement; P2 geometric converter and execution hot loops as
their workloads justify; retain central search, scenario handling, visualization,
CLI, reports and PX4 orchestration in Python initially. The final independent
validator is a low-cost reference in the current profiles, not the first port.

## What is ready to commit before C++

The validated Python implementation is the Git checkpoint before changing build
systems or numerical backends. The audit started from HEAD `abad7a6`; at that
point the physical builder, recovery primitive, regression tests and report tools
included untracked files that had to accompany their callers. The stabilization
checkpoint includes the complete implementation, tests and documentation in one
commit. The groups below describe review boundaries, not separate committed
snapshots.

Suggested dependent review units:

| Order | Suggested commit | Scope / readiness |
|---|---|---|
| 1 | `fix(execution): validate actual asynchronous control boundaries` | `execution.py`, `test_execution_boundaries.py`, corresponding tracking documentation. Independently applicable to HEAD; four focused tests passed in an isolated snapshot. |
| 2 | `feat(trajectory): add opt-in complete physical mission construction` | Physical generation, recovery, Config/Trajectory dispatch and metadata, trajectory CLI, numerical allocation optimizations, associated physics/regression tests and relevant guides. Ready as the tested opt-in Python reference. |
| 3 | `feat(editor): persist generator choice in versioned scenarios` | Editor checkbox, strict single/fleet v2 records and v1 loading, PX4 CLI overrides, migration tests and guides. Automated tests pass; native UI clicks remain unverified and fleet startup remains variable. |
| 4 | `test(planning): preserve paired generator qualification evidence` | Campaign replay/process options, comparison/revalidation tools, benchmark scripts/tests, documentation/architecture and this readiness audit. Preserve failed trials and the geometric default. |

These are review groups with dependencies. Intermediate feature groups have not
each been rerun as standalone full suites; the 900-test evidence applies to their
combined frozen implementation, which is preserved by the single checkpoint.

The first fix is also available locally as `01-execution-boundaries.patch`, with
`commit01-validation.json` and `commit01-tests.log`. It was tested against an
unpacked HEAD outside the working tree, importing that snapshot's own source.
The user's Git index was not changed by the analysis. The subsequent Python
checkpoint was explicitly requested separately; no C++ implementation is included.

Keep generated profiling/telemetry/results, binaries, `.venv` and caches out of
Git. This audit alone establishes neither default promotion, a complete 230-case
refresh, reliable PX4 startup, verified native editor interaction nor a C++
speedup. See [current migration results and gates](generator_migration.md) and
the subsequent [native implementation and measurements](native_kernel.md).
