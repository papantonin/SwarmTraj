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
The complete suite will be rerun after integrating the remaining planner stages.

## Remaining ordered work

1. Native proposals and resumable catalogs, preserving NumPy PCG64/SeedSequence
   stream consumption and deterministic candidate/rejection ordering.
2. Native bounded joint search, margin refinement, reservation checks and an
   independent final full-horizon recheck.
3. A complete native mission entry point with Python input/output adapters,
   followed by paired campaign, distribution and full-suite qualification.

No departure bound, margin, deadline, physical envelope, validation sample or
search budget may be weakened during migration. Runtime timings remain outside
canonical mission fingerprints.
