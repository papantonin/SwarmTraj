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

## Remaining ordered work

1. Native bounded joint search, margin refinement, reservation checks and an
   independent final full-horizon recheck.
2. A complete native mission entry point with Python input/output adapters,
   followed by paired campaign, distribution and full-suite qualification.

No departure bound, margin, deadline, physical envelope, validation sample or
search budget may be weakened during migration. Runtime timings remain outside
canonical mission fingerprints.
