# Migrating geometric proposals to direct physical generation

The physical builder is available throughout the coupled 3D preparation path:
trajectory CLI, Python editor, versioned shared scenarios, and the one/four
aircraft PX4/Gazebo adapters. The current default remains `geometric` pending
the validation gates below. Selecting the physical builder changes reference
construction, not Gazebo aerodynamics, PX4 control or the planner's safety limits.

## Use the physical builder explicitly

```bash
# Point-mass planning and own-state execution; no native simulator.
.venv/bin/python examples/trajectory_3d.py --no-gui \
  --generator-mode physical_mission --workers 2

# Edit, calculate and then fly one simulated aircraft.
.venv/bin/python examples/px4_gazebo.py --edit-scenario --run --gui \
  --scenario examples/scenarios/px4_single_loiter.json \
  --generator-mode physical_mission --workers 1 --timeout 1200

# Four aircraft in one isolated native world.
.venv/bin/python examples/px4_gazebo.py --num-drones 4 --edit-scenario --run --gui \
  --scenario examples/scenarios/px4_four_loiter.json \
  --generator-mode physical_mission --workers 2 --timeout 1200
```

Remove `--run --gui` to prepare/export without starting Docker. In the editor,
**Physical generator** checked means `physical_mission`; unchecked means
`geometric`. **Reset** restores the input mode as well as the scene. Planar and
legacy flight retain their historical builder. Physical passage-window fitting
remains unsupported and fails explicitly, preserving the original request.
The editor keeps the previous configuration if a switch is invalid.

New single/fleet JSON records use `swarm-px4-editor-v2` and
`swarm-px4-editor-fleet-v2`, respectively. Both require
`mission.generator_mode` (`geometric` or `physical_mission`). Existing v1 records
explicitly load as geometric; an explicit CLI/editor override may then change
the mode and export a v2 record. Unknown fields and missing v2 mode are rejected.
This preserves the meaning of archived scenes independently of future defaults.

## Performance and comparison procedure

Profiling the five formerly censored missions located the main cost in dense
physical rollouts, rather than joint search. `flight3d.derivative` now fills one
output array instead of repeatedly building stacked velocity/acceleration arrays.
`envelope_ok` computes vertical velocity directly. Arithmetic order, integration
subdivisions, sample points, tolerances and all envelope checks are preserved.
Frozen historical-formula tests compare derivatives and complete dense RK4
rollouts exactly. This is a performance change, not an integration-error bound.
Physical step checks now batch the refined samples and the separately integrated
nominal endpoint in one envelope call. Both integrations and both switch sides
are retained; the nominal endpoint is not substituted by a refined endpoint.

For obstacles with a free approach, the physical builder preserves the first
five proposals, then explores bounded deterministic heading offsets. This fixes
the case where all terminal joins fell inside the same obstacle. Assigned
approach sectors are never altered, and independently rejected proposals remain
rejected. The supplied four-aircraft scene now produces a jointly validated
reference: 126.32 m full-horizon distance lower bound versus 38.8 m required.

Campaigns can explicitly select process counts and execution families:

```bash
.venv/bin/python scripts/benchmark_scaling_campaign.py \
  --reference results/scaling-campaign-20261028/manifest.json \
  --suites controls core random replay --generator-mode physical_mission \
  --planning-workers 2 \
  --execution-families baseline obstacles heterogeneous rich_strict grid \
  --output results/physical-migration-parallel
```

Run the same command with `geometric` and with one/two processes, using new output
directories. Dedicated serial/spawn replay pairs keep their saved process counts.
`--jobs ID ...` selects a diagnostic subset and records it in the manifest; a
subset cannot qualify a default change. References are copied, never rewritten.
The saved timeout remains authoritative. Missing/failed executions and censored
planning stay visible. Compare signatures, accepted plans and matched execution
expectations before interpreting timings; desktop load and process startup matter.

## Gates before changing the default

1. Preserve planar/legacy behavior and the meaning of saved geometric scenarios.
2. Complete the five former >120 s jobs under the same cap and catalog/search
   budgets. `no_candidate` or `catalog_exhausted` is a completed bounded search,
   not a continuous-space infeasibility proof or a successful mission.
3. Compare identical saved missions with both builders and one/two processes,
   including the same requested nominal, gust/sensor and negative execution
   profiles. Retain all previous accepted missions and independently revalidate
   selected trajectories through the full horizon.
4. Pass relevant and full tests, scenario serialization and editor checks.
5. Run actual isolated PX4/Gazebo experiments at one then four aircraft. Check
   readiness, own-state initialization, native mission feasibility, target contact,
   spatial thresholds and completion evidence without weakening acceptance gates.
6. Promote only after the evidence supports it. Keep an explicit geometric option
   for historical replay and the physical builder's unsupported request boundary.

The original point-mass plan remains immutable. Native spatial waypoints lose
its timing and separation claims. A native experiment failure must remain a
failure even when the point-mass reference validates. Neither numerical model
establishes calibrated or certified real-aircraft safety.

The development record and measurements are stored under
`results/physical-rollout-migration-20261029-132241/`; generated telemetry and
reports remain ignored by Git. Earlier 230-mission evidence is documented in
[the physical generator guide](phase2_kinodynamic.md#completed-saved-input-comparison).

## Measured migration checks (29 October 2026)

On the same two-core/four-thread Intel Core i3-1115G4 desktop, a sequential
replay of the five former timeouts completed within the original 120 s process
cap, with one catalog process and unchanged mission inputs:

| Saved job | Process wall time | Outcome |
|---|---:|---|
| `core-short_horizon-n5-s7` | 64.22 s | `no_candidate` |
| `core-short_horizon-n5-s19` | 62.42 s | `no_candidate` |
| `core-grid-n3-s7` | 51.66 s | `catalog_exhausted` |
| `core-grid-n3-s19` | 48.68 s | `catalog_exhausted` |
| `core-grid-n3-s42` | 51.90 s | `catalog_exhausted` |

There is still no selected joint plan for these five jobs. These completed
bounded failures replace neither the original timeout records nor the need for
a fresh full qualification campaign. An earlier development timing pass retained
one 120 s timeout; that record is preserved too. The final measurements above
include the batched envelope optimization and were run without another planning
campaign in parallel. Desktop timings are observations, not runtime guarantees.

The native one-aircraft physical-reference experiment passed readiness, warm
initialization, loiter and spatial gates. Observed target contact was 5.43 m
inside the unchanged 6 m radius at simulation t=115.412 s; own-model removal was
acknowledged and absence verified. Spatial error p95 was 5.79 m against the
unchanged 25 m threshold. The original full-horizon reference was preserved.

The first four-aircraft native experiment failed: aircraft 1's estimated airborne
state did not converge within the existing startup gate. All four immediate
physical-state initialization checks passed, but that does not establish PX4
estimator convergence or full-flight tracking. The other workers were aborted;
no fleet flight/separation success is claimed. Reports and the failed trial are
retained under `fleet-native/`. This blocks promotion; startup thresholds and
physical/safety limits remain unchanged. Full-suite and matched generator/process
comparison records are separate from these native measurements.

The frozen migration implementation passed the complete repository suite:
**900 tests in 534.12 s**, with the four existing Gymnasium float32-bound
warnings. Focused checks cover exact dense-RK4 compatibility, retained nominal
and refined envelope samples, free versus assigned obstacle approaches,
single/fleet v1/v2 serialization, CLI overrides, editor rejection/reset behavior
and spawn reproducibility. An earlier full-suite process was interrupted by
SIGTERM before its summary; its log is retained and is not counted as a pass.
The editor window was opened and its final rendered layout inspected, but the
local native input provider did not provide a verified checkbox interaction.
Automated editor callbacks passed; an interactive click test remains unverified.

An identical native fleet repeat subsequently passed without source, parameter
or gate changes. Both trials retained exactly the same original mission SHA-256.
All four own target contacts/removals were accepted and scene
absence verified. Per-aircraft spatial p95 errors were 4.57–4.85 m, below 25 m.
The observer's minimum interpolated pair distance was 133.61 m against the native
experiment's 30 m threshold, using each pair's active overlapping interval.
This is one successful native experiment after one startup failure, not a
reproducibility or continuous-separation certificate. The repeat is retained in
`fleet-native-repeat/`; it does not replace the failed `fleet-native/` trial.
The geometric default remains pending broader qualification and reliable startup.

For the same saved single-aircraft inputs (ignoring schema/mode), the earlier
geometric native run `results/px4-gazebo/20261028-162939-172968/` and the new
physical run both passed:

| Native observation | Earlier geometric reference | Physical reference |
|---|---:|---:|
| Spatial error p95 | 5.60 m | 5.79 m |
| Maximum spatial error | 8.06 m | 9.27 m |
| First observed target distance | 5.96 m | 5.43 m |

These are individual native trials at different times, not evidence that either
generator improves PX4 tracking in general. A spatial overlay of the successful
four-aircraft repeat and its original physical references is retained as
`fleet-spatial-comparison.pdf` in the migration record.

## Matched generator/process comparison

The frozen migration revision completed all **176 jobs** in four variants:
15 identical saved exploratory missions, 19 negative controls and ten serial/
spawn replay jobs per variant. Each variant requested the same five execution
families. The exploratory subset deliberately includes the five former timeouts;
it does not reproduce the full 230-mission qualification denominator.

| Generator / catalog processes | Validated exploratory | Negative controls | Execution expectations | Exploratory planning wall-time sum |
|---|---:|---:|---:|---:|
| Geometric / 1 | 10/15 | 19/19 | 30/30 | 123.21 s |
| Geometric / 2 | 10/15 | 19/19 | 30/30 | 117.96 s |
| Physical / 1 | 10/15 | 19/19 | 30/30 | 406.21 s |
| Physical / 2 | 10/15 | 19/19 | 30/30 | 359.46 s |

All 44 result signatures per generator match between the one/two-process
variants. All five dedicated serial/spawn pairs also match within each variant.
All 44 geometric signatures match the historical records; all 39 physical
results comparable to the previous cached revision retain their exact signatures.
The other five previous physical results were censored, so no historical
trajectory signature is invented for them. Current physical process times for
these five cases are 55.77–76.89 s with one process and 54.35–58.56 s with two.
They remain two `no_candidate` and three `catalog_exhausted` outcomes.

Across variants, all **120 execution expectations** matched: 40 nominal and
40 bounded gust/sensor trials accepted, and 40 excessive-clock trials rejected.
Separate saved-plan checks revalidated 240 individual records, 328 full-horizon
pairs and 120 exact physical recovery connections, counting repeated selections.
All 240 corrupt-final-command and 240 post-arrival endpoint-obstacle trials were
rejected. Input/artifact/numerical audits report zero errors. This subset selected
no grid plans; positive reservation checks remain covered by the full suite and
earlier separate campaign evidence.

Physical construction remains more expensive on this matched sample. Two
processes reduce the summed planning time, but do not accelerate every case;
startup, adaptive catalog batches and uncontrolled desktop load matter. These
measurements exclude imports/export/execution and are not runtime guarantees.

The standard headless physical CLI also passed planning and own-state execution
with four aircraft and two catalog processes (23.86 s planning, 30.03 s total).
The complete record is `verification.json`; the combined report is `report.pdf`
and `report.md`, alongside the four immutable campaign manifests/journals.
The geometric default is retained pending a fresh full qualification and
reliable native fleet startup.
