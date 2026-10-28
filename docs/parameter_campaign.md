# Parameter campaign and rejection-guided candidate proposals

## Reproducible four-aircraft experiments

`scripts/validate_scenarios.py` exercises four aircraft throughout, with several
targets, initial positions/altitudes, speeds, approach spreads, departure bounds,
strict/mixed profiles and known static obstacles. It plans each mission once,
then reuses exactly the same immutable plans across execution profiles:

| Profile | Experiment | Expected result |
|---|---|---|
| `nominal` | No disturbance or clock error | All execution criteria pass |
| `mild` | 0.05 m/s rotating wind, offsets up to 0.03 s | All criteria pass in the tested trial |
| `gust_sensor` | 0.1 m/s synthetic gust, offsets 0.03 s, drift 0.0001 s/s, bias 0.05 m, noise 0.01 m | All criteria pass in the tested trial |
| `outside_clock` | Offsets up to 0.4 s, beyond the configured clock assumption | Execution must be rejected |

Disturbance seed defaults to 23. Gusts are horizontal velocity disturbances,
not aerodynamic loads. Sensor/clock semantics and acceptance limitations remain
those of [robust tracking](robust_tracking.md). A finite set of passing trials
does not establish a continuous robust-flight envelope.

Two input-negative cases check necessary rejection: an unreachable horizon and
already overlapping initial positions. Difficult mission requests, including
strict two-level fixed departures and heterogeneous speeds, remain positive
requests: an unsuccessful catalog is retained as a failed expectation, not
reclassified afterward as an expected negative.

```bash
# Broad campaign, historical proposal generation. May expose retained failures.
.venv/bin/python scripts/validate_scenarios.py --candidate-guidance none

# Same cases with the explicit rejection-guided proposal policy.
.venv/bin/python scripts/validate_scenarios.py --candidate-guidance rejections

# Focused repeat with another seed and identical planned controls per profile.
.venv/bin/python scripts/validate_scenarios.py --candidate-guidance rejections \
  --scenarios obstacles_strict obstacles_mixed --seeds 19 \
  --profiles nominal mild gust_sensor outside_clock
```

Each fresh JSONL output is opened exclusively; existing files are never
overwritten. Header, mission and execution records are flushed individually,
including failures. Worker exceptions are recorded and propagated, never turned
into acceptance of partial output. A source checksum guard prevents mixing
planner/script revisions during a campaign. The header records source hashes,
selected cases/profiles/seeds and software/CPU metadata. Mission records include
every configuration, initial snapshot, status, signature, diagnostic and timing.
Execution records include all acceptance criteria and whether plans stayed
unchanged. Exit code 2 means at least one expectation was unmet; it is not a
claim of physical infeasibility. Caught exceptions leave an error record and no
successful summary; external interruption may leave only the completed records
and no summary. Flushes are not a power-loss durability guarantee. This is
offline simulation, not a real-time flight trial.

## Offline audit of saved evidence

Use the standalone standard-library tool to inspect saved journals without
loading the simulator, running the planner or modifying any input file:

```bash
.venv/bin/python scripts/audit_campaigns.py \
  results/benchmarks/parameter-campaign-after-20261026.jsonl \
  results/benchmarks/parameter-campaign-seed19-20261026.jsonl

# Machine-readable output, including per-journal metric coverage.
.venv/bin/python scripts/audit_campaigns.py \
  results/benchmarks/parameter-campaign-compatibility-20261026.jsonl --json
```

The first command returns **2**, as expected: those complete, internally
consistent journals still contain three unmet positive planning requests.
The compatibility journal returns **0**. The gate requires all three conditions:

- `complete`: every declared scenario/seed has a mission record, every validated
  mission has each declared execution profile, and a terminal summary exists,
  with no recorded campaign error;
- `audit_valid`: records are internally consistent, without malformed JSON,
  non-finite numbers, duplicate keys/trials, records after the summary,
  signature disagreement, contradictory acceptance fields or summary counts;
- `all_expectations_met`: mission statuses and execution outcomes satisfy their
  recorded expectations, and plan immutability was retained.

Exit code 0 requires all three for **every** supplied journal; otherwise it is
2. A complete campaign with unsuccessful requested planning remains a failed
campaign, not an invalid journal or proof of physical infeasibility. A recorded
expected rejection is not counted as a successful flight. Input path duplicates
are rejected to avoid counting the same file twice. Distinct files can still
contain repeated experiments; totals count trials, not unique configurations.

The auditor cross-checks the saved execution acceptance criteria, failure list,
acceptance flag and mission/plan signatures. It does **not** reintegrate controls,
recompute separation, recompute a signature from flight arrays, authenticate
records or establish that an aircraft can fly a plan. Source hashes are checked
for presence/format, not against today's source: old revisions remain legitimate
evidence. The audit's acceptance contract is regression-tested against the
current observer gate, but future journal contracts may require a schema update.

New journals declare `swarm-scenario-campaign-v1` and the runner rejects duplicate
scenario, seed or profile selections before creating an output. Historical
headers without a schema remain supported when they declare selections and
source fingerprints. The earliest `parameter-campaign-before-20261026.jsonl`
does **not** declare either: its observed counts can be checked, but full coverage
and code provenance cannot be established retrospectively. Its audit therefore
fails closed with `coverage_declared=false`, rather than inferring that nothing
is missing from the trials that happen to be present. Do not edit that historical
journal to invent missing metadata.

Metric summaries include only internally consistent, unchanged-plan positive
executions that passed their recorded criteria. Each field reports a value and
its number of contributing trials. Missing obstacle measurements remain `null`
with zero observations, not a misleading distance of zero. Sampled distances
and numerical lower bounds remain separate metrics. Rejected trials never
improve these statistics. These summaries are evidence within the saved records,
not acceptance of a journal that fails the overall audit.

The three original main journals were audited without rerunning any trajectories:
their records total 26 planning trials, 51 accepted positive executions and 17
expected outside-clock rejections. All five unmet planning expectations remain
visible. The two later main journals are complete and internally consistent;
the baseline has the legacy metadata limitation above. No flight model,
controller, generator, physical limit or safety margin changes in this audit step.

Verification of this addition: **98 focused tests passed** (auditor, campaign
runner and trajectory CLI contracts). These cover truncation, missing requested
trials, duplicates, non-finite JSON, contradictory acceptance, signed obstacle
penetration in failed trials, immutable inputs and the observer-gate contract.
The auditor also runs with Python `-I -S`, without simulator imports. Both
negative-input cases were rerun with four aircraft and their new journal passed
the audit; the saved two-mission compatibility journal also passed. Historical
main journals correctly returned exit 2 with the retained failures and legacy
metadata limitation above. No new positive flight simulations or full-suite
rerun were needed for this reporting-only change; the 434-test suite result
below belongs to the preceding planner change, not this audit addition.

## New candidate feedback policy

`--candidate-guidance rejections` is opt-in; `none` preserves the historical
candidate stream. It currently requires coupled 3D flight. The policy uses only
the aircraft's own catalog rejection history, including in the initial catalog
before any compatible fleet combination exists. It is different from:

| Mechanism | Input used before proposal rollout |
|---|---|
| Candidate feedback | That aircraft's own numerical rejection counts |
| Conflict guidance | Offline sampled candidate-pair witnesses |
| Departure windows | Forecast cell/calendar overlap under release shifts |
| Intermediate passages | Forecast free cell intervals and cached geometric references |

After at least two matching rejections, alternate ordinary proposals with
bounded reference adjustments:

- `settle_levels`: after `validation_cruise_level_not_reached`, retain all
  requested levels and propose holds `max(5, requested_hold + 2)` or
  `max(5, requested_hold + 4)` seconds. This gives the physical controller more
  time to settle after transitions. The hold option is a reference construction
  preference, not a required exact physical dwell. The existing ordered
  attainment test (one second settled within 0.5 m and 0.2 m/s) is unchanged.
- `wider_route`: after `obstacle_clearance`, add 5, 10 or 15 m to the padding
  used by the visibility-route heuristic, including onward routing. This is
  **proposal padding**, not an additional certified actual clearance or a
  change to the configured hard obstacle margin.

Cycle available recipes deterministically. Local passage fitting takes
precedence when applicable; otherwise feedback can coexist with existing
bounded conflict/departure hints. Each proposal counts against the same
cumulative attempts and candidate caps. Per-ID PRNG streams and recipe counters
survive catalog resumption and ordered spawn transfer.

No existing plan is changed. Every proposal is rolled out from the original
snapshot with the original coupled physical configuration, then independently
validated for all physical, level/speed, target/angle, obstacle, timing and
recovery requirements. Full joint separation/grid checks remain mandatory.
Longer holds that do not fit are rejected; neither the horizon nor departure
window is extended. Strict profile requests are never changed into simpler
families. Recipe information stays in ground-side `candidate_proposals`
diagnostics, with individual acceptance/rejection; it is not fleet telemetry
or a new onboard coordination dependency.

## Geometric-template headroom diagnostic

The broad campaign exposed an important generator limitation for a 13 m/s
aircraft, a 30-degree descending pass and target altitude 30 m. Coupled preflight
uses total speed and `V sin(gamma)`; the current geometric template uses a
constant horizontal-speed proxy and `V tan(gamma)`, with a conservative 0.9
vertical-acceleration factor. Retiming changes the clock, not the curve's
spatial recovery depth. A geometric curve can therefore require more height
than the physical preflight's necessary bound even when that check passes.

`diagnostics.timing.geometric_template` now reports, per aircraft:

- geometric reference recovery drop;
- coupled preflight recovery drop (a necessary model check, not a guarantee);
- available height above the floor;
- whether the proxy is more restrictive than that preflight check.

The CLI prints this limitation explicitly. It does **not** declare the coupled
mission impossible, silently alter speed/angle/target altitude, or relax the
floor. The campaign includes `heterogeneous_speeds_high_target` as a separate,
explicit target-height variation (45 m, not an automatic correction of 30 m).
Replacing this proxy with a direct coupled/kinodynamic reference generator is
still future work. Simply increasing the geometric acceleration in a probe
caused many physical-envelope rejections; it was not adopted as a solution.

## Initial campaign observations

Before the new feedback policy, the seed-7 campaign made eleven mission
requests. Seven planned successfully; all 21 nominal/mild/gust-sensor executions
passed every acceptance criterion, and all seven out-of-bound clock executions
were rejected. The two deliberately invalid input cases were rejected before
generation. Two requested missions remained unsuccessful within finite catalogs:

- `fixed_obstacles_strict`: no accepted candidate for aircraft 3, despite 90
  attempts for that aircraft; obstacle, envelope, ordered-level and angle checks
  all contributed. Initial holds alone did not solve this scene.
- `heterogeneous_speeds`: all 90 proposals for aircraft 3 failed in the vertical
  template. The headroom diagnostic explains why increasing horizon alone is
  not a remedy for this particular construction limitation.

Raw baseline: `results/benchmarks/parameter-campaign-before-20261026.jsonl`.
The baseline exit code is 2 and those failures are retained. It is not correct
to describe the complete parameter space as functional from these experiments.

## Regression and compatibility checks

The complete post-change suite passed **434 tests**, with four existing
Gymnasium float32 warnings. Focused feedback/campaign/CLI/timing tests passed
before the full run. Real four-aircraft tests verify strict level/speed
requirements, cumulative budgets, serial/spawn equality, immutable plans and
nominal physical execution. Campaign tests cover negative-input rejection,
incremental failure records, source-change aborts, exception propagation and
refusal to overwrite outputs.

The historical `default` and `obstacles_strict` cases were regenerated with
feedback disabled and one worker. Both exact plan/result signatures match the
pre-change four-worker campaign. New configuration/advisory fields legitimately
change whole-mission manifests, not those individual flight plans.
Compatibility artifact:
`results/benchmarks/parameter-campaign-compatibility-20261026.jsonl`.

The new CLI was also exercised headlessly with four aircraft, seed 19, strict
two-level/smooth profiles, the unchanged large obstacles and all guidance
options. Nominal execution passed every criterion; minimum sampled separation
was 51.17 m and obstacle-surface distance 31.28 m. Each exported payload contains
only its own plan/reservations and shared configuration, not recipe catalogs
or other-aircraft state. Artifacts:
`results/trajectory3d/20261026-180900-733511/`.

Wall-clock timings overlap other local checks and are not isolated performance
measurements or deadline guarantees. Proposal-count changes are reported as
observations, not uniform gains or evidence of physical infeasibility.

## Completed before/after campaign

The three main journals contain **26 planning trials** (including repeated
before/after configurations and a second obstacle-scene seed), all with four
aircraft. Seventeen missions planned successfully. Their identical immutable
plans were reused across four profiles, for **68 executed trials**:

- All **51 nominal/mild/gust-sensor** executions passed every acceptance criterion.
- All **17 outside-clock** executions were rejected as expected.
- Four necessary input rejections (two repetitions of each negative case)
  occurred before execution.
- Five positive mission requests returned `no_candidate`: repetitions of the
  same two unresolved construction/catalog cases, not five new proven physical
  impossibilities. All three journals therefore retain `all_expectations_met=false`
  and their runner returned exit code 2.

Across accepted positive executions, minimum sampled inter-aircraft separation
was **25.49 m**, minimum sampled obstacle-surface distance **21.64 m**, and the
largest reported tracking-error upper bound **0.712 m** (rounded upward).
These statistics describe these finite numerical experiments only.

The observed proposal counts are deliberately mixed:

| Configuration | Historical proposals | With rejection feedback | Outcome |
|---|---:|---:|---|
| Strict rich obstacle scene, seed 7 | 59 | 80 | Both validated |
| Strict rich obstacle scene, seed 19 | 54 | 34 | Both validated |
| Strict rich fixed departure, seed 7 | 137 | 114 | Both `no_candidate` |
| Strict rich fixed departure, seed 19 | 154 | 106 | Both `no_candidate` |
| Heterogeneous speeds, target Z=30 m, seed 7 | 99 | 99 | Both `no_candidate` |
| Heterogeneous speeds, explicit target Z=45 m, seed 7 | Not run | 56 | Planned; all three positive execution profiles passed |

The seed-19 historical strict-rich reference is the saved
`results/benchmarks/passage-unique-fits-20261026.json` run. Physical inputs and
budgets match; worker count differs (four previously, two in the new campaign).
It is a workload comparison, not an isolated wall-time comparison.
The fixed-departure seed-19 reference is the retained pre-feedback experiment
in `results/benchmarks/passage-fixed-departure-20261026.json`.

One settling-adjusted plan was selected in the final seed-19 strict fleet and
passed all tested positive execution profiles. Wider-route alternatives were
actually generated and one was individually accepted in the explicit higher
target case, but that alternative was not selected. Individual acceptance,
final selection and global search improvement must not be conflated. The new
policy can increase global search work, as seed 7 demonstrates, and remains
opt-in. It does not yet resolve either difficult original mission request.

Final journals:

- `results/benchmarks/parameter-campaign-before-20261026.jsonl`: 11 missions, 28 executions.
- `results/benchmarks/parameter-campaign-after-20261026.jsonl`: 12 missions, 32 executions.
- `results/benchmarks/parameter-campaign-seed19-20261026.jsonl`: 3 missions, 8 executions.

The two additional compatibility executions and the CLI trial described above
also passed; they are not included in the 68 main-campaign executions.
The principal next model improvement is a direct coupled reference generator
that avoids the current conservative recovery-depth proxy, followed by fresh
validation under unchanged physical and safety limits.
