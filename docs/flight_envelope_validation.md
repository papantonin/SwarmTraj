# Expanded flight-envelope experiments

`scripts/validate_flight_envelope.py` extends the laboratory verification bench
without changing the flight model, limits, controller or planner. There are no
destinations, multi-aircraft missions or executable plan/control exports.

It also accepts one `--aircraft-profile JSON` for parameter-relative boundary
probes and speed/bank/slope grids. The original no-profile mode and its historical
fixtures remain unchanged. See [heterogeneous aircraft](heterogeneous_aircraft.md)
for the internet-inspired Rascal example, exact assumptions and recorded runs.
An admitted boundary or steady equilibrium does not establish maneuver
reachability or aircraft calibration; simultaneous exclusions remain visible.

## Commands

```bash
# Boundary classifications and steady-equilibrium tests only.
.venv/bin/python scripts/validate_flight_envelope.py

# Also request the existing mild vertical fixtures at three command intervals.
.venv/bin/python scripts/validate_flight_envelope.py --dynamic

# Additional coarse/fine command-interval checks, keeping the physical defaults.
.venv/bin/python scripts/validate_flight_envelope.py \
  --speeds 8 12 16 --dynamic --time-steps .5 .025

./scripts/test.sh tests/test_flight_envelope_validation.py \
  tests/test_vertical_flight_validation.py tests/test_flight3d.py
```

Each run reserves a fresh `results/benchmarks/flight-envelope-<timestamp>.json`
file; `--output` requires a new path. No GUI or hardware is used. The two dynamic
commands above return **exit code 2** in the recorded experiments: some requested
trials fail, and those failures are deliberately retained.

## Three different types of evidence

| Experiment | Default coverage | What is checked |
|---|---|---|
| Instantaneous boundaries | 66 samples across 22 boundary families and all 17 envelope margins, plus 6 input guards | Just inside, on and just outside each selected bound; non-finite/nonpositive-speed rejection |
| Steady equilibria | 9 speeds × 11 slopes × 5 bank angles = 495 samples | Independent force/limit classification; admitted equilibria integrated for two seconds and compared with analytical motion |
| Optional dynamic fixtures | 9 speeds × 3 fixtures × 3 command intervals = 81 trials | Existing level/climb/descent/recovery tests, with independent classification and numerical checks |

Default speeds are 8, 8.05, 9, 10, 11, 12, 13, 15.95 and 16 m/s. Steady-state
slopes cover −45° to +45° at eleven selected values; banks are −50°, −25°, 0°,
25° and 50°. The instantaneous boundary fixtures use the recorded default
physical parameters and fixed laboratory states. They are not a configurable
aircraft-envelope optimizer.

Only oracle-admitted steady equilibria are integrated. An equilibrium rejected
for thrust, braking, climb/descent rate or loaded stall is kept as classification
evidence, not counted as a successful flight. Its rejection does not show that
all maneuvers or missions from that state are impossible.

The dynamic fixtures remain the existing **±8° unbanked laboratory signals**.
The wider static angle grid does not validate high-amplitude dynamic transitions,
pull-outs, simultaneous roll/pitch transients or continued flight from every
instantaneously admitted boundary sample.

## Independent classifications and diagnostics

The scalar `oracle` separately evaluates all envelope inequalities using
`math` operations and recorded parameters. It calls neither the production
derivative, velocity, drag nor stall-ceiling methods. Its decision and each
margin are compared with `envelope_ok` and the observer's `envelope_margins`.
This is implementation cross-checking under shared equations/parameters, not
independent experimental validation of an aircraft.

Boundary perturbations are ±1e-4 in the target margin's native unit. Units are
recorded per rule. The production numerical envelope tolerance stays **1e-7**;
the cross-implementation margin comparison tolerance is 1e-10 per native unit.
The maximum reported native-unit discrepancy is a debugging summary, not a
physical norm combining meters, radians and load factors.

Some limits cannot be exercised as isolated acceptance boundaries with the
chosen default fixtures. At minimum load and structural maximum load, the
fixtures already violate vertical acceleration and flight-path rate. Reports
retain `other_violations`; an inward target perturbation is **not** automatically
an admissible state. These fixtures verify the corresponding margins but do
not independently establish global-gate enforcement of a redundant constraint.

Each dynamic trial reports:

- `classification_consistent`: independently evaluated envelope decisions
  agree with the original bench;
- `numerical_precision_passed`: the original derivative, energy and independent
  integration comparisons meet their unchanged tolerances;
- `summary_consistent`: independently recomputed settling/acceptance agrees
  with the reported outcome;
- `contract_consistent`: all three conditions hold;
- `trial_accepted`: the requested fixture itself satisfies every original
  check, which is a separate question from consistent rejection;
- first **observed sampled** violation times and minimum margins for each rule.

Dense actual-state checks cover both interval boundaries and three interior
fractions, with eight RK4 subdivisions, plus the original nominal endpoints.
First observed times are not exact first-crossing events. A two-second steady
equilibrium test is not a full-horizon or continuous-time certificate.

## Completion and failure handling

The JSON stores selections, limits, tolerances, units, software versions, source
hashes and individual results. Code changes during a run invalidate completion.
Files are opened exclusively; exceptions propagate and incomplete evidence
retains false completion/success flags. Abrupt process termination may leave a
partial file; this is not atomic or power-loss-durable recording.

- `all_contract_checks_passed` requires agreement/accuracy of all requested
  classifications and numerical comparisons.
- `all_requested_dynamic_trials_accepted` is `null` if no dynamic trials were
  requested, otherwise the conjunction of their original acceptance results.
- `all_checks_passed` and exit code 0 require complete contract checks and no
  failed dynamic requests. A completed failed check returns exit code 2.

Rejected equilibrium grid points are expected classifications derived from the
independent inequalities. Failed positive dynamic requests are **not** relabelled
as expected negative tests after the fact. A correct classifier can therefore
coexist with an unsuccessful overall campaign.

The report uses `swarm-flight-envelope-validation-v1`, distinct from mission
and campaign-journal schemas. Do not pass it to `audit_campaigns.py`.

## Findings

The main sweep produced **213 admitted and 282 excluded equilibria** out of
495. Every admitted equilibrium met the two-second analytical integration
comparison and sampled-envelope checks. Maximum analytical position error was
about **1.48e-6 m**. This is numerical agreement, not real positioning accuracy.

All 66 boundary and six guard classifications agreed. The largest numerical
margin discrepancy was about 8.9e-16 in a margin's native unit. Equilibrium
exclusions included climb-rate, descent-rate, thrust, braking and loaded-stall
constraints; violation counts overlap because a sample can violate several.

For the main 81 dynamic trials at 0.2, 0.1 and 0.05 s:

- **72 were accepted**, and **9 requested trials were refused**;
- all independent classifications, summaries and precision checks agreed;
- the refused trials were descending fixtures at exactly 8 m/s and both
  climbing/descending fixtures at exactly 16 m/s, repeated at the three intervals;
- held laboratory forces produce small airspeed excursions at the configured
  speed boundaries. The explicit numerical tolerance is retained, not widened.

The supplementary 8/12/16 m/s sweep at 0.5 and 0.025 s added 18 temporal trials:
11 accepted and seven refused. Four coarse-step fixtures exceeded the existing
numerical consistency tolerances, with held-solver position differences up to
**9.13e-6 m**, above the 2e-6 m criterion. Their envelope classifications still
agreed; precision failure must not be described as a classifier error or proven
physical infeasibility.

Changing command interval also changes the held force inputs, so acceptance at
an exact speed boundary is not necessarily monotonic. In particular, the 16 m/s
descending fixture is accepted at 0.025 s within the unchanged 1e-7 tolerance,
but refused at 0.05–0.2 s. This is neither an integration-order proof nor a
guarantee of strict physical compliance between samples. No command interval
or physical parameter is changed automatically in missions.

Artifacts recorded at the source revision used for these experiments (not
necessarily today's source revision):

- Main sweep: `results/benchmarks/flight-envelope-20261027-144755-111460.json`.
- Coarse/fine extension: `results/benchmarks/flight-envelope-20261027-144756-376223.json`.
- Classification-only repeat: `results/benchmarks/flight-envelope-20261027-144847-508517.json`.

The first two commands completed with exit code 2 and retained their failed
requests/checks. The classification-only repeat completed with exit code 0.
Supplementary equilibrium/boundary samples repeat subsets of the main grid;
do not count them as additional unique configurations.

**200 focused tests passed**, including the expanded bench, original vertical
fixtures, existing point-mass tests and four-aircraft regressions, campaign
audit/runner and trajectory CLI tests. Regression tests cover deliberately
broken gates/margins, oracle independence, coupled exclusions, coarse-step
precision failures, edge acceptance under the explicit tolerance, source changes,
exceptions, interruptions and refusal to overwrite output. No GUI changed; the
full repository suite was not rerun for this additive laboratory tool.

No real-aircraft calibration, coupled transient-envelope certification,
collision guarantee or resolution of the geometric planner limitation follows
from these experiments. See [vertical fixtures](vertical_flight_validation.md)
and [flight-model assumptions](flight_model_3d.md).
