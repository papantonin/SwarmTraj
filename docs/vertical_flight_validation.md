# Isolated vertical-flight physics validation

`scripts/validate_vertical_flight.py` is a laboratory verification bench for the
existing coordinated point-mass equations. It has **no mission planner,
destination, swarm coordination or guidance-controller calls**. It changes
neither physics nor flight limits. Its summary is not a deployable flight plan.

The separate [expanded envelope bench](flight_envelope_validation.md) adds
instantaneous boundary checks, wider bank/slope equilibrium classifications and
speed/command-interval sweeps. It retains the original mild signals for dynamic
tests and does not silently change this bench's defaults or tolerances.

## Running the bench

```bash
# Four separate speed settings, not a four-aircraft coordinated mission.
.venv/bin/python scripts/validate_vertical_flight.py --refine

# Smaller repeat, keeping the same physical defaults.
.venv/bin/python scripts/validate_vertical_flight.py \
  --speeds 10 13 --cases climb_recovery descent_recovery --dt 0.1 --refine

./scripts/test.sh tests/test_vertical_flight_validation.py tests/test_flight3d.py
```

Defaults are total airspeeds 10, 11, 12 and 13 m/s, nominal command interval
0.2 s and three test signals. Each run writes a fresh
`results/benchmarks/vertical-flight-<timestamp>.json`; `--output` selects a new
path. Existing files are never overwritten. There is no GUI or hardware output.

`--aircraft-profile` loads the common physical parameters from a validated
[fixed-wing profile](aircraft_profiles.md). Speed settings remain explicit:
provide `--speeds` if the defaults are outside the loaded envelope. A profile
record/hash is included in the result, and negative checks require an admissible
level-flight baseline at the first selected speed. No forces or limits are
silently adjusted to make a fixture pass.

`--dt` must be 0.025–0.5 s and divide the fixture phase durations exactly.
CLI speed settings must be unique and inside the effective profile's range (8–16 m/s by default).
The original reference-bench defaults remain unchanged without a profile. A
speed at an envelope boundary is not guaranteed to pass: discretized
forces can produce a small real speed excursion, which must not be clipped.

## Prescribed laboratory motion

All fixtures start at altitude 100 m, heading toward +X, with zero bank and
flight-path angle. The phases are:

```text
0 s              2 s                         12 s              16 s
| initial level  | smooth vertical variation | final level     |
```

- `level`: constant altitude and total airspeed throughout;
- `climb_recovery`: a smooth positive flight-path-angle pulse, peaking at 8°,
  then a return to level at the newly reached altitude;
- `descent_recovery`: the corresponding negative pulse followed by level flight.

The last two signals are manufactured solutions: a prescribed smooth slope
and constant total airspeed define laboratory force inputs under the same
equations. They are not generated from a destination or optimized to achieve
a maneuver. No reference feedback, trajectory-selection machinery or control
saturation is used. A force exceeding a configured limit is reported as a
failure, not silently clipped. The final four-second phase must have sampled
vertical speed below 0.1 m/s and absolute slope below 0.5° throughout.

Returning to level **does not mean returning to the initial altitude**. These
are deliberately mild, unbanked tests, not validation of steep descending
passes, obstacle routing, takeoff, landing or emergency recovery.

## Independent comparisons

The bench separates three questions:

| Question | Comparison | Interpretation |
|---|---|---|
| Are units, signs and forces consistent? | Manufactured state derivatives, total-speed kinematics and mechanical-energy rate | Algebraic consistency within the assumed equations |
| Does the implementation integrate held forces correctly? | Production RK4 versus separately written unbanked equations integrated with SciPy DOP853 | Numerical implementation comparison for identical held controls |
| What does command sampling change? | Recompute fixture forces with a halved command interval using `--refine` | A different held-force experiment, **not** an RK4-order estimate |

The independent solver does not call the production derivative or drag method.
It has separately written `[x,z,V,gamma]` equations, but uses the same physical
assumptions, parameters and gravity constant. Agreement cannot validate those
assumptions against real aircraft data.

For each nominal held-control interval, both boundaries and three interior
fractions are checked using eight RK4 subdivisions. The original nominal states
are also checked. Scalar production integration is compared with batched,
eight-subdivision integration under the same forces. Physical checks use the
existing flight envelope, including airspeed, altitude, climb/descent rate,
vertical acceleration, load/stall, slope, bank and actuator limits. Minimum
sampled margins are retained even on failures.

The mechanical-energy identity uses specific energy:

```text
E = V²/2 + g z                          [J/kg]
dE/dt = V (thrust/m - brake/m - drag/m) [W/kg]
```

This identity detects internal force/energy inconsistency; it is not battery
consumption or an endurance model. Angle is flight-path angle, not body pitch.
Total airspeed remains distinct from horizontal speed.

## Acceptance and traceability

The report records all tolerances with units, physical/kinematic limits,
fixture parameters, requested cases/speeds, software versions and source hashes.
`trial_passed` requires algebraic/numerical consistency, sampled envelope
compliance and a settled final level section. With `--refine`, both resolutions
must pass, and reference-position error must not increase, allowing for the
explicit numerical floor in already near-exact level cases.

Twelve additional negative checks deliberately violate representative limits,
including speed, floor/ceiling, bank, force/load, roll rate and non-finite data.
Their expected envelope rejection is recorded separately; they are not successful
flights. Some invalid samples violate more than one limit. They are not an
exhaustive boundary analysis.

Exit code 0 requires a completed run, passing positive trials and all expected
negative rejections. Exit code 2 means a completed numerical check failed.
Unexpected integration exceptions propagate with error evidence and
`completed=false`, `all_trials_passed=false`. A source change during the run
also invalidates the result. A caught interrupt leaves incomplete evidence;
an abrupt external termination can leave an incomplete JSON file. This is not
an atomic or power-loss-durable experiment recorder.

Reports contain summaries, not executable controls or flight plans. They use
`swarm-vertical-physics-validation-v1`, not the mission/campaign journal schema;
`audit_campaigns.py` must not be used to interpret these files.

## Observed results

The default `--refine` run completed with 12 nominal fixtures, 12 halved-command
repeats and 12 expected negative rejections. All checks passed. Artifact:
`results/benchmarks/vertical-flight-20261026-190728-000051.json`.

Across the 12 nominal fixtures:

- Maximum position disagreement with the independent held-force solver was
  approximately **1.14e-7 m**; this is numerical agreement, not navigation accuracy.
- Maximum vertical speed in the final level section was **0.000372 m/s**.
- Maximum position difference from the ideal continuous fixture was **0.1886 m**
  with 0.2 s commands, falling to **0.0925 m** with 0.1 s commands. Nontrivial
  fixture error ratios were about 0.489–0.493. Held-command discretization and
  integration error must not be conflated.

**163 focused tests passed**: the new laboratory bench, existing flight-model
tests (including four-aircraft planning/execution regressions), campaign audit,
campaign runner and trajectory CLI tests. No model/plan/control behavior was
changed. The full repository suite was not rerun for this additive bench.

These results do **not** fix or prove feasible the conservative geometric
recovery-depth case described in [parameter campaigns](parameter_campaign.md).
They provide an independent baseline for modest vertical maneuvers. The
geometric construction limitation and difficult finite-catalog missions remain
open. RK4 and sampled envelopes still have no certified continuous-time error
bound; see [flight model assumptions](flight_model_3d.md).
