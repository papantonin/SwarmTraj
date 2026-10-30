# Disturbed execution and local tracking

The 3D executor now defaults to `controller="constrained"`. Planning still
validates the same immutable nominal trajectories before joint selection.
No other aircraft's plan, position or clock enters a local controller. The
simulator's omniscient observer reports failures but never corrects aircraft.

This is a **numerically tested controller**, not a robust invariant-set proof,
certified control barrier or real-aircraft safety system. Tracking error must
still remain within the configured tube; neither margins nor arrival tolerances
are automatically relaxed.

## Controller changes

1. **Local control-switch scheduling.** A shifted reference clock may put a
   nominal actuator switch inside a 0.2 s simulation interval. The constrained
   executor also updates at its own plan's switch times instead of averaging
   two physically different maneuvers into one held command. The simulator maps
   local callbacks onto integration time using the experiment's clock model;
   controller feedback still receives only its own measurements and reference.
   This is not an implementation or certification of a real timing subsystem.
2. **Own-motion wind estimation.** From consecutive measured positions and the
   previous local command, predict no-wind displacement with the onboard model:

   ```text
   w_observed = (p_measured_now - p_predicted_now_without_wind) / local_elapsed_time
   w_estimated += (1 - exp(-local_elapsed_time / 1.5 s)) * (w_observed - w_estimated)
   ```

   Only the horizontal components are estimated. Constant position bias cancels
   under the translation-invariant model; noise, clock drift and model error do
   not. No true simulator wind is supplied to this estimator. Heading and speed
   corrections compensate estimated wind; the air-relative path-angle reference
   is computed before horizontal wind compensation to avoid directly confusing
   ground-track slope and requested air-relative angle.
3. **Combined-command protection.** The old executor separately limited nominal
   and error-feedback components before adding them, which could invalidate the
   final command's dynamic constraints. The new executor clips actuator/bank
   requests, then predicts the resulting held-control motion at **21 points**,
   using four RK4 subdivisions. It checks the existing complete envelope.
4. **Bounded fallback search.** If the request fails, test a local constant-slope,
   constant-bank trim. If verified, run ten bisection trials along the blend from
   trim to request, retaining only numerically feasible tested commands. No
   convexity or globally optimal projection is claimed. If even the fallback is
   unverified, record `control_protection_not_verified` and fail execution
   acceptance. This does not prove that no feasible control exists.

Aircraft states are never clipped or teleported. Simulation continues with
bounded actuators after a failed protection attempt to retain diagnostic data.
**This is not an emergency maneuver or a safe mission-abort policy.** It cannot
restore the coordinated separation guarantee after a tracking-tube violation.

`--tracking-controller legacy` preserves historical feedforward averaging and
feedback for comparisons. Local wind estimation and held-command protection are
specific to coupled 3D mode; the historical planar/independent-Z controller is
not upgraded into a coupled robust controller by this work.

The optional `--execution-backend cpp` compiles this same constrained own-state
update and sampled propagation. Reference sampling, clock/sensor scheduling,
both sides of actuator switches and the independent global observer remain
Python. The controller receives estimated wind from own motion, never the true
simulator disturbance. Protection keeps all sample counts, integration steps,
tolerances and failure rules above. Select it independently of construction's
`--numerical-backend`; local execution defaults to Python while fresh physical
3D planning defaults to C++. C++ execution rejects the legacy
controller and planar/legacy flight modes. See [native execution](native_kernel.md#optional-local-execution-kernel)
for build, identity and paired-replay commands.

## Reproducible disturbances

| Option | Default | Meaning |
|---|---|---|
| `--wind` | 0 m/s | Upper bound on horizontal synthetic velocity disturbance |
| `--wind-model` | `rotating` | `rotating`, `constant` or `gust` |
| `--wind-heading` | 0 degrees | Constant direction or profile direction offset |
| `--clock-offset` | 0 s | Offsets distributed from negative to positive amplitude |
| `--clock-drift` | 0 s/s | Drift distributed from negative to positive amplitude |
| `--position-bias` | 0 m | Fixed position-bias norm with deterministic ID-derived direction |
| `--position-noise` | 0 m | Radius of bounded independent position-noise draws |
| `--disturbance-seed` | 0 | Per-drone noise seed, independent of the planning seed |
| `--tracking-controller` | `constrained` | New executor or historical `legacy` comparison |
| `--execution-backend` | `python` | `cpp` compiles constrained coupled-3D control and simulator integration; observer stays Python |

Rotating wind retains the previous per-ID formula. Constant wind is common to
all aircraft. Gust amplitude follows a bounded sinusoid between zero and the
supplied bound, with a slowly changing direction. Wind is held per 0.2 s
mission-grid interval and added to ground velocity. It is **not** a model of
aerodynamic gust loads, vertical turbulence or lift loss.

Position is measured with constant bias plus bounded uniform-ball noise at each
controller callback. Airspeed, heading, flight-path angle and bank measurements
remain ideal. This is not a GNSS/INS estimator model; it includes neither sensor
latency, dropouts nor attitude/airspeed errors. Actual physical initial states
are unchanged. The observer evaluates true simulated states, not noisy sensors.
Noise streams depend only on disturbance seed and drone ID; membership/order
of other aircraft cannot change an individual stream. Different controller
callback schedules consume samples at different rates.

The bound on a measurement error is `norm(bias) + noise_radius`. It is **not**
automatically added to `position_error`, which remains the allowed total
tracking error used in planning. Sensor error consumes available tracking
authority and may invalidate that tube. Clock uncertainty is still checked as
`abs(offset + drift * true_time) <= clock_error` over the whole mission.

```bash
# Four drones, known obstacles and bounded gust/noisy-position experiment.
.venv/bin/python examples/trajectory_3d.py --num-drones 4 \
  --obstacles examples/scenarios/static_obstacles.json --approach-spread 180 \
  --target 500 500 --horizon 180 \
  --wind 0.1 --wind-model gust --clock-offset 0.03 \
  --position-bias 0.05 --position-noise 0.01 --disturbance-seed 5
```

This example requests an experiment, not a guaranteed admissible wind/noise
envelope for every placement. Exit code **3** indicates any failed execution
criterion, even if all aircraft reach the destination without a sampled collision.

## Diagnostics and acceptance

`report.json` exports controller settings, full disturbance configuration,
measurement-error bounds and per-drone wind estimates, update counts, protection
interventions and unresolved attempts. A successful intervention is not itself
a safety violation; an unresolved attempt prevents acceptance.

`safety_events` records the first observed failure per checked condition and
aircraft/pair, with time and relevant values. Events include physical-envelope
constraints, tracking-tube/clock-bound loss, conservative obstacle/inter-drone
clearance failures and descent-angle requirements. `first_safety_event` gives
the earliest recorded event. Times are observed grid/callback times, not exact
continuous-time crossing solutions. A bound failure need not be an actual
collision. An empty event list alone is not acceptance: `execution_failures`
and `execution_validated` include all arrival, timing and mission checks.
Coupled target contact is now separately localized by reintegrating held
controls to a 1e-6 s event bracket, rather than taking the first observer sample;
other safety-event times retain the grid/callback interpretation above. See
[contact events and mission preparation](mission_pipeline.md).

Physical checking includes both sides of **all** actuator switches, including
switches between ordinary observer samples. Minimum per-drone envelope margins
and maximum absolute vertical acceleration are reported. The existing envelope
thresholds and 1-degree descent-angle tolerance are retained. RK4 and sampled
protection have no certified integration/interval error bound.

The observer pairs each switch state with the incoming and outgoing controls
recorded at that exact hold boundary. A previous observer-grid sample can
precede an intervening local-clock update and therefore carry an older control.
Pairing that older control with a later state produced a reproducible false
vertical-acceleration rejection in a five-aircraft gust trial. The redundant
stale pairing has been removed; all exact boundary records remain checked.
Separate negative regressions inject an unsafe incoming or outgoing command at
a switch between samples and require physical rejection. Replaying the saved
gust trial before/after this observer correction gives bit-identical times,
simulated states and references; only the erroneous acceptance flags change.

## Four-drone validation campaign

```bash
# Six historical small-wind/clock trials, using identical nominal plans.
.venv/bin/python scripts/benchmark_tracking.py --profiles regression \
  --baseline results/benchmarks/robust-tracking-before-20261025.json

# Broader sweep: nominal, rotating/constant wind, gusts plus position errors,
# clock drift, and deliberate violations of the assumed bounds.
.venv/bin/python scripts/benchmark_tracking.py

# Include the configured obstacle scene; four aircraft throughout.
.venv/bin/python scripts/benchmark_tracking.py --scenarios obstacles_180s \
  --seeds 7 --profiles nominal regression gust_sensor clock_drift outside_bounds
```

Each mission is planned once, then exactly the same immutable plans are reused
across profiles/controllers. Optional baseline comparison checks exact plan
signatures. All raw reports, including failures, are retained in a new JSON
file; existing outputs are never overwritten. The `outside_bounds` profile is
an expected rejection, not an acceptable flight. Exit code 2 indicates failed
expectations or a planning/signature mismatch. Failed moderate-wind cases must
remain failures, not be hidden by widening the angle or flight-envelope limits.

The pre-change six-trial baseline used seeds 7/19, horizons 90/150/240 s,
approach spreads 0/180/360 degrees, 0.05 m/s rotating wind and clock offsets
from -0.05 to +0.05 s. Only two of six passed every execution check. Baseline:
`results/benchmarks/robust-tracking-before-20261025.json`.
The final constrained executor passed **all six trials**, with all six exact
nominal plan signatures matching that baseline. Largest sampled tracking error
was 0.60 m, maximum observed absolute vertical acceleration across trials was
0.912 m/s² (unchanged 1 m/s² limit), and worst passage-angle error was 0.830
degrees (unchanged 1-degree tolerance). Final comparison:
`results/benchmarks/robust-tracking-regression-final-20261025.json`.

A separate run used four aircraft, the enlarged obstacle scene, target
`(500,500,30)`, horizon 180 s, 180-degree approaches, 0.1 m/s gust profile,
clock offsets +/-0.03 s, 0.05 m position bias and 0.01 m bounded sensor noise.
Every execution check passed: maximum tracking-error upper bound 0.681 m,
minimum sampled inter-drone distance 50.92 m and minimum obstacle-surface
distance 24.31 m. Artifacts:
`results/trajectory3d/20261025-202112-992477/`.

The retained intermediate campaigns deliberately include residual failures at
0.2 m/s or under combined sensor errors. They are experimental evidence of the
remaining air-angle/trajectory-tube tradeoff, not proof that the new executor
supports all such disturbances. A successful finite campaign likewise does not
prove robustness to every perturbation within a continuous range.
For example, the final air-path compensation still gives a 31.265-degree
passage for drone 2 in the seed-7 default scene under the `constant` 0.2 m/s
profile: this correctly fails the 30 +/- 1 degree requirement. The same
short campaign also retains gust/sensor and drift angle failures:
`results/benchmarks/robust-tracking-air-path-20261025.json`.
