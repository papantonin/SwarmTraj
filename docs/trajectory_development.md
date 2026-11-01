# Ordered trajectory development

This record distinguishes changes to proposal geometry, selection objectives
and the requested speed history. Evidence is local under
`results/development-20261101/`; generated experiments are ignored by Git.

## Stable checkpoint

Commit `7e5bf6f` preserves physical profiles v3, operating-speed selection,
the hypothetical compact 5 kg airframe, four-aircraft 180-degree arrivals and
bounded alignment-rejection feedback. Its integrated suite passed 1,057 tests
and six standalone C++ tests. It requests 175 km/h throughout the dive mission;
it does not switch automatically from the separately declared cruise speed.

## 1. Curvature-bounded approach references

The explicit `approach_routing=tangent` option adds a standalone C++ planar
reference builder. It selects the shortest of four forward circle-line-circle
connections, with radius derived conservatively from speed, bank and stall/load
authority. It is a bounded proposal family, not a global shortest-path solver.
The reference starts at the actual integrated post-loiter/profile state. Heading
and cross-track feedback include nominal curvature feedforward. The physical
integrator retains roll limits and independently checks the resulting flight.

`pursuit` retains the original proposal behavior for comparison and archived
inputs. Shared PX4 editor schemas currently reject tangent-mode export instead
of silently dropping the unsupported field. No clearance, arrival, recovery,
clock, envelope or tracking limit is weakened, and references are never treated
as accepted trajectories. Obstacles still require integrated clearance checks.

| Matched 175 km/h case | Pursuit | Tangent |
|---|---:|---:|
| Drone 0 transit | 124.2 s | 99.6 s |
| Drone 2 transit | 121.8 s | 98.8 s |
| Full-horizon pair lower bound | 195.04 m | 195.04 m |

Other aircraft do not necessarily improve individually under the existing
margin-only selection objective. Seven standalone C++ tests and 60 focused
Python checks passed, including independent selected-plan replay, point-mass
execution and exact serial/spawn plan agreement. The before/after JSON missions
are retained. These measurements describe this scenario, not guaranteed gains.

## 2. Duration/distance ranking after preferred slack

The explicit `selection_policy=efficient` ranks compatible selections by:

1. Minimum extra robust pair slack, capped at `efficiency_slack_cap_m`.
2. Earliest latest arrival (makespan).
3. Smallest sum of arrival times.
4. Smallest sampled 3D travel distance up to each arrival, including initial loiter.

The default cap is 20 m. It is a soft ranking preference, not a new safety bound
or permission to change required separation. If no selection reaches the cap,
the best available minimum slack still ranks first and the result records that
the cap was not reached. Independent acceptance checks and margins are unchanged.
`max_margin` and `first` retain their original behavior. The compact launcher
explicitly selects the new policy; other defaults remain unchanged.

C++ caches own-candidate costs and uses admissible optimistic bounds in the
bounded search. An independent exhaustive Python oracle checks small catalogs.
The shared search-node/refinement budgets still apply, including to a single
aircraft; incomplete optimization preserves a verified incumbent. Serial/spawn
plan equality and full point-mass execution are checked separately.

On the matched tangent-routing case, the last arrival moved from 138.4 to
134.0 s. The minimum extra robust slack decreased from 165.78 to 58.73 m,
remaining above the selected 20 m ranking cap. This is an explicit tradeoff
between preferred surplus slack and duration, not a safety-margin relaxation.
The result is optimal only within the explored finite catalog when budgets
permit, not in continuous trajectory space.

Native planning/catalog/search capabilities are version 2; stale extensions
fail with rebuild instructions rather than silently ignoring the new options.
The integrated suite passed 1,075 tests and all seven standalone C++ tests.

## 3. Integrated operating-speed schedule

The explicit `--phase-speeds` option binds each own profile's loiter, climb,
cruise, descent and dive airspeeds. The terminal regime is selected separately.
Native proposals slew the reference through physical thrust/braking commands;
they never reset airspeed. The compact launcher uses a 1.5 m/s² reference slew
and an explicit 420 s horizon. It starts at 25.5 m/s, transits at 100 km/h,
accelerates upstream of descent to 175 km/h, then decelerates, climbs and
recovers 100 km/h cruise. Its initial state and horizon differ from the earlier
constant-speed case, so the elapsed mission times are not a pure optimization
comparison.

Native and independently written Python validators require a rate-bounded
reference, actual cruise held before arrival, actual terminal attainment and
terminal speed at reintegrated contact, plus recovered final cruise speed.
Execution adds observer-only actual-speed acceptance checks. Single-level
strict profiles are supported; smooth/mixed overlays and lossy shared PX4
exports are explicitly rejected. Existing constant-speed behavior remains
unchanged; all four prior selected candidates, state arrays and control arrays
matched exactly in the replayed 240 s constant-speed mission.

The four-aircraft operating trial passed execution with actual contact times
182.08, 107.49, 203.68 and 91.93 s and an execution pair lower bound of
205.04 m. Numerical limits and tracking/clock assumptions are unchanged.
Native planning/catalog capabilities are now version 3, validation version 2
and search version 2; older binaries fail with rebuild instructions.
The integrated suite passed 1,087 tests (four existing Gymnasium float32
warnings), and all seven standalone native tests passed. Negative cases cover
missing or corrupted speed references, unattained terminal regimes, wrong
initial airspeed, insufficient descent authority, low targets and absent brakes.

## 4. Numerical and mission qualification

Run the incremental qualification with:

```bash
.venv/bin/python scripts/qualify_operating_speeds.py \
  --output results/operating-qualification-NEW
```

The recorded 12 requests all passed: 1 and 4 aircraft, seeds 7 and 19,
control steps 0.2, 0.1 and 0.05 s, 420 s common horizon, four candidates and
24 attempts. Each selected native mission was independently replayed in Python
and accepted by the full execution observer, including attained phase speeds.
Source hashes, binary identity, hardware, settings, failures and signatures are
retained in the incremental report. An incomplete journal never reports that
the full campaign passed. This small campaign is not broad aircraft qualification.

On an Intel Core i3-1115G4, serial native planning measured 0.060–0.290 s for
one aircraft and 0.431–1.166 s for four across these different control steps.
The process peak RSS was about 334 MiB, including Python independent audits,
execution and accumulated trials; this is not isolated native-kernel memory or
a Python/C++ speedup measurement. Clock timing remains outside mission identity.

Reintegrating each same held-command interval with eight rather than two RK4
substeps differed by at most 7.0e-7 m per position coordinate and 7.7e-8 m/s
in airspeed in these fixtures. These are local interval comparisons, not a
certified global integration-error bound. Changing the control step also changes
feedback, catalog acceptance and selection: for four aircraft at seed 19,
drone 3 contact moved from 87.90 to 94.57 to 94.75 s as the step decreased.
These independently valid selected trajectories are not identical commands
under numerical refinement. Failed low-target, no-brake and insufficient
descent-authority requests remain explicit regression negatives, not successes.

## 5. Separate hypothetical Gazebo prototype

The user selected a hypothetical simulation prototype rather than supplying a
native measured airframe. The [build prototype](compact_gazebo_prototype.md)
exports authored compact geometry, exact 5 kg component accounting, primitive
0.30 m enclosure bounds, assumed inertias, inherited sensor/servo conventions
and explicit aerodynamic assumptions. SDF validation and a 250-step standalone
Gazebo load passed in the pinned isolated image. The existing Cessna adapter is
unchanged; the prototype is deliberately unavailable as a production aircraft.

An explicit [C++ force component](compact_force_model.md) now matches the
profile's polar and ideal 20 N brake under manufactured coordinated loads.
Twenty-four analytical fixtures and four actual Gazebo force/acceleration
fixtures passed. It removes inherited LiftDrag forces to avoid double counting;
the default earlier fixture is preserved. Static fractions are bench inputs,
not PX4 actuator bindings. The full suite passed 1,098 tests.

This stage remains incomplete for native flight: aerodynamic moments,
PX4 actuator binding, trim/stability,
controller tuning and actual missions remain unqualified. Those are concrete
next gates before an explicit native adapter and single-aircraft/fleet trials.
No point-mass timing, tracking or separation claim transfers to this prototype.
