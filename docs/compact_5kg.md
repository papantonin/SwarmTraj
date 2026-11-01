# Hypothetical compact 5 kg fixed-wing profile

[compact_5kg.json](../examples/aircraft/compact_5kg.json) implements the requested
mass, enclosing radius and operating speeds without changing any reference
aircraft, planner constraints or numerical safety assertions.

| Requested quantity | Recorded value |
|---|---|
| Flying mass | 5 kg |
| Enclosing sphere radius | 0.30 m |
| Cruise total airspeed | 100 km/h = 27.77777777777778 m/s |
| Dive total airspeed | 175 km/h = 48.61111111111111 m/s |
| Descending target-passage angle | 30 degrees, set in the companion mission launcher |

The angle is a mission requirement rather than an aerodynamic parameter. The
profile keeps the absolute flight-path envelope distinct (45 degrees). Operating
speeds are used by the launcher's explicit `--phase-speeds` mode: initial loiter
at 25.5 m/s, climb at 27 m/s when needed, transit at 100 km/h, terminal dive at
175 km/h, deceleration and recovery, then 100 km/h cruise. Reference speeds slew
at at most 1.5 m/s²; actual speeds follow integrated thrust/drag dynamics.
Without this flag, `--flight-regime dive` retains the historical constant
175 km/h request. A speed reference alone never establishes actual attainment.

## Assumed airframe and capabilities

All values other than the requested quantities are hypothetical. The profile
is labeled `illustrative`, not `measured`. A radius of 0.30 m allows at most
0.60 m full span about the modeled aircraft center; no large conventional
5 kg airframe is implicitly substituted.

A compact wing with an illustrative 0.56 m span and 0.12 m² planform area has
aspect ratio about 2.61 and mass loading 41.7 kg/m². The span is a rationale
for the chosen polar, not an extra JSON geometry field or a supplied mesh.
For example, a centered elliptical wing of that area/span has about 0.273 m
maximum chord; a short fuselage and thin wing could geometrically fit the
sphere. Actual center of mass, all equipment, propulsor, control surfaces and
deployed airbrakes must still fit. No collision mesh or stability analysis
verifies this enclosure or establishes a flyable real design.

| Assumption | Value and role |
|---|---|
| Constant air density | 1.225 kg/m³ |
| Zero-lift drag CD0 | 0.045 |
| Induced-drag factor k | 0.18, corresponding to efficiency about 0.68 at the illustrative aspect ratio |
| Maximum lift coefficient | 1.4 |
| Maximum thrust | 35 N, a constant-force model limit |
| Maximum additional airbrake drag | 20 N, an idealized constant-force model limit |
| Minimum/maximum permitted total airspeed | 25 / 55.55555555555556 m/s (90 / 200 km/h) |
| Loiter / climb / normal descent setpoints | 25.5 / 27 / 33.333333333333336 m/s |
| Bank / roll rate | 50 degrees / 60 degrees/s |
| Load-factor interval | 0.3–3.5 g |
| Maximum climb / descent rate | 8 / 30 m/s |
| Maximum vertical acceleration | 4 m/s² |

The extra permitted maximum of 200 km/h is a declared simulation assumption,
not a demonstrated structural limit. The nominal 175 km/h dive is below it.
The polar follows the standard `CD = CD0 + k CL²` form; the relationship
`k = 1/(pi AR e)` is described by
[NASA Glenn](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/induced-drag-coefficient/).
The chosen coefficients and efficiency are not NASA measurements for this drone.

The loaded model derives a 1g stall speed of about 21.83 m/s (78.6 km/h),
or 24.45 m/s (88.0 km/h) with the declared 1.12 margin. Bank/load-dependent
stall remains checked, so all bank angles are not necessarily attainable at
the slowest operating speed. The mission uses a 400 m loiter radius explicitly.

In straight level cruise, the chosen polar needs about 10.19 N thrust
(283 W mechanical propulsive power in this model, before efficiency).
At 175 km/h and a 30-degree descending path, vertical speed is 24.31 m/s
downward. Holding that airspeed requires about 14.84 N additional braking;
the declared 20 N airbrake supplies modeled authority. A real aircraft without
this braking capability is not represented by this file. Thrust/drag curves,
energy storage, actuator sizing, inertia and stability remain unmodeled.

## Run the companion mission

The launcher now requests four aircraft with phase-dependent speeds and a 30-degree
descending passage. Initial altitudes are 500, 525, 550 and 575 m; cruise
proposals remain 500–550 m, target `(3000, 0, 200)` m, ceiling 700 m and horizon
420 s. This explicitly longer observation horizon covers slower transit and
recovery; the planner never extends a requested horizon or arrival deadline.
Arrival sides span 180 degrees, with four directions spaced by 60 degrees.
The geometric assignment chooses which aircraft uses each direction; these
horizontal approach directions are distinct from the 30-degree vertical slope.

The launcher now selects `--approach-routing tangent`: a bounded circle-line-circle
reference is constructed from the actual post-loiter pose, then followed by the
same physical controller and independently validated. The historical `pursuit`
mode remains explicit for comparison. This does not claim a globally shortest
path or remove the aircraft's roll/turn constraints. On the matched four-aircraft
175 km/h case, the outer transit durations fell from 124.2/121.8 s to 99.6/98.8 s;
the full-horizon pair lower bound remained 195.04 m. See
[the development record](trajectory_development.md).

The launcher also selects `--selection-policy efficient --efficiency-slack-cap 20`.
Extra robust slack is preferred up to 20 m, then selections are ranked by latest
arrival, summed arrival times and sampled distance. This cap is a soft preference;
mandatory separation and validation limits are unchanged. The selected minimum
extra slack was 58.73 m and the last arrival 134.0 s in the matched case, compared
with 165.78 m and 138.4 s under margin-only selection with tangent routing.

Own-rejection feedback is enabled. After at least two own alignment rejections,
alternating proposals can move the approach join point 5–30 seconds of flight
farther upstream. This is a bounded geometric proposal change, not extra time
or a deadline extension. Individual and joint checks remain unchanged.

```bash
# Editor followed by point-mass playback.
bash examples/scenarios/compact_5kg_30deg.sh

# Same request without windows.
bash examples/scenarios/compact_5kg_30deg.sh --no-gui

# Reproduce the earlier constant-175 km/h mission input and 240 s horizon.
bash examples/scenarios/compact_5kg_30deg.sh --no-gui --no-phase-speeds --horizon 240

# Separate level-cruise experiment at 100 km/h.
bash examples/scenarios/compact_5kg_30deg.sh --no-gui \
  --num-drones 1 --approach-spread 0 \
  --flight-regime cruise --descent-angle 0 --target-altitude 500
```

The script's final arguments allow explicit mission overrides, not physical
overrides of the loaded profile. It does not initialize or configure Gazebo.
An explicitly separate [hypothetical Gazebo build prototype](compact_gazebo_prototype.md)
is available; its force model and PX4 flight remain unqualified.

The native local pull-out primitive loses about 81.9 m over 6.6 s for this
profile at 175 km/h and 30 degrees. The 200 m target leaves room for recovery;
no altitude floor or arrival requirement was relaxed. A 2 m target and a
zero-airbrake variant are deliberately rejected under the same model.

## Verification and limits

The four-aircraft operating-speed trial passed full-horizon native planning,
independent Python replay and simulated execution. All four aircraft attained
cruise before contact, the declared terminal speed at contact, and recovered
cruise. Actual contacts occurred at 182.08, 107.49, 203.68 and 91.93 s; the
execution pair lower bound was 205.04 m. These timings cannot be interpreted
as an algorithm-only speedup against constant-175 km/h trials: initial speeds
and the requested horizon differ. Strict single-level profiles are currently
required; combining operating speeds with smooth or mixed profile modes is
rejected explicitly. Shared PX4 scenario export also rejects this mode until
its schema and native aircraft support it.

The headless cruise and dive missions both passed the point-mass execution
observer checks. The observed dive passage angle was approximately 30 degrees.
The focused profile/operating-speed/heterogeneous-aircraft run passed 110 tests,
including independent Python validation of the integrated native dive plan,
native held-command checks and the two deliberate negative requests.
Local evidence is under `results/compact-5kg-qualification/` (ignored by Git).

The original four-aircraft 180-degree request exhausted its finite catalog on
alignment checks, including with 180 attempts and several seeds/assignments.
Those failures are retained. With bounded longer-approach feedback, the same
physical settings, horizon and 180-degree spread produced four accepted plans
per aircraft within the original 24-attempt budget; point-mass execution passed.
The regression also revalidates each selected plan independently in Python and
compares serial/spawn plans exactly. The final integrated suite passed 1,057
tests, and all six standalone C++ tests passed after the native feedback change.

These are numerical consistency checks for this hypothetical configuration.
They do not demonstrate a real 5 kg aircraft fitting this sphere, validate a
whole high-speed mission family, certify an RK4 error bound, or transfer to
PX4/Gazebo. See the [profile contract](aircraft_profiles.md) and
[flight model](flight_model_3d.md).
