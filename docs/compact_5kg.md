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
speeds do not automatically switch between phases: choosing `dive` currently
uses that setpoint for the whole mission. This is not a 100-to-175 km/h transition
experiment.

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

The launcher now requests four aircraft at the dive setpoint and a 30-degree
descending passage. Initial altitudes are 500, 525, 550 and 575 m; cruise
proposals remain 500–550 m, target `(3000, 0, 200)` m, ceiling 700 m and horizon
240 s. Arrival sides span 180 degrees, with four directions spaced by 60 degrees.
The geometric assignment chooses which aircraft uses each direction; these
horizontal approach directions are distinct from the 30-degree vertical slope.

Own-rejection feedback is enabled. After at least two own alignment rejections,
alternating proposals can move the approach join point 5–30 seconds of flight
farther upstream. This is a bounded geometric proposal change, not extra time
or a deadline extension. Individual and joint checks remain unchanged.

```bash
# Editor followed by point-mass playback.
bash examples/scenarios/compact_5kg_30deg.sh

# Same request without windows.
bash examples/scenarios/compact_5kg_30deg.sh --no-gui

# Separate level-cruise experiment at 100 km/h.
bash examples/scenarios/compact_5kg_30deg.sh --no-gui \
  --num-drones 1 --approach-spread 0 \
  --flight-regime cruise --descent-angle 0 --target-altitude 500
```

The script's final arguments allow explicit mission overrides, not physical
overrides of the loaded profile. It does not initialize or configure Gazebo.

The native local pull-out primitive loses about 81.9 m over 6.6 s for this
profile at 175 km/h and 30 degrees. The 200 m target leaves room for recovery;
no altitude floor or arrival requirement was relaxed. A 2 m target and a
zero-airbrake variant are deliberately rejected under the same model.

## Verification and limits

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
