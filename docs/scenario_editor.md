# Mission preparation editor

The Python preparation window opens in **Mission** view. **Advanced** contains
the detailed mission and planner settings. Both views edit the same scenario;
switching views preserves configuration, pending input, selection and map zoom.

## Mission view

1. Select a drone, **Target**, or an obstacle in the list on the right.
2. Click the map to move it, or enter its coordinates.
3. Use **Apply** to update the scene, then **Launch mission** to continue.
   The Gazebo launcher labels the final action **Calculate → Gazebo**.

The fields follow the selection:

| Selected item | Visible fields |
|---|---|
| Aircraft in 3D | X, Y, **Altitude (m)**, **Heading (°)**, **Airspeed (m/s)** |
| Shared-altitude target | X, Y, **Altitude (m)** |
| Target without shared altitude | X and Y; the helper explains the altitude policy |
| Cylinder | XYZ centre, **Radius (m)** and **Half-height (m)** |
| Box | XYZ centre, **Half-width X (m)**, **Half-width Y (m)** and **Half-height (m)** |

Planar mode omits altitude. The aircraft point is its actual initial position;
the dotted circle is its initial holding pattern. Compact labels use `D` for
drone and `O` for obstacle. Aircraft altitude remains visible on the larger
Mission map; the Advanced context map uses IDs only. The selected aircraft has
a black marker outline. **Fit all** changes only the map viewport.

**+ Cylinder** and **+ Box** add an obstacle; **Remove obstacle** is visible
when one is selected. Sizes remain half-dimensions: a box's complete width,
depth and height are twice those values. Cylinder radius is not doubled.

The read-only summary below the map retains the horizon, initial loiter,
departure-delay bounds, explicit arrival deadline, minimum post-arrival time,
cruise policy, descent angle, approach spread, separation and target radius.
It displays pending text as entered; **Apply** or launch validates and applies
that text. Merely hiding an input does not disable its constraint.

## Advanced view

| Group | Settings |
|---|---|
| Mission timing | **Horizon (s)**, **Arrival deadline (s)**, **After arrival (s)**, **Initial loiter (s)** and extra wait bounds |
| Flight references | **Automatic cruise**, cruise bounds, **Approach spread (°)** and **Descent (°)** |
| Selected aircraft | **Cruise (m)**, **Loiter radius (m)** and **Counterclockwise** |
| Target policy | **Target: common altitude** |
| Numerical planner | Fixed physical C++; no backend/generator switch |
| Aircraft profile | **Profile JSON** and **Load profile**, where profile loading is supported |

Own-aircraft physical limits and profile identity remain visible in this view.
The PX4/Gazebo editor hides profile-loading controls because its pinned native
airframe is fixed. Production uses fixed physical C++. Archived incompatible settings remain
visible for inspection with launch disabled; use an explicit migration to a new
file. See the [production/backend policy](planner_defaults.md).

**Reset** restores the original scenario, input values and numerical choices,
and returns to Mission view. View changes do not apply, clear or repair input.
Launch still validates all fields, including hidden Advanced fields, before
the existing full planning and adapter validation. A failed check leaves the
window open with the original error message. No deadline, physical limit,
margin or search budget is relaxed to simplify the interface.

## Launch commands

```bash
# Point-mass preparation, planning and playback:
.venv/bin/python examples/trajectory_3d.py
# Four-aircraft preparation followed by Gazebo after acceptance:
.venv/bin/python examples/px4_gazebo.py --num-drones 4 \
  --scenario examples/scenarios/px4_four_loiter_cpp.json \
  --edit-scenario --run --gui --timeout 1200
```

The underlying scenario schemas and command-line options are unchanged. View
state is presentation only and is not added to saved scenario JSON. See
[3D trajectories](trajectory_3d.md), [shared Gazebo editor](gazebo_editor.md)
and [fleet experiment](gazebo_fleet.md) for their separate execution scopes.
