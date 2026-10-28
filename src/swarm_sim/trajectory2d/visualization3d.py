"""3D replay with altitude, separation and spatial tracking diagnostics."""

import numpy as np

from .dynamics import holding_circle
from .execution import positions


def visualize(result, config, evaluation, output, gui=True, playback_speed=4.0):
    import matplotlib
    if not gui:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    times = evaluation.times
    colors = plt.cm.tab10(np.linspace(0, 0.9, len(result.plans)))
    figure = plt.figure(figsize=(15, 10), layout="constrained")
    grid = figure.add_gridspec(5 if config.coupled_flight else 4, 2, width_ratios=[1.7, 1],
                              height_ratios=[1, 1, 1, 1, 2] if config.coupled_flight else [1, 1, 1, 2])
    view = figure.add_subplot(grid[:, 0], projection="3d")
    separation = figure.add_subplot(grid[0, 1])
    altitude = figure.add_subplot(grid[1, 1])
    errors = figure.add_subplot(grid[2, 1])
    flight_axis = figure.add_subplot(grid[3, 1]) if config.coupled_flight else None
    info = figure.add_subplot(grid[4 if config.coupled_flight else 3, 1])
    info.axis("off")
    mode = "separate arrival altitudes" if config.target_altitude is None else f"common target altitude: {config.target_altitude:g} m"
    view.set(title=f"3D trajectories — {mode}", xlabel="x (m)", ylabel="y (m)", zlabel="Altitude (m)")
    view.view_init(elev=25, azim=-60)
    from .obstacle_view import draw_obstacles_3d
    draw_obstacles_3d(view, config)
    view_end = min(config.horizon, max(p.arrival_time for p in result.plans) + 6)
    visible = times <= view_end
    xyz = positions(evaluation.nominal)
    for k, (plan, color) in enumerate(zip(result.plans, colors)):
        cruise_text = ("levels " + "/".join(f"{z:.1f}" for z in plan.cruise_levels_m)
                       if plan.cruise_levels_m else f"cruise {plan.cruise_altitude:.1f}")
        view.plot(*xyz[visible, k].T, "--", color=color, alpha=0.5,
                  label=f"Drone {plan.drone_id} · {cruise_text} m · {plan.arrival_time:.1f} s")
        circle = holding_circle(plan.states[0], plan.speed)
        if circle is not None:
            centre, radius = circle
            angles = np.linspace(0, 2 * np.pi, 100)
            view.plot(centre[0] + radius * np.cos(angles), centre[1] + radius * np.sin(angles),
                      np.full_like(angles, plan.vertical_states[0, 0]), ":", color=color)
        arrival_z = plan.sample(plan.arrival_time)[4]
        view.scatter(*config.target, arrival_z, color="gold", edgecolor="black", marker="X", s=70)
        if plan.approach_angle is not None:
            direction = np.array([np.cos(plan.approach_angle), np.sin(plan.approach_angle)])
            start = np.asarray(config.target) + 40 * direction
            view.quiver(*start, arrival_z, *(-direction), 0, length=30, color=color,
                        arrow_length_ratio=0.2)
        altitude.plot(times, evaluation.actual[:, k, 4], color=color, label=f"Drone {plan.drone_id}")
        errors.plot(times, evaluation.tracking_errors[:, k], color=color)
        if flight_axis is not None:
            flight_axis.plot(times, evaluation.flight_actual[:, k, 3], color=color, label=f"V air {plan.drone_id}")
            if config.aircraft_profiles:
                local = config.for_drone(plan.drone_id)
                flight_axis.axhline(local.limits.min_speed, color=color, ls=':', alpha=.5)
                flight_axis.axhline(local.limits.max_speed, color=color, ls=':', alpha=.5)
    if config.target_altitude is None:
        view.plot([config.target[0]] * 2, [config.target[1]] * 2,
                  [float(xyz[..., 2].min()), float(xyz[..., 2].max())], "k:", alpha=0.5)
    else:
        u, v = np.meshgrid(np.linspace(0, 2 * np.pi, 20), np.linspace(0, np.pi, 12))
        r = config.target_radius
        view.plot_wireframe(config.target[0] + r * np.cos(u) * np.sin(v),
                            config.target[1] + r * np.sin(u) * np.sin(v),
                            config.target_altitude + r * np.cos(v), color="gold", alpha=0.4)
    ranges = np.ptp(xyz[visible].reshape(-1, 3), axis=0)
    aspect = np.maximum(ranges, 20)
    aspect[2] = max(aspect[2], 0.35 * max(aspect[:2]))
    view.set_box_aspect(aspect, zoom=0.85)
    view.zaxis.set_major_locator(MaxNLocator(nbins=5))
    view.legend(fontsize=8, loc="upper left")
    if config.obstacles:
        # Use known finite scene bounds rather than collection/quiver autolim
        # state, which can produce invalid ranges with mixed 3D artists.
        points = [xyz[visible].reshape(-1, 3)]
        for obstacle in config.obstacles:
            points.extend([np.array(obstacle.center)[None] - obstacle.half_extents,
                           np.array(obstacle.center)[None] + obstacle.half_extents])
        scene = np.concatenate(points)
        low, high = scene.min(axis=0), scene.max(axis=0)
        margin = np.maximum((high - low) * 0.05, 2)
        view.set_xlim(low[0] - margin[0], high[0] + margin[0])
        view.set_ylim(low[1] - margin[1], high[1] + margin[1])
        view.set_zlim(low[2] - margin[2], high[2] + margin[2])
    view.set_autoscale_on(False)
    if len(result.plans) > 1:
        separation.plot(times, evaluation.distances, color="navy")
    radii = [config.for_drone(p.drone_id).aircraft_radius for p in result.plans]
    maximum_center_requirement = config.separation + sum(sorted(radii, reverse=True)[:2])
    separation.axhline(maximum_center_requirement, color="red", ls="--", label="Minimum center distance (largest pair)")
    separation.set(title="Minimum 3D inter-aircraft distance", xlabel="Time (s)", ylabel="m")
    separation.legend(fontsize=8)
    altitude.set(title="Executed altitudes", xlabel="Time (s)", ylabel="m")
    altitude.legend(fontsize=7, ncols=2)
    if config.cruise_altitude_range is not None:
        altitude.axhspan(*config.cruise_altitude_range, color="green", alpha=0.07)
    if config.target_altitude is not None:
        altitude.axhline(config.target_altitude, color="black", ls=":")
    errors.axhline(config.position_error, color="red", ls="--")
    errors.set(title="3D spatial tracking error", xlabel="Time (s)", ylabel="m")
    preparing = "preparation" in result.diagnostics
    if preparing:
        for axis in (separation, altitude, errors, *([flight_axis] if flight_axis is not None else [])):
            axis.axvspan(0, config.activation_time, color="gold", alpha=.12)
            axis.axvline(config.activation_time, color="darkgoldenrod", ls=":")
    if flight_axis is not None:
        if not config.aircraft_profiles:
            flight_axis.axhline(config.limits.min_speed, color="red", ls=":")
            flight_axis.axhline(config.limits.max_speed, color="red", ls=":")
        flight_axis.set(title="Total airspeed", xlabel="Time (s)", ylabel="m/s")
    report = evaluation.report
    lower = result.minimum_distance_lower_bound
    cruise_label = (f"auto {config.cruise_altitude_range[0]:g}–{config.cruise_altitude_range[1]:g} m"
                    if config.cruise_altitude_range is not None else "manual")
    info.text(0, 1, f"Numerical validation · horizon {config.horizon:g} s\n"
              f"Loiter: {config.min_extra_loiter:g}–{config.max_extra_loiter:g} s · approach spread {config.approach_spread_deg:g}°\n"
              f"Cruise: {cruise_label} · descent: {config.descent_angle_deg:g}°\n"
              f"Nominal lower bound: {lower:.2f} m\n" if lower is not None else "Single-aircraft mission\n",
              va="top", fontsize=8.5, linespacing=1.05)
    model_label = (f"3D point mass · envelope verified: {report['physical_limits_verified']}"
                   if config.coupled_flight else "2.5D kinematic model")
    vertical_label = ('Per-aircraft physical limits (see report).'
                      if config.aircraft_profiles else
                      f'Vz: +{config.max_climb_rate:g} / −{config.descent_rate_limit:g} m/s; accel. ≤ {config.max_vertical_acceleration:g} m/s²')
    info.annotate(f"Execution validated: {report['execution_validated']} · alerts: {len(report['safety_events'])}",
              (0, 1), xycoords="axes fraction", xytext=(0, -42), textcoords="offset points",
              va="top", fontsize=8.5, color="darkgreen" if report['execution_validated'] else "firebrick")
    info.annotate(f"All arrived: {report['all_arrived']} · tubes: {report['tube_conditions_verified']}\n"
              f"Vertical limits: {report['vertical_limits_verified']}\n"
              f"{vertical_label}\n"
              f"{model_label} · no terrain mesh.\n"
              f"Obstacles: {len(config.obstacles)} · clearance verified: {report['obstacle_clearance_verified_in_execution_model']}\n"
              "Vertical scale exaggerated for readability.\n"
              "Drag the mouse to rotate the view.",
              (0, 1), xycoords="axes fraction", xytext=(0, -53), textcoords="offset points",
              va="top", fontsize=8.5, linespacing=1.05)
    lines = [view.plot([], [], [], color=color, lw=1.8)[0] for color in colors]
    markers = [view.plot([], [], [], "o", color=color, ms=7)[0] for color in colors]
    headings = [view.plot([], [], [], color=color, lw=2.5)[0] for color in colors]
    cursors = [axis.axvline(0, color="black", alpha=0.5) for axis in
               ((separation, altitude, errors, flight_axis) if flight_axis is not None else (separation, altitude, errors))]
    title = figure.suptitle("3D autonomous execution")

    def update(t):
        index = min(int(np.searchsorted(times, t)), len(times) - 1)
        for k, (line, marker, heading) in enumerate(zip(lines, markers, headings)):
            path = positions(evaluation.actual[:index + 1, k])
            line.set_data_3d(*path.T)
            point = path[-1]
            marker.set_data_3d(*point[:, None])
            state = evaluation.actual[index, k]
            if config.coupled_flight:
                from .flight3d import air_velocity
                velocity = air_velocity(evaluation.flight_actual[index, k])
                direction = 12 * velocity / np.linalg.norm(velocity)
            else:
                direction = 12 * np.array([np.cos(state[2]), np.sin(state[2]), state[5] / result.plans[k].speed])
            heading.set_data_3d(*np.stack((point, point + direction)).T)
        for cursor in cursors:
            cursor.set_xdata([t, t])
        phase = "In-flight preparation" if preparing and t < config.activation_time else "3D autonomous execution"
        title.set_text(f"{phase} — t = {t:.1f} s / {config.horizon:g} s")
        figure.canvas.draw_idle()

    update(view_end)
    figure.savefig(output, dpi=150)
    if gui:
        from matplotlib.animation import FuncAnimation
        from matplotlib.widgets import Button, Slider
        figure.set_layout_engine(None)
        figure.subplots_adjust(bottom=0.15, top=0.9, hspace=0.65)
        title.set_position((.5, .98))
        slider = Slider(figure.add_axes([0.20, 0.035, 0.62, 0.03]), "Time", 0, config.horizon, valinit=0)
        button = Button(figure.add_axes([0.04, 0.025, 0.11, 0.045]), "Pause / Play")
        playing = [True]
        button.on_clicked(lambda event: playing.__setitem__(0, not playing[0]))
        slider.on_changed(update)

        def frame(_):
            if playing[0]:
                next_time = min(slider.val + playback_speed / 30, config.horizon)
                slider.set_val(next_time)
                if next_time >= config.horizon:
                    playing[0] = False

        animation = FuncAnimation(figure, frame, interval=1000 / 30, cache_frame_data=False)
        update(0)
        plt.show()
        del animation
    plt.close(figure)
