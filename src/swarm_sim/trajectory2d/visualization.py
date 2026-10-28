"""Matplotlib replay; this view is an observer, not a controller input."""

import numpy as np

from .dynamics import holding_circle


def _draw_holding_circle(axis, state, speed, drone_id, color):
    import matplotlib.pyplot as plt

    circle = holding_circle(state, speed)
    if circle is None:
        return
    centre, radius = circle
    axis.add_patch(plt.Circle(centre, radius, fill=False, color=color,
                              linestyle=":", linewidth=1.5, alpha=0.8))
    axis.plot(*centre, "+", color=color, ms=6)
    axis.annotate(f"Loiter {drone_id}", centre, xytext=(4, 5),
                  textcoords="offset points", fontsize=8, color=color)


def visualize(result, config, evaluation, output, gui=True, playback_speed=4.0):
    if config.dimensions == 3:
        from .visualization3d import visualize as visualize3d
        return visualize3d(result, config, evaluation, output, gui, playback_speed)
    import matplotlib
    if not gui:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = evaluation.times
    colors = plt.cm.tab10(np.linspace(0, 0.9, len(result.plans)))
    figure = plt.figure(figsize=(13, 8), layout="constrained")
    grid = figure.add_gridspec(3, 2, width_ratios=[1.7, 1])
    map_axis = figure.add_subplot(grid[:, 0])
    separation_axis = figure.add_subplot(grid[0, 1])
    error_axis = figure.add_subplot(grid[1, 1])
    text_axis = figure.add_subplot(grid[2, 1])
    text_axis.axis("off")
    map_axis.set(title="2D trajectories — destination passage and onward flight", xlabel="x (m)", ylabel="y (m)")
    map_axis.set_aspect("equal", adjustable="box")
    map_axis.grid(alpha=0.25)
    from .obstacle_view import draw_obstacles_2d
    draw_obstacles_2d(map_axis, config)
    map_axis.add_patch(plt.Circle(config.target, config.target_radius, color="gold", alpha=0.35))
    map_axis.plot(*config.target, "kx", ms=10, label="Shared destination")
    view_end = min(config.horizon, max(p.arrival_time for p in result.plans) + 6)
    visible = times <= view_end
    for k, (plan, color) in enumerate(zip(result.plans, colors)):
        _draw_holding_circle(map_axis, plan.states[0], plan.speed, plan.drone_id, color)
        path = evaluation.nominal[visible, k, :2]
        map_axis.plot(*path.T, "--", color=color, alpha=0.6, lw=1,
                      label=f"Drone {plan.drone_id} · {plan.speed:g} m/s · arrival {plan.arrival_time:.1f} s")
        if plan.approach_angle is not None:
            direction = np.array([np.cos(plan.approach_angle), np.sin(plan.approach_angle)])
            start = np.asarray(config.target) + 45 * direction
            end = np.asarray(config.target) + config.target_radius * direction
            map_axis.annotate("", xy=end, xytext=start,
                              arrowprops={"arrowstyle": "->", "color": color, "lw": 1.8})
    map_axis.legend(fontsize=8, loc="best")
    map_axis.autoscale_view()
    map_axis.set_autoscale_on(False)
    # Full-horizon paths remain in the exported data and were jointly checked.
    if len(result.plans) > 1:
        separation_axis.plot(times, evaluation.distances, color="navy")
        separation_axis.axhline(config.separation, color="red", ls="--", label="Required physical separation")
        separation_axis.legend(fontsize=7)
    separation_axis.set(title="Minimum inter-aircraft distance", xlabel="Time (s)", ylabel="m")
    for k, color in enumerate(colors):
        error_axis.plot(times, evaluation.tracking_errors[:, k], color=color)
    error_axis.axhline(config.position_error, color="red", ls="--")
    error_axis.set(title="Tracking error at local time", xlabel="Time (s)", ylabel="m")
    report = evaluation.report
    lower = result.minimum_distance_lower_bound
    text_axis.text(0, 1, "Plan validated under stated assumptions\n"
                   f"Seed: {config.seed}   |   Activation: {config.activation_time:g} s\n"
                   f"Approach spread: {config.approach_spread_deg:g}°\n"
                   f"Extra wait: {config.min_extra_loiter:g}–{config.max_extra_loiter:g} s\n"
                   f"Nominal lower bound: {lower:.2f} m\n" if lower is not None else "Single-aircraft mission\n",
                   va="top", fontsize=10)
    text_axis.text(0, 0.57,
                   f"All arrived: {report['all_arrived']}\n"
                   f"Approach sectors satisfied: {report['approach_sectors_verified_in_execution']}\n"
                   f"Tracking tubes verified in this trial: {report['tube_conditions_verified']}\n"
                   f"Maximum clock error: {report['max_clock_error_s']:.3f} s\n"
                   f"Static obstacles: {len(config.obstacles)} · clearance verified: {report['obstacle_clearance_verified_in_execution_model']}\n"
                   "After arrival: aircraft continue flying and remain present.\n"
                   "The map frames the approaches; zoom and pan are available.",
                   va="top", fontsize=9)
    actual_lines = [map_axis.plot([], [], color=color, lw=1.6)[0] for color in colors]
    arrows = map_axis.quiver(np.zeros(len(colors)), np.zeros(len(colors)),
                            np.ones(len(colors)), np.zeros(len(colors)), color=colors,
                            angles="xy", scale_units="xy", scale=0.09, width=0.006, zorder=5)
    envelopes = []
    for plan, color in zip(result.plans, colors):
        radius = config.separation / 2 + config.position_error + plan.speed * config.clock_error
        circle = plt.Circle(plan.states[0, :2], radius, color=color, alpha=0.12)
        map_axis.add_patch(circle)
        envelopes.append(circle)
    cursors = [axis.axvline(0, color="black", alpha=0.6) for axis in (separation_axis, error_axis)]
    title = figure.suptitle("Simulated autonomous execution", y=0.98)

    def update(t):
        index = min(int(np.searchsorted(times, t)), len(times) - 1)
        for k, (line, envelope) in enumerate(zip(actual_lines, envelopes)):
            line.set_data(evaluation.actual[:index + 1, k, 0], evaluation.actual[:index + 1, k, 1])
            envelope.center = evaluation.nominal[index, k, :2]
        arrows.set_offsets(evaluation.actual[index, :, :2])
        arrows.set_UVC(np.cos(evaluation.actual[index, :, 2]), np.sin(evaluation.actual[index, :, 2]))
        for cursor in cursors:
            cursor.set_xdata([t, t])
        title.set_text(f"Simulated autonomous execution — t = {t:.1f} s / {config.horizon:g} s")
        figure.canvas.draw_idle()

    update(view_end)
    figure.savefig(output, dpi=150)
    if gui:
        from matplotlib.animation import FuncAnimation
        from matplotlib.widgets import Button, Slider

        figure.set_layout_engine(None)
        figure.subplots_adjust(bottom=0.15)
        slider = Slider(figure.add_axes([0.19, 0.035, 0.64, 0.03]), "Time", 0, config.horizon, valinit=0)
        button = Button(figure.add_axes([0.04, 0.025, 0.10, 0.045]), "Pause / Play")
        playing = [True]
        button.on_clicked(lambda event: playing.__setitem__(0, not playing[0]))
        slider.on_changed(update)

        def frame(_):
            if playing[0]:
                next_time = slider.val + playback_speed / 30
                slider.set_val(min(next_time, config.horizon))
                if next_time >= config.horizon:
                    playing[0] = False

        animation = FuncAnimation(figure, frame, interval=1000 / 30, cache_frame_data=False)
        update(0)
        plt.show()
        # Keep animation and widgets alive until the window closes.
        del animation
    plt.close(figure)


def pick_target(snapshots, default):
    """Choose a destination on a local XY map of the frozen snapshot."""
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(9, 7))
    positions = np.array([s.state[:2] for s in snapshots])
    headings = np.array([s.state[2] for s in snapshots])
    colors = plt.cm.tab10(np.linspace(0, 0.9, len(snapshots)))
    axis.quiver(*positions.T, np.cos(headings), np.sin(headings), color=colors)
    for s, color in zip(snapshots, colors):
        _draw_holding_circle(axis, s.state, s.speed, s.drone_id, color)
        axis.plot(*s.state[:2], ".", color=color)
        axis.annotate(f"Drone {s.drone_id}", s.state[:2])
    axis.plot(*default, "kx", label="Default destination")
    axis.margins(0.25)
    axis.set(xlabel="x (m)", ylabel="y (m)",
             title="Aircraft snapshot at t=0 — click once to select the destination")
    axis.set_aspect("equal")
    axis.grid(alpha=.3)
    selected = plt.ginput(1, timeout=-1)
    plt.close(figure)
    if not selected:
        raise ValueError("destination selection cancelled")
    return tuple(selected[0])
