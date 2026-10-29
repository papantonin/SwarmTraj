"""Interactive placement of actual initial aircraft positions and destination."""

from dataclasses import replace

import numpy as np

from .dynamics import holding_circle, wrap
from .planning import approach_angles
from .vertical import descent_error


class ScenarioEditor:
    def __init__(self, snapshots, target, config, launch_validator=None, profile_loading=True):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, CheckButtons, RadioButtons
        from .editor_widgets import DeferredTextBox as TextBox

        self.original = tuple(snapshots), tuple(target)
        self.snapshots = list(snapshots)
        self.target = tuple(target)
        self.config = config
        self.original_config = config
        self.accepted = False
        self.launch_validator = launch_validator
        self.profile_loading = profile_loading
        self.selected = 0
        self.figure = plt.figure(figsize=(14, 9))
        self.axis = self.figure.add_axes([0.08, 0.43, 0.64, 0.47])
        self.figure.suptitle("Prepare the mission — position aircraft and target")
        self.figure.text(0.08, 0.94,
                         "Select an item on the right, then click the map or enter its coordinates.",
                         fontsize=10)
        self._build_selector()
        self.add_cylinder_button = Button(self.figure.add_axes([0.74, 0.925, 0.105, 0.035]), '+ Cylinder')
        self.add_box_button = Button(self.figure.add_axes([0.855, 0.925, 0.105, 0.035]), '+ Box')
        self.remove_obstacle_button = Button(self.figure.add_axes([0.08, 0.045, 0.19, 0.03]), 'Remove obstacle')
        self.add_cylinder_button.on_clicked(lambda event: self.add_obstacle('cylinder'))
        self.add_box_button.on_clicked(lambda event: self.add_obstacle('box'))
        self.remove_obstacle_button.on_clicked(self.remove_obstacle)
        self.x_input = TextBox(self.figure.add_axes([0.85, 0.64, 0.11, 0.035]), "x (m) ")
        self.y_input = TextBox(self.figure.add_axes([0.85, 0.59, 0.11, 0.035]), "y (m) ")
        self.altitude_input = TextBox(self.figure.add_axes([0.85, 0.54, 0.11, 0.035]), "Altitude (m) ")
        self.cruise_input = TextBox(self.figure.add_axes([0.85, 0.49, 0.11, 0.035]), "Cruise (m) ")
        self.heading_input = TextBox(self.figure.add_axes([0.85, 0.44, 0.11, 0.035]), "Heading (°) ")
        self.speed_input = TextBox(self.figure.add_axes([0.85, 0.39, 0.11, 0.035]),
                                  "Airspeed (m/s) " if config.coupled_flight else "XY speed (m/s) ")
        self.generator = CheckButtons(self.figure.add_axes([0.08, 0.9025, 0.31, 0.025]),
                                      ['Physical generator'], [config.generator_mode != 'geometric'])
        self.generator.active = config.coupled_flight
        self.generator.labels[0].set_fontsize(9)
        self.figure.text(0.41, 0.9125, 'Off: geometric generator', fontsize=8)
        self.generator.on_clicked(self.toggle_generator)
        self.backend = CheckButtons(self.figure.add_axes([0.62, 0.9025, 0.35, 0.025]),
                                    ['C++ construction (off: Python)'], [config.numerical_backend == 'cpp'])
        self.backend.labels[0].set_fontsize(9)
        self.backend.on_clicked(self.toggle_backend)
        self._sync_backend()
        self.apply_button = Button(self.figure.add_axes([0.77, 0.335, 0.19, 0.04]), "Apply")
        self.horizon_input = TextBox(self.figure.add_axes([0.19, 0.355, 0.08, 0.035]),
                                     "Horizon (s) ", initial=f"{config.horizon:g}")
        self.deadline_input = TextBox(self.figure.add_axes([0.43, 0.355, 0.07, 0.035]),
                                      "Arrival deadline (s) ", initial="Auto" if config.arrival_deadline is None else f"{config.arrival_deadline:g}")
        self.post_arrival_input = TextBox(self.figure.add_axes([0.66, 0.355, 0.06, 0.035]),
                                          "After arrival (s) ", initial=f"{config.post_arrival_time:g}")
        self.figure.text(0.08, 0.335, "Horizon = total duration; Auto deadline = horizon minus post-arrival time.", fontsize=8)
        self.spread_input = TextBox(self.figure.add_axes([0.85, 0.275, 0.11, 0.035]),
                                    "Approach spread (°) ", initial=f"{config.approach_spread_deg:g}")
        self.descent_input = TextBox(self.figure.add_axes([0.85, 0.135, 0.11, 0.035]),
                                     "Descent (°) ", initial=f"{config.descent_angle_deg:g}")
        self.descent_input.set_active(config.dimensions == 3)
        self.common_altitude = None
        self.auto_cruise = None
        cruise_bounds = config.cruise_altitude_range or (config.min_altitude, config.max_altitude)
        self.cruise_min_input = TextBox(self.figure.add_axes([0.44, 0.285, 0.09, 0.035]),
                                        "Cruise min (m) ", initial=f"{cruise_bounds[0]:g}")
        self.cruise_max_input = TextBox(self.figure.add_axes([0.64, 0.285, 0.08, 0.035]),
                                        "max (m) ", initial=f"{cruise_bounds[1]:g}")
        self.cruise_min_input.set_active(config.cruise_altitude_range is not None)
        self.cruise_max_input.set_active(config.cruise_altitude_range is not None)
        if config.dimensions == 3:
            self.auto_cruise = CheckButtons(self.figure.add_axes([0.08, 0.27, 0.21, 0.06]),
                                           ["Automatic cruise"], [config.cruise_altitude_range is not None])
            self.auto_cruise.on_clicked(self.toggle_auto_cruise)
            self.common_altitude = CheckButtons(self.figure.add_axes([0.74, 0.19, 0.23, 0.06]),
                                                ["Target: common altitude"], [config.target_altitude is not None])
            self.common_altitude.on_clicked(self.toggle_target_altitude)
        self.delay_min_input = TextBox(self.figure.add_axes([0.25, 0.205, 0.10, 0.04]),
                                       "Extra wait min (s) ", initial=f"{config.min_extra_loiter:g}")
        self.delay_max_input = TextBox(self.figure.add_axes([0.52, 0.205, 0.10, 0.04]),
                                       "max (s) ", initial=f"{config.max_extra_loiter:g}")
        self.wait_info = self.figure.text(0.08, 0.18,
                         f"Extra wait after activation at {config.activation_time:g} s. "
                         "0 / 0 = simultaneous departures.", fontsize=9)
        self.radius_input = TextBox(self.figure.add_axes([0.19, 0.135, 0.08, 0.035]), 'Loiter radius (m) ')
        self.loiter_direction = CheckButtons(self.figure.add_axes([0.29, 0.135, 0.17, 0.035]),
                                             ['Counterclockwise'], [snapshots[0].state[3] > 0])
        self.activation_input = TextBox(self.figure.add_axes([0.62, 0.135, 0.10, 0.035]),
                                       'Initial loiter (s) ', initial=f'{config.activation_time:g}')
        self.fit_button = Button(self.figure.add_axes([0.08, 0.085, 0.16, 0.04]), "Fit all")
        self.reset_button = Button(self.figure.add_axes([0.27, 0.085, 0.16, 0.04]), "Reset")
        self.start_button = Button(self.figure.add_axes([0.55, 0.085, 0.17, 0.04]), "Launch mission")
        self.status = self.figure.text(0.08, 0.028, "", fontsize=10)
        self.capabilities = self.figure.text(0.77, 0.035,
                         "The point marks the aircraft position.\n"
                         "Heading: 0° = east; 90° = north.\n"
                         f"Airspeed: {config.limits.min_speed:g} to {config.limits.max_speed:g} m/s.\n"
                         "Approach spread: 0° unrestricted; 360° full circle.\n"
                         f"Vz: +{config.max_climb_rate:g} / −{config.descent_rate_limit:g} m/s.", fontsize=9)
        self.profile_input = TextBox(self.figure.add_axes([0.17, 0.004, 0.40, 0.025]), 'Profile JSON ')
        self.profile_button = Button(self.figure.add_axes([0.59, 0.004, 0.13, 0.025]), 'Load profile')
        self.profile_input.set_active(config.coupled_flight)
        self.profile_button.set_active(config.coupled_flight)
        self.profile_button.on_clicked(self.load_profile)
        self.apply_button.on_clicked(self.apply_fields)
        self.fit_button.on_clicked(lambda event: self.draw(fit=True))
        self.reset_button.on_clicked(self.reset)
        self.start_button.on_clicked(self.start)
        self.figure.canvas.mpl_connect("button_press_event", self.click)
        self.select(0)
        self.draw(fit=True)

    def toggle_generator(self, label=None):
        requested = 'physical_mission' if self.generator.get_status()[0] else 'geometric'
        try:
            if requested == 'geometric' and self.config.numerical_backend == 'cpp':
                raise ValueError('Switch off C++ construction before selecting the geometric generator.')
            updated = replace(self.config, generator_mode=requested)
        except ValueError as error:
            self.generator.eventson = False
            if self.generator.get_status()[0] != (self.config.generator_mode != 'geometric'):
                self.generator.set_active(0)
            self.generator.eventson = True
            self.status.set_text(str(error))
            self.status.set_color('firebrick')
            self.figure.canvas.draw_idle()
            return
        self.config = updated
        self._sync_backend()
        self.draw()

    def _sync_backend(self):
        self.backend.eventson = False
        if self.backend.get_status()[0] != (self.config.numerical_backend == 'cpp'):
            self.backend.set_active(0)
        self.backend.eventson = True
        self.backend.active = self.config.coupled_flight and self.config.generator_mode != 'geometric'
        self.backend.labels[0].set_alpha(1. if self.backend.active else .4)

    def toggle_backend(self, label=None):
        requested = 'cpp' if self.backend.get_status()[0] else 'python'
        try:
            updated = replace(self.config, numerical_backend=requested)
            if requested == 'cpp':
                from .native_backend import require_native
                require_native()
        except (ValueError, RuntimeError) as error:
            self._sync_backend()
            self.status.set_text(str(error))
            self.status.set_color('firebrick')
            self.figure.canvas.draw_idle()
            return
        self.config = updated
        self._sync_backend()
        self.draw()

    def _build_selector(self):
        from matplotlib.widgets import RadioButtons
        if hasattr(self, 'selector'):
            self.selector.disconnect_events()
            self.selector.ax.remove()
        self.selector_labels = ([f'Drone {s.drone_id}' for s in self.snapshots] + ['Target']
                                + [f'Obstacle {i} · {o.kind}' for i, o in enumerate(self.config.obstacles)])
        self.selector = RadioButtons(self.figure.add_axes([0.74, 0.70, 0.23, 0.20]), self.selector_labels,
                                     active=min(self.selected, len(self.selector_labels)-1))
        for label in self.selector.labels:
            label.set_fontsize(8 if len(self.selector_labels) > 5 else 10)
        self.selector.on_clicked(lambda label: self.select(self.selector_labels.index(label)))

    def add_obstacle(self, kind):
        from .obstacles import Obstacle
        center = ((self.snapshots[0].state[0]+self.target[0])/2,
                  (self.snapshots[0].state[1]+self.target[1])/2, 40.)
        obstacle = Obstacle(kind, center, (25., 25., 40.))
        self.config = replace(self.config, obstacles=(*self.config.obstacles, obstacle))
        self._build_selector()
        self.selector.set_active(len(self.snapshots)+len(self.config.obstacles))
        self.draw(fit=True)

    def remove_obstacle(self, event=None):
        index = self.selected-len(self.snapshots)-1
        if not 0 <= index < len(self.config.obstacles):
            return False
        self.config = replace(self.config, obstacles=self.config.obstacles[:index]+self.config.obstacles[index+1:])
        self._build_selector()
        self.selector.set_active(len(self.snapshots))
        self.draw(fit=True)
        return True

    @staticmethod
    def _set_field(widget, value):
        """Populate a field without submitting input or drawing the whole figure.

        TextBox.set_val renders its cursor with a synchronous canvas.draw even
        when drawon is False. Selection/reset are display updates, not typing;
        update the text artist and let the caller queue one final draw instead.
        Keyboard editing keeps TextBox's input and submission semantics.
        """
        text = str(value)
        if widget.text != text:
            widget.text_disp.set_text(text)
            widget.cursor_index = len(text)
            widget.cursor.set_visible(False)

    def select(self, index):
        self.selected = index
        obstacle_index = index-len(self.snapshots)-1
        if obstacle_index >= 0:
            obstacle = self.config.obstacles[obstacle_index]
            for widget, value in zip((self.x_input, self.y_input, self.altitude_input), obstacle.center):
                self._set_field(widget, f'{value:.12g}')
                widget.set_active(True)
            for widget, value, label in (
                    (self.heading_input, obstacle.half_extents[0], 'Radius / half-X (m) '),
                    (self.speed_input, obstacle.half_extents[1], 'Half-Y (m) '),
                    (self.cruise_input, obstacle.half_extents[2], 'Half-height (m) ')):
                widget.label.set_text(label)
                self._set_field(widget, f'{value:.12g}')
                widget.set_active(True)
            self.speed_input.set_active(obstacle.kind == 'box')
            self.radius_input.set_active(False)
            self.profile_input.set_active(False)
            self.profile_button.set_active(False)
            self.capabilities.set_text('Obstacle: XYZ center and half-extents.\nCylinder: half-X = radius.\nClick the map to move its center.')
            self.figure.canvas.draw_idle()
            return
        self.heading_input.label.set_text('Heading (°) ')
        self.speed_input.label.set_text('Airspeed (m/s) ' if self.config.coupled_flight else 'XY speed (m/s) ')
        self.cruise_input.label.set_text('Cruise (m) ')
        drone = index < len(self.snapshots)
        radius = (abs(self.snapshots[index].speed/self.snapshots[index].state[3])
                  if drone and abs(self.snapshots[index].state[3]) > 1e-12 else 0.)
        self._set_field(self.radius_input, f'{radius:.12g}')
        self.radius_input.set_active(drone)
        if drone:
            self.loiter_direction.eventson = False
            if self.loiter_direction.get_status()[0] != (self.snapshots[index].state[3] > 0):
                self.loiter_direction.set_active(0)
            self.loiter_direction.eventson = True
        if index < len(self.snapshots):
            state = self.snapshots[index].state
            position, heading = state[:2], np.rad2deg(state[2])
        else:
            position, heading = self.target, None
        self._set_field(self.x_input, f"{position[0]:.12g}")
        self._set_field(self.y_input, f"{position[1]:.12g}")
        self._set_field(self.heading_input, "" if heading is None else f"{heading:.12g}")
        self.heading_input.set_active(heading is not None)
        self._set_field(self.speed_input, f"{self.snapshots[index].speed:.12g}" if heading is not None else "")
        self.speed_input.set_active(heading is not None)
        altitude = (self.snapshots[index].altitude if heading is not None else self.config.target_altitude)
        cruise = (self.snapshots[index].cruise_altitude if heading is not None else None)
        if heading is not None and cruise is None:
            cruise = altitude
        self._set_field(self.altitude_input, "" if altitude is None else f"{altitude:.12g}")
        automatic = self.config.cruise_altitude_range is not None
        self._set_field(self.cruise_input, "Auto" if automatic and heading is not None else "" if cruise is None else f"{cruise:.12g}")
        self.altitude_input.set_active(self.config.dimensions == 3 and altitude is not None)
        self.cruise_input.set_active(self.config.dimensions == 3 and heading is not None and not automatic)
        local = self.config.for_drone(self.snapshots[index].drone_id) if heading is not None else None
        profile = self.config.profile_for(self.snapshots[index].drone_id) if heading is not None else None
        self.profile_input.set_active(heading is not None and self.config.coupled_flight and self.profile_loading)
        self.profile_button.set_active(heading is not None and self.config.coupled_flight and self.profile_loading)
        self.capabilities.set_text(
            (f'Profile: {profile.name[:30]}\n' if profile else 'Profile: common configuration\n') +
            f'Airspeed: {local.limits.min_speed:g}–{local.limits.max_speed:g} m/s\n'
            f'Bank: {local.limits.max_bank_deg:g}°; radius: {local.aircraft_radius:g} m\n'
            f'Vz: +{local.max_climb_rate:g} / −{local.descent_rate_limit:g} m/s'
            if local is not None else 'Target: shared mission settings.')
        self.figure.canvas.draw_idle()

    def load_profile(self, event=None):
        """Pre-planning assignment only; no mutation of a running simulation."""
        if not self.profile_loading or not self.config.coupled_flight or self.selected >= len(self.snapshots):
            return False
        from .aircraft_profiles import AircraftProfile, PerformanceLimits, load_aircraft_profile
        from .flight3d import from_snapshot, trim_control, envelope_ok
        try:
            profile = load_aircraft_profile(self.profile_input.text.strip())
            snapshot = self.snapshots[self.selected]
            local = profile.apply(replace(self.config, aircraft_profiles=()))
            state = from_snapshot(snapshot)
            if not envelope_ok(state, trim_control(state, local.flight), local):
                raise ValueError('Profile incompatible with the current airspeed or loiter circle.')
            bindings = dict(self.config.aircraft_profiles)
            if not bindings:
                common = AircraftProfile('configured_common_airframe',
                    'Snapshot of the common configured physics before a GUI per-ID assignment; not calibrated.',
                    'user_supplied', self.config.limits, self.config.flight,
                    PerformanceLimits(self.config.max_climb_rate, self.config.descent_rate_limit,
                                      self.config.max_vertical_acceleration), self.config.aircraft_radius or None)
                bindings = {s.drone_id: common for s in self.snapshots}
            bindings[snapshot.drone_id] = profile
            updated = replace(self.config, aircraft_profiles=tuple(sorted(bindings.items())))
        except (ValueError, OSError) as error:
            self.status.set_text(str(error))
            self.status.set_color('firebrick')
            self.figure.canvas.draw_idle()
            return False
        self.config = updated
        self.select(self.selected)
        self.draw()
        return True

    def toggle_auto_cruise(self, label):
        enabled = self.auto_cruise.get_status()[0]
        try:
            bounds = (float(self.cruise_min_input.text), float(self.cruise_max_input.text)) if enabled else None
            updated = replace(self.config, cruise_altitude_range=bounds)
        except ValueError as error:
            self.auto_cruise.eventson = False
            self.auto_cruise.set_active(0)
            self.auto_cruise.eventson = True
            self.status.set_text(str(error))
            self.status.set_color("firebrick")
            self.figure.canvas.draw_idle()
            return
        self.config = updated
        self.cruise_min_input.set_active(enabled)
        self.cruise_max_input.set_active(enabled)
        self.select(self.selected)
        self.draw()

    def toggle_target_altitude(self, label):
        enabled = self.common_altitude.get_status()[0]
        altitude = float(np.clip(self.snapshots[0].altitude, self.config.min_altitude, self.config.max_altitude))
        try:
            updated = replace(self.config, target_altitude=altitude if enabled else None)
        except ValueError as error:
            self.common_altitude.eventson = False
            self.common_altitude.set_active(0)
            self.common_altitude.eventson = True
            self.status.set_text(str(error))
            self.status.set_color("firebrick")
            self.figure.canvas.draw_idle()
            return
        self.config = updated
        self.select(self.selected)
        self.draw()

    def place(self, position, heading=None, speed=None, altitude=None, cruise=None, *, refresh=True):
        obstacle_index = self.selected-len(self.snapshots)-1
        if obstacle_index >= 0:
            from .obstacles import Obstacle
            obstacle = self.config.obstacles[obstacle_index]
            updated = Obstacle(obstacle.kind, (*map(float, position), obstacle.center[2]), obstacle.half_extents)
            values = list(self.config.obstacles)
            values[obstacle_index] = updated
            self.config = replace(self.config, obstacles=tuple(values))
            if refresh:
                self.select(self.selected)
                self.draw()
            return
        if any(z is not None and (not np.isfinite(z) or not self.config.min_altitude <= z <= self.config.max_altitude)
               for z in (altitude, cruise)):
            raise ValueError(f"Required altitude: {self.config.min_altitude:g} to {self.config.max_altitude:g} m.")
        if self.selected < len(self.snapshots):
            snapshot = self.snapshots[self.selected]
            if speed is not None:
                snapshot = snapshot.with_speed(speed, self.config.for_drone(snapshot.drone_id).limits)
            state = (*map(float, position),
                     snapshot.state[2] if heading is None else float(wrap(np.deg2rad(heading))),
                     snapshot.state[3])
            self.snapshots[self.selected] = replace(snapshot, state=state,
                altitude=snapshot.altitude if altitude is None else altitude,
                cruise_altitude=snapshot.cruise_altitude if cruise is None else cruise)
        else:
            self.target = tuple(map(float, position))
            if altitude is not None and self.config.target_altitude is not None:
                self.config = replace(self.config, target_altitude=altitude)
        if refresh:
            self.select(self.selected)
            self.draw()

    def click(self, event):
        if event.inaxes is not self.axis or event.button != 1:
            return
        toolbar = getattr(self.figure.canvas, "toolbar", None)
        if toolbar is not None and toolbar.mode:
            return
        if event.xdata is not None and event.ydata is not None:
            self.place((event.xdata, event.ydata))

    def apply_fields(self, event=None):
        if self.selected > len(self.snapshots):
            return self.apply_obstacle_fields()
        try:
            position = float(self.x_input.text), float(self.y_input.text)
            heading = float(self.heading_input.text) if self.selected < len(self.snapshots) else None
            speed = float(self.speed_input.text) if heading is not None else None
            spread = float(self.spread_input.text)
            delay_min = float(self.delay_min_input.text)
            delay_max = float(self.delay_max_input.text)
            altitude = float(self.altitude_input.text) if self.altitude_input.active else None
            cruise = float(self.cruise_input.text) if self.cruise_input.active else None
            cruise_bounds = ((float(self.cruise_min_input.text), float(self.cruise_max_input.text))
                             if self.auto_cruise is not None and self.auto_cruise.get_status()[0] else None)
            descent = float(self.descent_input.text)
            horizon = float(self.horizon_input.text)
            deadline = None if self.deadline_input.text.strip().lower() in ("", "auto") else float(self.deadline_input.text)
            post_arrival = float(self.post_arrival_input.text)
            activation = float(self.activation_input.text)
            radius = float(self.radius_input.text) if heading is not None else None
            counter_clockwise = self.loiter_direction.get_status()[0]
            if not np.all(np.isfinite((*position, spread, delay_min, delay_max,
                                       *[z for z in (altitude, cruise) if z is not None],
                                       *(() if heading is None else (heading, speed))))):
                raise ValueError
        except ValueError:
            self.status.set_text("All fields must contain finite numbers.")
            self.status.set_color("firebrick")
            self.figure.canvas.draw_idle()
            return False
        try:
            if not 0 <= spread <= 360:
                raise ValueError("Approach spread must be between 0° and 360°.")
            if not 0 <= delay_min <= delay_max:
                raise ValueError("Extra wait must satisfy: 0 ≤ minimum ≤ maximum (seconds).")
            replace(self.config, cruise_altitude_range=cruise_bounds, descent_angle_deg=descent,
                    horizon=horizon, arrival_deadline=deadline, post_arrival_time=post_arrival,
                    activation_time=activation)
            if radius is not None and (not np.isfinite(radius) or radius < 0):
                raise ValueError('Loiter radius must be finite and positive, or 0 for a straight departure.')
            self.place(position, heading, speed, altitude, cruise, refresh=False)
            if radius is not None:
                snapshot = self.snapshots[self.selected]
                rate = (snapshot.speed/radius * (1 if counter_clockwise else -1)
                        if radius > 0 else 0.)
                self.snapshots[self.selected] = replace(snapshot, state=(*snapshot.state[:3], rate))
        except ValueError as error:
            self.status.set_text(str(error))
            self.status.set_color("firebrick")
            self.figure.canvas.draw_idle()
            return False
        self.config = replace(self.config, approach_spread_deg=spread,
                              min_extra_loiter=delay_min, max_extra_loiter=delay_max,
                              cruise_altitude_range=cruise_bounds, descent_angle_deg=descent,
                              horizon=horizon, arrival_deadline=deadline, post_arrival_time=post_arrival,
                              activation_time=activation)
        self.select(self.selected)
        self.draw()
        return True

    def apply_obstacle_fields(self):
        from .obstacles import Obstacle
        try:
            index = self.selected-len(self.snapshots)-1
            old = self.config.obstacles[index]
            half_x = float(self.heading_input.text)
            half_y = float(self.speed_input.text) if old.kind == 'box' else half_x
            obstacle = Obstacle(old.kind,
                (float(self.x_input.text), float(self.y_input.text), float(self.altitude_input.text)),
                (half_x, half_y, float(self.cruise_input.text)))
            values = list(self.config.obstacles)
            values[index] = obstacle
            # Global mission edits remain effective with an obstacle selected.
            bounds = ((float(self.cruise_min_input.text), float(self.cruise_max_input.text))
                      if self.auto_cruise is not None and self.auto_cruise.get_status()[0] else None)
            self.config = replace(self.config, obstacles=tuple(values),
                horizon=float(self.horizon_input.text), activation_time=float(self.activation_input.text),
                arrival_deadline=None if self.deadline_input.text.strip().lower() in ('', 'auto') else float(self.deadline_input.text),
                post_arrival_time=float(self.post_arrival_input.text),
                min_extra_loiter=float(self.delay_min_input.text), max_extra_loiter=float(self.delay_max_input.text),
                approach_spread_deg=float(self.spread_input.text), descent_angle_deg=float(self.descent_input.text),
                cruise_altitude_range=bounds)
        except ValueError as error:
            self.status.set_text(str(error))
            self.status.set_color('firebrick')
            self.figure.canvas.draw_idle()
            return False
        self.draw()
        return True

    def initial_conflict(self):
        from .obstacles import obstacle_preflight
        conflict = obstacle_preflight(self.snapshots, replace(self.config, target=self.target))
        if conflict:
            return conflict[1]
        for snapshot in self.snapshots:
            local = self.config.for_drone(snapshot.drone_id)
            if self.config.coupled_flight:
                from .flight3d import from_snapshot, trim_control, envelope_ok
                state = from_snapshot(snapshot)
                if not envelope_ok(state[None], trim_control(state, local.flight)[None], local):
                    return f"Drone {snapshot.drone_id}: loiter violates the flight envelope (load, stall, propulsion)."
            error = descent_error(snapshot, local)
            if error is not None:
                return error
        for i, first in enumerate(self.snapshots):
            for second in self.snapshots[i + 1:]:
                distance = (np.linalg.norm(first.position3d - second.position3d) if self.config.dimensions == 3
                            else np.linalg.norm(np.subtract(first.state[:2], second.state[:2])))
                speed_sum = (self.config.for_drone(first.drone_id).limits.max_speed + self.config.for_drone(second.drone_id).limits.max_speed if self.config.coupled_flight else
                             np.hypot(first.speed, self.config.vertical_speed_limit) + np.hypot(second.speed, self.config.vertical_speed_limit)
                             if self.config.dimensions == 3 else first.speed + second.speed)
                required = (self.config.separation + self.config.for_drone(first.drone_id).aircraft_radius
                            + self.config.for_drone(second.drone_id).aircraft_radius + 2 * self.config.position_error
                            + speed_sum * self.config.clock_error)
                if distance < required:
                    return (f"Drones {first.drone_id} and {second.drone_id} too close at departure: "
                            f"{distance:.1f} m; required minimum {required:.1f} m.")
        return None

    def draw(self, fit=False):
        import matplotlib.pyplot as plt

        bounds = self.axis.get_xlim(), self.axis.get_ylim()
        self.wait_info.set_text(f'Extra wait after the initial loiter of {self.config.activation_time:g} s. '
                                '0 / 0 = no extra wait.')
        self.axis.clear()
        from .obstacle_view import draw_obstacles_2d
        draw_obstacles_2d(self.axis, self.config)
        colors = plt.cm.tab10(np.linspace(0, 0.9, len(self.snapshots)))
        for snapshot, color in zip(self.snapshots, colors):
            x, y, heading, _ = snapshot.state
            circle = holding_circle(snapshot.state, snapshot.speed)
            if circle is not None:
                centre, radius = circle
                self.axis.add_patch(plt.Circle(centre, radius, fill=False, color=color, ls=":"))
                self.axis.plot(*centre, "+", color=color, alpha=0.5)
            self.axis.plot(x, y, "o", color=color, ms=8)
            self.axis.quiver(x, y, np.cos(heading), np.sin(heading), color=color,
                             angles="xy", scale_units="xy", scale=0.1, width=0.006)
            altitude_label = f" · z={snapshot.altitude:g} m" if self.config.dimensions == 3 else ""
            self.axis.annotate(f"Drone {snapshot.drone_id} · {snapshot.speed:g} m/s{altitude_label}", (x, y), xytext=(7, 7),
                               textcoords="offset points", color=color)
        self.axis.add_patch(plt.Circle(self.target, self.config.target_radius, color="gold", alpha=0.35))
        self.axis.plot(*self.target, "kx", ms=12)
        assignments = approach_angles(self.snapshots, replace(self.config, target=self.target))
        for snapshot, color in zip(self.snapshots, colors):
            angle = assignments[snapshot.drone_id]
            if angle is not None:
                offset = 45 * np.array([np.cos(angle), np.sin(angle)])
                start = np.asarray(self.target) + offset
                end = np.asarray(self.target) + 0.25 * offset
                self.axis.annotate("", xy=end, xytext=start,
                                   arrowprops={"arrowstyle": "->", "color": color, "lw": 1.5})
                self.axis.annotate(f"D{snapshot.drone_id}", start, xytext=(4, 4),
                                   textcoords="offset points", color=color, fontsize=8)
        target_label = ("Target · separate altitudes" if self.config.dimensions == 3 and self.config.target_altitude is None
                        else f"Target · z={self.config.target_altitude:g} m" if self.config.target_altitude is not None else "Target")
        self.axis.annotate(target_label, self.target, xytext=(7, 7), textcoords="offset points")
        self.axis.set(xlabel="x (m)", ylabel="y (m)")
        self.axis.set_aspect("equal", adjustable="box")
        self.axis.grid(alpha=0.25)
        if fit:
            self.axis.margins(0.25)
            self.axis.autoscale_view()
        else:
            self.axis.set_xlim(bounds[0])
            self.axis.set_ylim(bounds[1])
        conflict = self.initial_conflict()
        self.status.set_text(conflict or "Placement ready. Trajectories will be validated at launch.")
        self.status.set_color("firebrick" if conflict else "darkgreen")
        self.figure.canvas.draw_idle()

    def reset(self, event=None):
        self.snapshots = list(self.original[0])
        self.target = self.original[1]
        self.config = self.original_config
        self.generator.eventson = False
        if self.generator.get_status()[0] != (self.config.generator_mode != 'geometric'):
            self.generator.set_active(0)
        self.generator.eventson = True
        self._sync_backend()
        self._build_selector()
        self.selected = min(self.selected, len(self.snapshots)+len(self.config.obstacles))
        self._set_field(self.activation_input, f'{self.config.activation_time:g}')
        self._set_field(self.descent_input, f"{self.config.descent_angle_deg:g}")
        if self.common_altitude is not None:
            self.common_altitude.eventson = False
            if self.common_altitude.get_status()[0] != (self.config.target_altitude is not None):
                self.common_altitude.set_active(0)
            self.common_altitude.eventson = True
        if self.auto_cruise is not None:
            enabled = self.config.cruise_altitude_range is not None
            self.auto_cruise.eventson = False
            if self.auto_cruise.get_status()[0] != enabled:
                self.auto_cruise.set_active(0)
            self.auto_cruise.eventson = True
            bounds = self.config.cruise_altitude_range or (self.config.min_altitude, self.config.max_altitude)
            self._set_field(self.cruise_min_input, f"{bounds[0]:g}")
            self._set_field(self.cruise_max_input, f"{bounds[1]:g}")
            self.cruise_min_input.set_active(enabled)
            self.cruise_max_input.set_active(enabled)
        self._set_field(self.spread_input, f"{self.config.approach_spread_deg:g}")
        self._set_field(self.delay_min_input, f"{self.config.min_extra_loiter:g}")
        self._set_field(self.delay_max_input, f"{self.config.max_extra_loiter:g}")
        self._set_field(self.horizon_input, f"{self.config.horizon:g}")
        self._set_field(self.deadline_input, "Auto" if self.config.arrival_deadline is None else f"{self.config.arrival_deadline:g}")
        self._set_field(self.post_arrival_input, f"{self.config.post_arrival_time:g}")
        self.select(self.selected)
        self.draw(fit=True)

    def start(self, event=None):
        import matplotlib.pyplot as plt

        if not self.apply_fields() or self.initial_conflict():
            return
        if self.config.numerical_backend == 'cpp':
            try:
                from .native_backend import require_native
                require_native()
            except RuntimeError as error:
                self.status.set_text(str(error))
                self.status.set_color('firebrick')
                self.figure.canvas.draw_idle()
                return
        if self.launch_validator is not None:
            try:
                self.status.set_text('Computing and validating trajectories…')
                self.figure.canvas.draw()
                self.figure.canvas.flush_events()
                self.launch_validator(tuple(self.snapshots), replace(self.config, target=self.target))
            except (ValueError, OSError, RuntimeError) as error:
                self.status.set_text(str(error))
                self.status.set_color('firebrick')
                self.figure.canvas.draw_idle()
                return
        self.accepted = True
        plt.close(self.figure)


def edit_scenario(snapshots, target, config, launch_validator=None, profile_loading=True):
    import matplotlib.pyplot as plt

    editor = ScenarioEditor(snapshots, target, config, launch_validator=launch_validator,
                            profile_loading=profile_loading)
    if not profile_loading:
        editor.figure.suptitle(f'Gazebo / PX4 mission — {len(snapshots)} aircraft, target and obstacles')
        editor.start_button.label.set_text('Calculate → Gazebo')
    plt.show()
    if not editor.accepted:
        return None
    return tuple(editor.snapshots), replace(editor.config, target=editor.target)
