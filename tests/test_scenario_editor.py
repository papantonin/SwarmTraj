"""Exercise editor interactions without requiring a graphical display."""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from dataclasses import replace
from matplotlib.backend_bases import KeyEvent, MouseEvent
import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, separate_loiter_snapshot
from swarm_sim.trajectory2d.dynamics import holding_circle
from swarm_sim.trajectory2d.scenario_editor import ScenarioEditor
from pathlib import Path


@pytest.fixture
def editor():
    result = ScenarioEditor(tuple(replace(s, altitude=80.) for s in separate_loiter_snapshot()), (140, 50), Config(dimensions=3))
    yield result
    plt.close(result.figure)


def test_selection_batches_field_updates_without_synchronous_draws(editor, monkeypatch):
    draws, queued, submitted = [], [], []
    monkeypatch.setattr(editor.figure.canvas, 'draw', lambda: draws.append(True))
    monkeypatch.setattr(editor.figure.canvas, 'draw_idle', lambda: queued.append(True))
    editor.x_input.on_submit(submitted.append)
    editor.select(len(editor.snapshots))
    assert editor.x_input.text == f'{editor.target[0]:.12g}'
    assert not editor.heading_input.active
    assert editor.heading_input.text == ''
    assert not draws and not submitted and len(queued) == 1
    editor.select(1)
    assert editor.x_input.text == f'{editor.snapshots[1].state[0]:.12g}'
    assert editor.heading_input.active
    assert editor.x_input.cursor_index == len(editor.x_input.text)
    assert not editor.x_input.cursor.get_visible()
    assert not draws and not submitted and len(queued) == 2


def test_obstacle_selection_and_reset_do_not_render_each_field(editor, monkeypatch):
    editor.add_obstacle('cylinder')
    draws, queued = [], []
    monkeypatch.setattr(editor.figure.canvas, 'draw', lambda: draws.append(True))
    monkeypatch.setattr(editor.figure.canvas, 'draw_idle', lambda: queued.append(True))
    editor.select(len(editor.snapshots)+1)
    assert editor.heading_input.text == f'{editor.config.obstacles[0].half_extents[0]:.12g}'
    assert not draws and len(queued) == 1
    editor.horizon_input.set_val('300')  # Normal TextBox input still works.
    draws.clear()
    editor.reset()
    assert editor.horizon_input.text == f'{editor.original_config.horizon:g}'
    assert not draws


def test_apply_refreshes_geometry_once_and_preserves_textbox_input(editor, monkeypatch):
    submitted = []
    editor.x_input.on_submit(submitted.append)
    editor.x_input.set_val('123')
    assert submitted == ['123']
    refreshes = []
    original = editor.draw

    def draw(*args, **kwargs):
        refreshes.append(True)
        return original(*args, **kwargs)

    monkeypatch.setattr(editor, 'draw', draw)
    assert editor.apply_fields()
    assert editor.snapshots[0].state[0] == 123
    assert len(refreshes) == 1


def test_keyboard_editing_after_silent_selection(editor):
    editor.select(len(editor.snapshots))
    widget = editor.x_input
    previous = widget.text
    submitted = []
    widget.on_submit(submitted.append)
    widget.begin_typing()
    try:
        for key in ('end', '9', 'backspace', 'enter'):
            event = KeyEvent('key_press_event', editor.figure.canvas, key=key)
            editor.figure.canvas.callbacks.process('key_press_event', event)
            if key == '9':
                assert widget.text == previous+'9'
        assert widget.text == previous
        assert submitted == [previous]
    finally:
        widget.stop_typing()


def test_mouse_click_does_not_redraw_every_unfocused_input(editor, monkeypatch):
    synchronous, queued, clicked = [], [], []
    monkeypatch.setattr(editor.figure.canvas, 'draw', lambda: synchronous.append(True))
    monkeypatch.setattr(editor.figure.canvas, 'draw_idle', lambda: queued.append(True))
    editor.apply_button.on_clicked(lambda event: clicked.append(True))
    x, y = editor.apply_button.ax.transAxes.transform((.5, .5))
    for kind in ('motion_notify_event', 'button_press_event', 'button_release_event'):
        event = MouseEvent(kind, editor.figure.canvas, x, y, button=1)
        editor.figure.canvas.callbacks.process(kind, event)
    assert clicked == [True]  # Exercise the actual canvas event dispatch.
    assert not synchronous
    assert len(queued) == 2  # Field refresh and scene refresh, coalesced by Tk.


def test_mouse_focus_hover_and_typing_queue_paints_without_blocking(editor, monkeypatch):
    synchronous, submitted, queued = [], [], []
    monkeypatch.setattr(editor.figure.canvas, 'draw', lambda: synchronous.append(True))
    monkeypatch.setattr(editor.figure.canvas, 'draw_idle', lambda: queued.append(True))
    field = editor.x_input
    field.on_submit(submitted.append)
    x, y = field.ax.transAxes.transform((.5, .5))
    for kind in ('motion_notify_event', 'button_press_event', 'button_release_event'):
        editor.figure.canvas.callbacks.process(
            kind, MouseEvent(kind, editor.figure.canvas, x, y, button=1))
    assert field.capturekeystrokes and field.cursor.get_visible()
    for key in ('end', '9', 'backspace'):
        editor.figure.canvas.callbacks.process(
            'key_press_event', KeyEvent('key_press_event', editor.figure.canvas, key=key))
    # Clicking elsewhere submits only the input that owned focus.
    x, y = editor.fit_button.ax.transAxes.transform((.5, .5))
    editor.figure.canvas.callbacks.process(
        'button_press_event', MouseEvent('button_press_event', editor.figure.canvas, x, y, button=1))
    assert submitted == [field.text]
    assert not field.capturekeystrokes and not field.cursor.get_visible()
    assert queued and not synchronous


def test_per_drone_profile_loading_and_reset_are_transactional():
    root = Path(__file__).resolve().parents[1]
    initial = tuple(replace(s, altitude=30 + 25 * s.drone_id) for s in separate_loiter_snapshot())
    editor = ScenarioEditor(initial, (140, 50), Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp'))
    try:
        original = editor.config
        editor.profile_input.set_val(str(root / 'examples/aircraft/fixed_wing_training.json'))
        assert editor.load_profile()
        assert len(editor.config.aircraft_profiles) == 4
        assert editor.config.for_drone(0).flight.max_brake_acceleration == 0
        assert editor.config.for_drone(1).flight.max_brake_acceleration == 5
        assert '35°' in editor.capabilities.get_text()
        configured = editor.config
        editor.profile_input.set_val(str(root / 'examples/aircraft/rascal110_empty_approx.json'))
        assert not editor.load_profile()  # 30° bank cannot fly the unchanged 25 m/12 m/s circle.
        assert editor.config is configured and tuple(editor.snapshots) == initial
        editor.select(4)
        assert not editor.profile_input.active and not editor.load_profile()
        editor.reset()
        assert editor.config == original
        assert not editor.config.aircraft_profiles
    finally:
        plt.close(editor.figure)


def test_map_click_moves_aircraft_not_circle_centre_and_reset_restores_scene(editor):
    original = tuple(editor.snapshots)
    editor.figure.canvas.draw()
    pixel = editor.axis.transData.transform((-30, -60))
    click = MouseEvent("button_press_event", editor.figure.canvas, *pixel, button=1)
    editor.figure.canvas.callbacks.process("button_press_event", click)
    np.testing.assert_allclose(editor.snapshots[0].state[:2], (-30, -60), atol=1e-10)
    centre, radius = holding_circle(editor.snapshots[0].state, editor.snapshots[0].speed)
    assert np.linalg.norm(centre - editor.snapshots[0].state[:2]) == pytest.approx(radius)
    assert editor.snapshots[1:] == list(original[1:])
    editor.selector.set_active(4)
    editor.place((300, -100))
    editor.reset()
    assert tuple(editor.snapshots) == original
    assert editor.target == (140, 50)
    assert not editor.accepted


def test_exact_coordinates_heading_target_and_launch_apply_pending_fields(editor):
    editor.x_input.set_val("-1000")
    editor.y_input.set_val("250")
    editor.heading_input.set_val("180")
    assert editor.apply_fields()
    np.testing.assert_allclose(editor.snapshots[0].state, (-1000, 250, -np.pi, 12 / 25))
    editor.selector.set_active(4)
    editor.x_input.set_val("450.125")
    editor.y_input.set_val("-280.5")
    editor.start()
    assert editor.accepted
    assert editor.target == (450.125, -280.5)


def test_invalid_fields_and_initial_conflict_prevent_launch(editor):
    editor.x_input.set_val("nan")
    editor.start()
    assert not editor.accepted
    assert "finite" in editor.status.get_text()
    editor.select(0)
    editor.place(editor.snapshots[1].state[:2])
    editor.start()
    assert not editor.accepted
    assert "too close" in editor.status.get_text()
    editor.reset()
    editor.start()
    assert editor.accepted


def test_speed_changes_preserve_loiter_geometry_and_are_applied_at_launch(editor):
    original = editor.snapshots[0]
    centre, radius = holding_circle(original.state, original.speed)
    editor.speed_input.set_val("15")
    assert editor.apply_fields()
    changed = editor.snapshots[0]
    assert changed.speed == 15
    updated_centre, updated_radius = holding_circle(changed.state, changed.speed)
    np.testing.assert_allclose(updated_centre, centre, atol=1e-10)
    assert updated_radius == pytest.approx(radius)
    editor.selector.set_active(1)
    editor.speed_input.set_val("10")
    editor.start()
    assert editor.accepted
    assert [s.speed for s in editor.snapshots] == [15, 10, 12, 12]


def test_invalid_speed_does_not_modify_snapshot_and_target_has_no_speed(editor):
    original = editor.snapshots[0]
    editor.speed_input.set_val("100")
    editor.start()
    assert not editor.accepted
    assert editor.snapshots[0] == original
    assert "8 to 16 m/s" in editor.status.get_text()
    editor.selector.set_active(4)
    assert not editor.speed_input.active
    editor.start()
    assert editor.accepted


def test_global_approach_spread_is_applied_and_invalid_values_block_launch(editor):
    editor.selector.set_active(4)
    editor.spread_input.set_val("400")
    editor.start()
    assert not editor.accepted
    assert "360°" in editor.status.get_text()
    editor.spread_input.set_val("360")
    assert editor.apply_fields()
    assert editor.config.approach_spread_deg == 360
    editor.reset()
    assert editor.config.approach_spread_deg == 0
    editor.spread_input.set_val("180")
    editor.start()
    assert editor.accepted
    assert editor.config.approach_spread_deg == 180


def test_departure_delay_interval_validation_reset_and_launch(editor):
    editor.delay_min_input.set_val("4")
    editor.delay_max_input.set_val("1")
    editor.start()
    assert not editor.accepted
    assert "minimum ≤ maximum" in editor.status.get_text()
    editor.delay_min_input.set_val("1")
    editor.delay_max_input.set_val("2")
    assert editor.apply_fields()
    assert (editor.config.min_extra_loiter, editor.config.max_extra_loiter) == (1, 2)
    editor.reset()
    assert editor.config.max_extra_loiter == 22
    assert editor.delay_max_input.text == "22"
    editor.delay_min_input.set_val("0")
    editor.delay_max_input.set_val("0")
    editor.start()
    assert editor.accepted
    assert editor.config.min_extra_loiter == editor.config.max_extra_loiter == 0


def test_altitudes_common_destination_and_3d_initial_separation():
    snapshots = tuple(replace(s, altitude=30 + 25 * k, cruise_altitude=40 + 25 * k)
                      for k, s in enumerate(separate_loiter_snapshot()))
    editor = ScenarioEditor(snapshots, (140, 50), Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp'))
    try:
        editor.altitude_input.set_val("35")
        editor.cruise_input.set_val("50")
        assert editor.apply_fields()
        assert editor.snapshots[0].altitude == 35
        assert editor.snapshots[0].cruise_altitude == 50
        editor.common_altitude.set_active(0)
        editor.selector.set_active(4)
        editor.altitude_input.set_val("25")
        assert editor.apply_fields()
        assert editor.config.target_altitude == 25
        editor.reset()
        assert editor.config.target_altitude is None
        assert not editor.common_altitude.get_status()[0]
        editor.select(0)
        editor.place(editor.snapshots[1].state[:2])
        assert editor.initial_conflict() is None  # Same XY, but 25 m of vertical separation.
        editor.altitude_input.set_val("55")
        assert editor.apply_fields()
        assert editor.initial_conflict() is not None
    finally:
        plt.close(editor.figure)


def test_automatic_cruise_controls_validation_and_manual_switch():
    snapshots = tuple(replace(s, altitude=30 + 25 * k) for k, s in enumerate(separate_loiter_snapshot()))
    editor = ScenarioEditor(snapshots, (500, 300), Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', target_altitude=30,
                            cruise_altitude_range=(40, 110)))
    try:
        assert editor.auto_cruise.get_status()[0]
        assert not editor.cruise_input.active
        assert editor.cruise_input.text == "Auto"
        assert editor.altitude_input.active
        editor.cruise_min_input.set_val("120")
        editor.cruise_max_input.set_val("80")
        editor.start()
        assert not editor.accepted
        assert editor.config.cruise_altitude_range == (40, 110)
        editor.cruise_min_input.set_val("50")
        editor.cruise_max_input.set_val("100")
        assert editor.apply_fields()
        assert editor.config.cruise_altitude_range == (50, 100)
        editor.auto_cruise.set_active(0)
        assert editor.config.cruise_altitude_range is None
        assert editor.cruise_input.active
        editor.cruise_input.set_val("70")
        assert editor.apply_fields()
        assert editor.snapshots[0].cruise_altitude == 70
        editor.reset()
        assert editor.auto_cruise.get_status()[0]
        assert editor.config.cruise_altitude_range == (40, 110)
        assert editor.config.target_altitude == 30
        assert not editor.cruise_input.active
    finally:
        plt.close(editor.figure)


def test_coupled_editor_displays_total_airspeed_and_refuses_loaded_stall():
    from swarm_sim.trajectory2d import FlightLimits
    config = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', flight=FlightLimits(stall_speed=8))
    snapshots = tuple(replace(s.with_speed(8, config.limits), altitude=30) for s in separate_loiter_snapshot())
    editor = ScenarioEditor(snapshots, config.target, config)
    try:
        assert "Airspeed" in editor.speed_input.label.get_text()
        editor.start()
        assert not editor.accepted
        assert "stall" in editor.status.get_text()
    finally:
        plt.close(editor.figure)


def test_moving_drone_updates_geometric_side_and_gate_labels():
    from swarm_sim.trajectory2d import Snapshot
    from swarm_sim.trajectory2d.planning import approach_angles
    snapshots = tuple(Snapshot(i, (x, -200, 0, 0), 12) for i, x in enumerate([90, -90, 30, -30]))
    config = Config(dimensions=3, target=(0, 0), approach_spread_deg=180)
    editor = ScenarioEditor(snapshots, config.target, config)
    try:
        assert np.cos(approach_angles(editor.snapshots, config)[0]) > 0.99
        editor.place((-150, -200))
        assert np.cos(approach_angles(editor.snapshots, config)[0]) < 0
        labels = {text.get_text() for text in editor.axis.texts}
        assert {"D0", "D1", "D2", "D3"} <= labels
    finally:
        plt.close(editor.figure)
