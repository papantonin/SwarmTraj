"""Presentation changes preserve pending input, constraints and launch gates."""
from dataclasses import replace
import importlib.util

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backend_bases import KeyEvent, MouseEvent
import pytest

from swarm_sim.px4_fleet import default_fleet
from swarm_sim.trajectory2d import Config, separate_loiter_snapshot
from swarm_sim.trajectory2d.scenario_editor import ScenarioEditor


@pytest.fixture
def editor():
    snapshots, config = default_fleet()
    result = ScenarioEditor(snapshots, config.target, config, profile_loading=False)
    yield result
    plt.close(result.figure)


def click(editor, widget):
    x, y = widget.ax.transAxes.transform((.5, .5))
    for name in ('button_press_event', 'button_release_event'):
        editor.figure.canvas.callbacks.process(name, MouseEvent(name, editor.figure.canvas, x, y, button=1))


def test_mission_is_default_and_advanced_reuses_controls_without_changing_inputs(editor):
    original = editor.config, tuple(editor.snapshots), editor.target
    widgets = tuple(editor._inputs)
    editor.axis.set_xlim(-100, 500)
    editor.axis.set_ylim(-400, 700)
    viewport = editor.axis.get_xlim(), editor.axis.get_ylim()
    assert not editor.advanced
    assert sum(w.ax.get_visible() for w in widgets) == 5
    assert editor.x_input.ax.get_visible() and editor.heading_input.ax.get_visible()
    assert not editor.generator.ax.get_visible() and not editor.backend.ax.get_visible()
    assert not editor.horizon_input.ax.get_visible()
    click(editor, editor.advanced_button)
    assert editor.advanced and editor.horizon_input.ax.get_visible()
    assert editor.generator.ax.get_visible() and editor.backend.ax.get_visible()
    assert not editor.x_input.ax.get_visible()
    assert tuple(editor._inputs) == widgets
    click(editor, editor.mission_button)
    assert not editor.advanced and editor.mission_summary.get_visible()
    assert (editor.config, tuple(editor.snapshots), editor.target) == original
    assert (editor.axis.get_xlim(), editor.axis.get_ylim()) == viewport


def test_switching_views_preserves_invalid_pending_text_and_releases_keyboard_focus(editor):
    original = editor.config
    editor.x_input.begin_typing()
    editor.set_advanced(True)
    assert not editor.x_input.capturekeystrokes and not editor.x_input.cursor.get_visible()
    old_x = editor.x_input.text
    editor.figure.canvas.callbacks.process('key_press_event', KeyEvent('key_press_event', editor.figure.canvas, key='9'))
    assert editor.x_input.text == old_x
    editor.horizon_input.set_val('not a number')
    editor.set_advanced(False)
    editor.select(len(editor.snapshots))
    assert editor.horizon_input.text == 'not a number'
    assert 'Horizon not a number s' in editor.mission_summary.get_text()
    assert editor.config is original


@pytest.mark.parametrize('value', ['nan', '10'])
def test_hidden_invalid_mission_settings_still_block_launch_transactionally(editor, value):
    original = editor.config, tuple(editor.snapshots), editor.target
    editor.set_advanced(True)
    editor.horizon_input.set_val(value)
    editor.set_advanced(False)
    editor.x_input.set_val('450')
    editor.start()
    assert not editor.accepted and plt.fignum_exists(editor.figure.number)
    assert (editor.config, tuple(editor.snapshots), editor.target) == original
    assert editor.status.get_color() == 'firebrick'


def test_hidden_valid_settings_are_applied_to_the_actual_launch(editor):
    # Select the reference explicitly to isolate the form from native availability.
    editor.config = replace(editor.config, numerical_backend='python')
    observed = []
    editor.launch_validator = lambda snapshots, config: observed.append((snapshots, config))
    editor.set_advanced(True)
    editor.horizon_input.set_val('220')
    editor.deadline_input.set_val('180')
    editor.post_arrival_input.set_val('12')
    editor.activation_input.set_val('45')
    editor.delay_min_input.set_val('2')
    editor.delay_max_input.set_val('4')
    editor.set_advanced(False)
    editor.start()
    assert editor.accepted and len(observed) == 1
    actual = observed[0][1]
    assert actual.horizon == 220 and actual.arrival_deadline == 180
    assert actual.post_arrival_time == 12 and actual.activation_time == 45
    assert (actual.min_extra_loiter, actual.max_extra_loiter) == (2, 4)
    assert actual.separation == editor.original_config.separation
    assert actual.position_error == editor.original_config.position_error
    assert actual.clock_error == editor.original_config.clock_error


def test_selection_exposes_only_relevant_fields_and_obstacle_dimensions(editor):
    editor.select(len(editor.snapshots))
    assert editor.altitude_input.ax.get_visible()
    assert not editor.heading_input.ax.get_visible() and not editor.speed_input.ax.get_visible()
    assert not editor.cruise_input.ax.get_visible()
    assert not editor.remove_obstacle_button.ax.get_visible()
    editor.add_obstacle('cylinder')
    assert editor.heading_input.ax.get_visible() and editor.cruise_input.ax.get_visible()
    assert not editor.speed_input.ax.get_visible() and not editor.speed_input.active
    assert editor.remove_obstacle_button.ax.get_visible()
    editor.add_obstacle('box')
    assert editor.speed_input.ax.get_visible() and editor.speed_input.active
    assert sum(w.ax.get_visible() for w in editor._inputs) == 6
    editor.set_advanced(True)
    assert not editor.cruise_input.ax.get_visible()
    assert not editor.profile_input.ax.get_visible() and not editor.profile_button.ax.get_visible()
    editor.select(0)
    assert editor.cruise_input.ax.get_visible() and editor.radius_input.ax.get_visible()
    assert not editor.profile_input.ax.get_visible()  # Fixed PX4 airframe stays fixed.


def test_summary_tracks_binding_and_pending_constraints_then_reset_restores_mission_view(editor):
    editor.deadline_input.set_val('160')
    editor.post_arrival_input.set_val('10')
    assert editor.apply_fields()
    editor.set_advanced(True)
    editor.delay_max_input.set_val('3')
    editor.horizon_input.set_val('230')
    editor.set_advanced(False)
    summary = editor.mission_summary.get_text()
    assert 'Arrival deadline: 160 s' in summary
    assert 'Minimum after arrival: 10 s' in summary
    assert 'Horizon 230 s' in summary and 'Extra wait 0–3 s' in summary
    assert 'Separation: 30 m' in summary
    editor.set_advanced(True)
    editor.reset()
    assert not editor.advanced and editor.config == editor.original_config
    assert editor.horizon_input.text == '200' and editor.deadline_input.text == 'Auto'
    assert 'Arrival deadline: none' in editor.mission_summary.get_text()
    assert not editor.horizon_input.ax.get_visible()


def test_planar_view_has_no_altitude_or_3d_profile_controls():
    config = Config()
    editor = ScenarioEditor(separate_loiter_snapshot(), config.target, config)
    try:
        assert not editor.altitude_input.ax.get_visible()
        assert sum(w.ax.get_visible() for w in editor._inputs) == 4
        editor.set_advanced(True)
        assert not editor.cruise_min_input.ax.get_visible()
        assert not editor.descent_input.ax.get_visible()
        assert not editor.generator.active and not editor.backend.active
    finally:
        plt.close(editor.figure)


def test_compact_obstacle_captions_preserve_geometry_and_historical_renderer_labels():
    import numpy as np
    from swarm_sim.trajectory2d.obstacle_view import draw_obstacles_2d
    from swarm_sim.trajectory2d.obstacles import Obstacle
    config = Config(obstacles=(Obstacle.cylinder(20, 30, 10, 0, 50),
                               Obstacle.box(60, 70, 0, 90, 100, 50)))
    figure, axes = plt.subplots(1, 2)
    try:
        draw_obstacles_2d(axes[0], config)
        draw_obstacles_2d(axes[1], config, compact_labels=True)
        assert [text.get_text() for text in axes[0].texts] == ['Obstacle 0', 'Obstacle 1']
        assert [text.get_text() for text in axes[1].texts] == ['O0', 'O1']
        assert len(axes[0].patches) == len(axes[1].patches) == 4
        for first, second in zip(axes[0].patches, axes[1].patches):
            np.testing.assert_array_equal(first.get_patch_transform().get_matrix(), second.get_patch_transform().get_matrix())
            np.testing.assert_array_equal(first.get_path().vertices, second.get_path().vertices)
    finally:
        plt.close(figure)


@pytest.mark.parametrize('advanced', [False, True])
def test_visible_controls_and_field_labels_do_not_overlap(editor, advanced):
    editor.set_advanced(advanced)
    editor.figure.canvas.draw()
    renderer = editor.figure.canvas.get_renderer()
    widgets = [*editor._view_widgets, editor.mission_button, editor.advanced_button,
               editor.selector, editor.apply_button, editor.fit_button, editor.reset_button,
               editor.start_button, editor.add_box_button, editor.add_cylinder_button,
               editor.remove_obstacle_button]
    visible = [w for w in widgets if w is not None and w.ax.get_visible()]
    for index, first in enumerate(visible):
        bounds = first.ax.get_window_extent(renderer)
        for second in visible[index+1:]:
            assert not bounds.overlaps(second.ax.get_window_extent(renderer))
    for field in editor._inputs:
        if not field.ax.get_visible():
            continue
        label = field.label.get_window_extent(renderer)
        assert editor.figure.bbox.contains(label.x0, label.y0)
        for other in visible:
            if other is not field:
                assert not label.overlaps(other.ax.get_window_extent(renderer)), field.label.get_text()


@pytest.mark.skipif(importlib.util.find_spec('swarm_sim._trajectory_native') is None, reason='native extension not built')
def test_switching_views_preserves_the_joint_plan_and_px4_launch_validation(editor):
    from swarm_sim.px4_fleet import prepare_fleet
    assert editor.apply_fields()  # Use the same field normalization as an actual launch.
    reference = prepare_fleet(tuple(editor.snapshots), editor.config, 'remove-at-target')
    observed = []
    def validate(snapshots, config):
        observed.append(prepare_fleet(snapshots, config, 'remove-at-target'))
    editor.launch_validator = validate
    editor.set_advanced(True)
    editor.set_advanced(False)
    editor.start()
    assert editor.accepted and len(observed) == 1
    assert observed[0] == reference
