"""Deferred rendering must preserve TextBox cursor and focus semantics."""

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox
import numpy as np
import pytest

from swarm_sim.trajectory2d.editor_widgets import DeferredTextBox


@pytest.mark.parametrize('text', ['', '123.45', '-90', 'Auto'])
def test_cursor_matches_matplotlib_without_synchronous_paint(text, monkeypatch):
    figures = [plt.figure(figsize=(4, 2)) for _ in range(2)]
    try:
        standard, deferred = [cls(fig.add_axes([.2, .3, .6, .3]), 'x', initial=text)
                              for cls, fig in zip((TextBox, DeferredTextBox), figures)]
        for fig in figures:
            fig.canvas.draw()
        queued = []
        monkeypatch.setattr(figures[1].canvas, 'draw_idle', lambda: queued.append(True))

        def forbidden_draw():
            pytest.fail('Cursor refresh must not paint synchronously')

        monkeypatch.setattr(figures[1].canvas, 'draw', forbidden_draw)
        for index in range(len(text)+1):
            standard.cursor_index = deferred.cursor_index = index
            standard._rendercursor()
            deferred._rendercursor()
            np.testing.assert_allclose(deferred.cursor.get_segments(), standard.cursor.get_segments())
            assert deferred.text == text and deferred.cursor.get_visible()
        assert len(queued) == len(text)+1
    finally:
        for fig in figures:
            plt.close(fig)


def test_stop_typing_submits_once_and_restores_keyboard_shortcuts(monkeypatch):
    fig = plt.figure()
    original_keymaps = {key: list(value) for key, value in matplotlib.rcParams.items()
                        if key.startswith('keymap.')}
    try:
        widget = DeferredTextBox(fig.add_axes([.2, .3, .6, .3]), 'x', initial='42')
        submitted, queued = [], []
        widget.on_submit(submitted.append)
        monkeypatch.setattr(fig.canvas, 'draw_idle', lambda: queued.append(True))
        widget.stop_typing()
        assert not queued and not submitted
        widget.begin_typing()
        widget.stop_typing()
        widget.stop_typing()
        assert submitted == ['42'] and len(queued) == 1
        assert not widget.capturekeystrokes and not widget.cursor.get_visible()
        assert {key: list(matplotlib.rcParams[key]) for key in original_keymaps} == original_keymaps
    finally:
        plt.close(fig)
