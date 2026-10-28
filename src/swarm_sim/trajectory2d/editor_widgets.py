"""Matplotlib editor widgets with coalesced, non-blocking repaint requests.

TextBox dispatches every canvas click to every input. Its default stop_typing
redraws synchronously even for inputs that never held focus. With many inputs,
one mouse click consequently paints the entire editor many times. Keep the
standard input/focus/callback behavior, but repaint only actual visual changes
and defer rendering to the GUI event loop.
"""

from matplotlib import colors
from matplotlib.widgets import TextBox


class DeferredTextBox(TextBox):
    def stop_typing(self):
        submitting = self.capturekeystrokes
        visible = self.cursor.get_visible()
        if not submitting and not visible:
            return
        if submitting:
            self._on_stop_typing()
            self._on_stop_typing = None
        self.capturekeystrokes = False
        self.cursor.set_visible(False)
        self.ax.figure.canvas.draw_idle()
        if submitting and self.eventson:
            self._observers.process('submit', self.text)

    def _motion(self, event):
        if self.ignore(event):
            return
        color = self.hovercolor if self.ax.contains(event)[0] else self.color
        if not colors.same_color(color, self.ax.get_facecolor()):
            self.ax.set_facecolor(color)
            if self.drawon:
                self.ax.figure.canvas.draw_idle()

    def _rendercursor(self):
        # Match TextBox's text-width cursor placement without its synchronous
        # full-figure draw on every keypress/click. The renderer can measure
        # text before presentation; draw_idle commits the final text and cursor.
        renderer = self.ax.figure._get_renderer()
        text = self.text
        prefix = text[:self.cursor_index]
        bounds = self.text_disp.get_window_extent(renderer)
        try:
            self.text_disp.set_text(prefix or ',')
            width = self.text_disp.get_window_extent(renderer)
        finally:
            self.text_disp.set_text(text)
        if bounds.y0 == bounds.y1:
            bounds.y0 -= width.height / 2
            bounds.y1 += width.height / 2
        x = bounds.x0 + (width.width if prefix else 0)
        self.cursor.set(segments=[[(x, bounds.y0), (x, bounds.y1)]], visible=True)
        self.ax.figure.canvas.draw_idle()
