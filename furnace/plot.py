"""Live temperature plot in a separate matplotlib window.

The figure lives in its own GUI window, independent of the terminal dashboard.
It's driven from the control loop (main thread): :meth:`update` redraws (cheap,
throttled) and :meth:`pump` keeps the window responsive between redraws.

If no display / GUI backend is available, :func:`make_plot` returns a
:class:`NullPlot` no-op so the controller runs headless without complaint.
"""

import time


class NullPlot:
    """Does nothing — used when plotting is disabled or unavailable."""

    def update(self, times, temps, setpoints):
        pass

    def pump(self):
        pass

    def close(self):
        pass


class LivePlot:
    def __init__(self, config, update_s: float = 2.0):
        import matplotlib.pyplot as plt

        self._plt = plt
        self._update_s = update_s
        self._last = 0.0

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 4.5))
        self.fig.canvas.manager.set_window_title("Tube Furnace — live")
        (self.temp_line,) = self.ax.plot([], [], "r-", label="temp")
        (self.sp_line,) = self.ax.plot([], [], "b--", label="setpoint")
        self.ax.set_xlabel("Time (min)")
        self.ax.set_ylabel("Temperature (°C)")
        self.ax.set_title(f"Target {config.target_temp:.0f} °C")
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc="upper left")
        self.fig.tight_layout()
        self.fig.show()

    def update(self, times, temps, setpoints):
        now = time.monotonic()
        if now - self._last < self._update_s:
            return
        self._last = now
        self.temp_line.set_data(times, temps)
        self.sp_line.set_data(times, setpoints)
        self.ax.relim()
        self.ax.autoscale_view()
        try:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        except Exception:
            pass

    def pump(self):
        """Keep the window interactive without a full redraw."""
        try:
            self.fig.canvas.flush_events()
        except Exception:
            pass

    def close(self):
        try:
            self._plt.close(self.fig)
        except Exception:
            pass


def make_plot(config, enabled: bool = True):
    """Build a LivePlot, or a NullPlot if disabled / no GUI backend."""
    if not enabled:
        return NullPlot()
    try:
        return LivePlot(config, update_s=config.plot_update_s)
    except Exception as e:
        print(f"[plot] disabled (no GUI backend?): {e}")
        return NullPlot()
