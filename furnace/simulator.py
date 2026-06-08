"""A fake furnace for testing the UI/menu/control logic without hardware.

Implements the same method surface as :class:`DeltaDTB`, backed by a crude
first-order thermal model. Good enough to watch the dashboard ramp, exercise
the menu, and verify the safety net — not a physically accurate model.

Enable with ``--simulate`` on the command line.
"""

import time

from .config import Config


class FakeFurnace:
    # Model constants (tuned for a believable-looking ramp, not realism).
    _AMBIENT = 25.0
    _MAX_HEAT_RATE = 1.2     # °C/s at 100% power
    _COOL_COEFF = 0.0009     # natural loss per °C above ambient, per second

    def __init__(self, config: Config):
        self.cfg = config
        self.temp = self._AMBIENT
        self.setpoint = 0.0
        self.power = 0.0
        self.ctrl_method = 0          # 0=PID, 2=manual
        self.manual_power = 0.0
        self.p, self.i, self.d = 200.0, 100.0, 25.0
        self._auto_tune = False
        self._at_until = 0.0
        self._last = time.monotonic()

    def close(self):
        pass

    # --- model integration ---------------------------------------------- #
    def _advance(self):
        now = time.monotonic()
        dt = now - self._last
        self._last = now
        if dt <= 0:
            return

        if self._auto_tune and now >= self._at_until:
            self._auto_tune = False

        # Decide output power.
        if self.ctrl_method == 2:                       # manual
            self.power = self.manual_power
        else:                                           # simple P control to SV
            error = self.setpoint - self.temp
            band = max(self.p / 10.0, 1.0)              # P is a 0.1-unit prop. band
            self.power = max(0.0, min(100.0, (error / band) * 100.0))

        heat = (self.power / 100.0) * self._MAX_HEAT_RATE
        loss = self._COOL_COEFF * (self.temp - self._AMBIENT)
        self.temp += (heat - loss) * dt
        self.temp = max(self._AMBIENT, self.temp)

    # --- reads ----------------------------------------------------------- #
    def get_pv(self):
        self._advance()
        return round(self.temp, 1)

    def get_sv(self):
        return self.setpoint

    def get_power_percent(self):
        self._advance()
        return round(self.power, 1)

    def get_pid_values(self):
        return (self.p / 10.0, self.i, self.d)

    def is_auto_tuning(self):
        self._advance()
        return self._auto_tune

    # --- writes / commands ---------------------------------------------- #
    def set_setpoint(self, temp):
        self.setpoint = float(temp)
        return True

    def set_control_method(self, method):
        self.ctrl_method = int(method)
        return True

    def set_manual_power(self, percent):
        self.manual_power = float(percent)
        return True

    def set_pid(self, p, i, d):
        self.p, self.i, self.d = p * 10.0, i, d

    def set_hysteresis(self, hys):
        return True

    def start_auto_tune(self):
        self._auto_tune = True
        self._at_until = time.monotonic() + 8.0        # pretend AT takes 8 s
        return True

    def stop_auto_tune(self):
        self._auto_tune = False
        return True

    def run(self):
        return True

    def stop(self):
        return True

    def emergency_stop(self):
        self.ctrl_method = 2
        self.manual_power = 0.0
        self.power = 0.0

    def initialize(self):
        print("[sim] fake furnace ready (no hardware).")
