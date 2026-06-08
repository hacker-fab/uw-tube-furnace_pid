"""Supervisory controller: software setpoint-ramp, safety net, logging, menu.

The furnace runs its own PID loop. Every tick we read the temperature, nudge
the setpoint toward the active recipe's target at the configured ramp rate,
enforce a software over-temp / over-rate cutout, log to CSV, and service a
non-blocking live menu. The control loop never blocks on input — menu and
typing are drained from a queue between ticks so safety checks keep running.
"""

import csv
import time
from collections import deque

from rich.console import Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table

from .config import Config
from .keyboard import KeyboardListener
from .plot import NullPlot
from .recipe import Recipe, Segment, build_simple, list_recipes

# Keys we treat specially regardless of platform.
_ENTER = ("\r", "\n")
_BACKSPACE = ("\x7f", "\x08")
_ESC = ("\x1b",)
_INPUT_CHARS = set("0123456789.,;- ")

MENU_TEXT = (
    "[bold]MENU[/]  (m/esc) close  |  (p) pause/resume  |  (c) cooldown  |  "
    "(x) E-STOP  |  (r) reload recipe  |  (t) set target  |  (g) ramp rate  |  "
    "(d) set PID  |  (a) auto-tune  |  (q) quit"
)


class FurnaceController:
    def __init__(self, furnace, config: Config, recipe: Recipe, plot=None):
        self.furnace = furnace
        self.cfg = config
        self.plot = plot if plot is not None else NullPlot()

        # Telemetry
        self.temp = None
        self.power = 0.0
        self.rate = 0.0
        self.setpoint = 0.0
        self.kp = self.ki = self.kd = 0.0
        self.now_s = 0.0
        self._t0 = time.monotonic()
        self._hist = deque(maxlen=max(2, config.rate_window))

        # Plot history (longer than the rate window): minutes, temp, setpoint
        plot_len = max(2, int(config.plot_history_s / max(config.sample_time_s, 0.1)))
        self._pt_t = deque(maxlen=plot_len)
        self._pt_temp = deque(maxlen=plot_len)
        self._pt_sp = deque(maxlen=plot_len)

        # Trajectory state
        self.segments = list(recipe.segments)
        self.recipe_name = recipe.name
        self.seg_idx = 0
        self.seg_phase = "ramp"          # "ramp" | "hold"
        self._last_sp_t = time.monotonic()
        self._hold_t = time.monotonic()
        self._gov = 1.0                  # current governor factor (1=full, 0=frozen)
        self.finished = False

        # Mode flags
        self.running = True
        self.paused = False
        self.cooling = False
        self.estopped = False
        self.tripped = False
        self.tuning = False
        self._was_tuning = False
        self._cool_done = False

        # UI
        self.menu_open = False
        self.input = None                # {"prompt", "buffer", "cb"}
        self._logs = deque(maxlen=10)
        self._recipe_files = []

        self._csv_writer = None
        self._csv_file = None

    # ------------------------------------------------------------------ #
    # Logging helpers
    # ------------------------------------------------------------------ #
    def log(self, msg: str):
        self._logs.append(f"{time.strftime('%H:%M:%S')}  {msg}")

    def status(self) -> str:
        if self.tripped:
            return "SAFETY-HOLD"
        if self.estopped:
            return "E-STOP"
        if self.tuning:
            return "AUTO-TUNE"
        if self.paused:
            return "PAUSED"
        if self.cooling:
            return "COOLING"
        if self.finished:
            return "OPERATING"
        return self.seg_phase.upper()    # RAMP | HOLD

    # ------------------------------------------------------------------ #
    # Trajectory
    # ------------------------------------------------------------------ #
    def _anchor_here(self):
        """(Re)start the current segment ramp from the current temperature.

        Used on load/resume. Setting the setpoint to the measured temp means
        resuming never *jumps* the setpoint upward — the tracking clamp then
        eases it back out, so there's no discontinuity after a safety event.
        """
        self.setpoint = self.temp if self.temp is not None else 25.0
        self._last_sp_t = time.monotonic()
        self.seg_phase = "ramp"

    def load_recipe(self, recipe: Recipe):
        self.segments = list(recipe.segments)
        self.recipe_name = recipe.name
        self.seg_idx = 0
        self.finished = False
        self.cooling = False
        self.estopped = False
        self.paused = False
        self._cool_done = False
        self._anchor_here()

    def _advance_segment(self, now):
        self.seg_idx += 1
        if self.seg_idx < len(self.segments):
            # Continue from the previous target (= current setpoint), not the
            # measured temp, so consecutive ramps chain smoothly.
            self._last_sp_t = now
            self.seg_phase = "ramp"
            seg = self.segments[self.seg_idx]
            self.log(f"segment {self.seg_idx}: ramp to {seg.target:.0f}°C "
                     f"@ {seg.ramp_rate:.0f}°C/min")
        else:
            self.finished = True
            if self.cooling:
                self.log("cooldown setpoint reached - cutting power")
            else:
                self.log("recipe complete - holding at target (OPERATING)")

    def _governor_factor(self) -> float:
        """Scale 1->0 as the measured heating rate climbs into the governor band.

        Below ``rate_throttle_frac * max_rate`` the setpoint advances at full
        speed; at ``max_rate`` it's frozen. This eases the rate into a ceiling
        instead of slamming power off at a hard threshold. Cooling (negative
        rate) is never throttled.
        """
        c = self.cfg
        soft = c.max_rate * c.rate_throttle_frac
        if self.rate <= soft:
            return 1.0
        if self.rate >= c.max_rate:
            return 0.0
        return (c.max_rate - self.rate) / (c.max_rate - soft)

    def _update_setpoint(self):
        now = time.monotonic()
        dt_min = max(0.0, (now - self._last_sp_t) / 60.0)
        self._last_sp_t = now
        if self.finished or self.seg_idx >= len(self.segments):
            self.finished = True
            self._gov = 1.0
            return
        seg = self.segments[self.seg_idx]

        if self.seg_phase == "ramp":
            # Advance the setpoint incrementally, throttled by the governor.
            self._gov = self._governor_factor()
            step = seg.ramp_rate * dt_min * self._gov
            lead = self.cfg.setpoint_lead
            if seg.target >= self.setpoint:                 # ramping up
                sp = self.setpoint + step
                if self.temp is not None:                   # tracking clamp (anti-windup)
                    sp = min(sp, self.temp + lead)
                sp = min(sp, seg.target)
                reached = sp >= seg.target - 0.05
            else:                                           # ramping down
                sp = self.setpoint - step
                if self.temp is not None:
                    sp = max(sp, self.temp - lead)
                sp = max(sp, seg.target)
                reached = sp <= seg.target + 0.05
            self.setpoint = sp
            if reached:
                self.setpoint = seg.target
                self.seg_phase = "hold"
                self._hold_t = now
        else:  # hold
            self._gov = 1.0
            self.setpoint = seg.target
            # Only count hold time once the temperature has actually settled.
            if (self.temp is not None
                    and abs(self.temp - seg.target) > self.cfg.hold_band):
                self._hold_t = now
            elif (now - self._hold_t) / 60.0 >= seg.hold_min:
                self._advance_segment(now)

    # ------------------------------------------------------------------ #
    # Safety
    # ------------------------------------------------------------------ #
    def _check_safety(self) -> bool:
        """Return True if the safety net is currently overriding control."""
        c = self.cfg
        if self.tripped:
            if (self.temp < c.safety_limit - c.safety_hys
                    and self.rate < c.emergency_rate - c.rate_hys):
                self.tripped = False
                self.furnace.set_control_method(0)
                self._anchor_here()      # resume from current temp (no jump)
                self.log(f"safe again ({self.temp:.1f}°C, {self.rate:.1f}°C/min) "
                         "- resuming")
                return False
            return True
        if self.temp >= c.safety_limit or self.rate >= c.emergency_rate:
            self.tripped = True
            self.furnace.emergency_stop()
            self.log(f"[!] EMERGENCY CUT {self.temp:.1f}°C {self.rate:+.1f}°C/min "
                     "- power off")
            return True
        return False

    # ------------------------------------------------------------------ #
    # Per-tick update
    # ------------------------------------------------------------------ #
    def _rate(self) -> float:
        """Least-squares dT/dt over the recent window, in °C/min."""
        if len(self._hist) < 2:
            return 0.0
        xs = [p[0] for p in self._hist]
        ys = [p[1] for p in self._hist]
        n = len(xs)
        mx = sum(xs) / n
        my = sum(ys) / n
        denom = sum((x - mx) ** 2 for x in xs)
        if denom == 0:
            return 0.0
        slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / denom
        return slope * 60.0

    def tick(self):
        temp = self.furnace.get_pv()
        self.power = self.furnace.get_power_percent()
        if temp is None:
            self.log("PV read failed - holding last state")
            return
        self.temp = temp
        self.now_s = time.monotonic() - self._t0
        self._hist.append((self.now_s, temp))
        self.rate = self._rate()

        # 1) Safety overrides everything.
        if self._check_safety():
            self._write_csv()
            return

        # 2) Auto-tune: let the hardware run it, then pick up the new gains.
        self.tuning = self.furnace.is_auto_tuning()
        if self._was_tuning and not self.tuning:
            self.kp, self.ki, self.kd = self.furnace.get_pid_values()
            self.furnace.set_control_method(0)
            self._anchor_here()
            self.log(f"auto-tune done: P={self.kp} I={self.ki} D={self.kd}")
        self._was_tuning = self.tuning
        if self.tuning:
            self._write_csv()
            return

        # 3) Paused / e-stopped: keep monitoring, command nothing.
        if self.paused or self.estopped:
            self._write_csv()
            return

        # 4) Normal setpoint ramp.
        self._update_setpoint()
        if self.cooling and self.finished and not self._cool_done:
            self.furnace.emergency_stop()
            self._cool_done = True
            self.log("cooldown complete - heater off")
        elif not (self.cooling and self.finished):
            self.furnace.set_setpoint(self.setpoint)
        self._write_csv()

    # ------------------------------------------------------------------ #
    # Menu / input
    # ------------------------------------------------------------------ #
    def handle_key(self, key: str):
        if self.input is not None:
            self._handle_input_key(key)
            return

        k = key.lower() if len(key) == 1 else key
        if not self.menu_open:
            if k == "m":
                self.menu_open = True
                self.log("menu opened")
            return

        if k == "m" or key in _ESC:
            self.menu_open = False
            self.log("menu closed")
        elif k == "p":
            self.paused = not self.paused
            if not self.paused:
                self._anchor_here()
            self.log("paused" if self.paused else "resumed")
        elif k == "c":
            self._start_cooldown()
        elif k == "x":
            self._estop()
        elif k == "r":
            self._menu_reload()
        elif k == "t":
            self._ask(f"new target °C [{self.cfg.target_temp:.0f}]: ", self._cb_target)
        elif k == "g":
            self._ask(f"new ramp rate °C/min [{self.cfg.ramp_rate:.0f}]: ",
                      self._cb_ramp)
        elif k == "d":
            self._ask(f"PID as P,I,D [{self.kp},{self.ki},{self.kd}]: ", self._cb_pid)
        elif k == "a":
            self._ask(f"auto-tune setpoint °C [{self.temp:.0f}]: ", self._cb_autotune)
        elif k == "q":
            self.running = False
        # any other key: ignore, leave menu open

    def _handle_input_key(self, key):
        if key in _ENTER:
            buf = self.input["buffer"]
            cb = self.input["cb"]
            self.input = None
            cb(buf)
        elif key in _ESC:
            self.input = None
            self.log("input cancelled")
        elif key in _BACKSPACE:
            self.input["buffer"] = self.input["buffer"][:-1]
        elif len(key) == 1 and key in _INPUT_CHARS:
            self.input["buffer"] += key

    def _ask(self, prompt, cb):
        self.input = {"prompt": prompt, "buffer": "", "cb": cb}

    # --- menu actions --------------------------------------------------- #
    def _start_cooldown(self):
        self.cooling = True
        self.finished = False
        self._cool_done = False
        self.paused = False
        self.estopped = False
        self.segments = [Segment(self.cfg.cool_target, self.cfg.safe_cool_rate, 0)]
        self.seg_idx = 0
        self.furnace.set_control_method(0)
        self._anchor_here()
        self.menu_open = False
        self.log(f"cooldown started -> {self.cfg.cool_target:.0f}°C "
                 f"@ {self.cfg.safe_cool_rate:.0f}°C/min")

    def _estop(self):
        self.furnace.emergency_stop()
        self.furnace.stop()
        self.estopped = True
        self.menu_open = False
        self.log("[!] E-STOP - heater off. Reload a recipe to resume.")

    def _menu_reload(self):
        files = list_recipes(self.cfg.recipes_dir)
        if not files:
            self.log(f"no recipes found in {self.cfg.recipes_dir}/")
            return
        self._recipe_files = files
        listing = "  ".join(f"{i}:{f.split('/')[-1]}" for i, f in enumerate(files))
        self._ask(f"recipe # [{listing}]: ", self._cb_reload)

    def _cb_reload(self, buf):
        try:
            recipe = Recipe.load(self._recipe_files[int(buf.strip())])
        except Exception as e:
            self.log(f"reload failed: {e}")
            return
        self.load_recipe(recipe)
        self.furnace.set_control_method(0)
        self.menu_open = False
        self.log(f"loaded '{recipe.name}' ({len(recipe.segments)} segments)")

    def _cb_target(self, buf):
        try:
            target = float(buf.strip())
        except ValueError:
            self.log("invalid target")
            return
        self.cfg.target_temp = target
        self.load_recipe(build_simple(target, self.cfg.ramp_rate, self.cfg.hold_min))
        self.menu_open = False
        self.log(f"new target {target:.0f}°C @ {self.cfg.ramp_rate:.0f}°C/min")

    def _cb_ramp(self, buf):
        try:
            rate = abs(float(buf.strip()))
        except ValueError:
            self.log("invalid rate")
            return
        if rate > self.cfg.max_rate:
            self.log(f"rate {rate} exceeds safety max {self.cfg.max_rate} - clamped")
            rate = self.cfg.max_rate
        self.cfg.ramp_rate = rate
        for seg in self.segments:
            seg.ramp_rate = rate
        self._anchor_here()
        self.menu_open = False
        self.log(f"ramp rate -> {rate:.0f}°C/min")

    def _cb_pid(self, buf):
        try:
            p, i, d = (float(x) for x in buf.replace(";", ",").split(","))
        except ValueError:
            self.log("invalid PID (need P,I,D)")
            return
        self.kp, self.ki, self.kd = p, i, d
        self.furnace.set_pid(p, i, d)
        self.menu_open = False
        self.log(f"PID set: P={p} I={i} D={d}")

    def _cb_autotune(self, buf):
        try:
            sp = float(buf.strip())
        except ValueError:
            self.log("invalid setpoint")
            return
        if abs(sp - self.temp) > self.cfg.auto_tune_safety:
            self.log(f"setpoint must be within {self.cfg.auto_tune_safety}°C of "
                     f"current ({self.temp:.0f}°C)")
            return
        self.furnace.set_control_method(0)
        self.furnace.set_setpoint(sp)
        self.furnace.start_auto_tune()
        self.menu_open = False
        self.log(f"auto-tune started at {sp:.0f}°C")

    # ------------------------------------------------------------------ #
    # Rendering
    # ------------------------------------------------------------------ #
    def render(self):
        t = Table(show_header=False, box=None, padding=(0, 1))
        t.add_column(justify="right", style="cyan")
        t.add_column()
        temp = f"{self.temp:.1f}" if self.temp is not None else "--"
        seg = (f"{self.seg_idx + 1}/{len(self.segments)}"
               if self.segments else "-")
        t.add_row("status", f"[bold]{self.status()}[/]   recipe: {self.recipe_name}"
                            f"   segment: {seg}")
        t.add_row("temp", f"[bold yellow]{temp} °C[/]   "
                          f"setpoint {self.setpoint:.1f} °C   "
                          f"target {self.cfg.target_temp:.0f} °C")
        gov = "" if self._gov >= 0.99 else f"   [yellow]rate-gov {self._gov * 100:.0f}%[/]"
        t.add_row("rate", f"{self.rate:+.2f} °C/min   power {self.power:.0f}%"
                          f"   (limit {self.cfg.max_rate:.0f}){gov}")
        t.add_row("pid", f"P={self.kp}  I={self.ki}  D={self.kd}")
        t.add_row("time", f"{self.now_s:.0f} s")

        body = [Panel(t, title="Tube Furnace", border_style="blue")]

        if self.input is not None:
            body.append(Panel(f"{self.input['prompt']}{self.input['buffer']}_",
                              border_style="magenta", title="input"))
        elif self.menu_open:
            body.append(Panel(MENU_TEXT, border_style="green"))
        else:
            body.append(Panel("press [bold]m[/] for menu",
                              border_style="grey37"))

        if self._logs:
            body.append(Panel("\n".join(self._logs), title="log",
                              border_style="grey37"))
        return Group(*body)

    # ------------------------------------------------------------------ #
    # CSV
    # ------------------------------------------------------------------ #
    _CSV_HEADER = ["time_s", "temp_C", "setpoint_C", "rate_C_min", "power_pct",
                   "P", "I", "D", "status", "segment"]

    def _record_sample(self):
        """Append the current sample to the plot history and redraw (throttled)."""
        if self.temp is None:
            return
        self._pt_t.append(self.now_s / 60.0)
        self._pt_temp.append(self.temp)
        self._pt_sp.append(self.setpoint)
        self.plot.update(self._pt_t, self._pt_temp, self._pt_sp)

    def _write_csv(self):
        self._record_sample()
        if not self._csv_writer:
            return
        self._csv_writer.writerow([
            f"{self.now_s:.2f}",
            self.temp if self.temp is not None else "",
            f"{self.setpoint:.1f}",
            f"{self.rate:.3f}",
            f"{self.power:.1f}",
            self.kp, self.ki, self.kd,
            self.status(),
            self.seg_idx,
        ])
        self._csv_file.flush()

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #
    def run(self):
        self.furnace.initialize()
        self.kp, self.ki, self.kd = self.furnace.get_pid_values()
        if self.cfg.kp is not None:
            self.furnace.set_pid(self.cfg.kp, self.cfg.ki or 0, self.cfg.kd or 0)
            self.kp, self.ki, self.kd = self.cfg.kp, self.cfg.ki or 0, self.cfg.kd or 0

        # Seed temp + anchor before the first ramp command.
        self.temp = self.furnace.get_pv()
        self._anchor_here()
        self.log(f"started: recipe '{self.recipe_name}', "
                 f"{len(self.segments)} segment(s)")

        kb = KeyboardListener().start()
        self._csv_file = open(self.cfg.csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(self._CSV_HEADER)

        next_tick = time.monotonic()
        try:
            with Live(self.render(), refresh_per_second=8, screen=False) as live:
                while self.running:
                    key = kb.get()
                    while key is not None:
                        self.handle_key(key)
                        key = kb.get()

                    now = time.monotonic()
                    if now >= next_tick:
                        self.tick()
                        next_tick += self.cfg.sample_time_s
                        if next_tick < now:        # fell behind; catch up
                            next_tick = now + self.cfg.sample_time_s

                    live.update(self.render())
                    self.plot.pump()
                    time.sleep(0.03)
        except KeyboardInterrupt:
            self.log("Ctrl+C - cooling down")
            self._start_cooldown()
        finally:
            kb.stop()
            self.plot.close()
            if self._csv_file:
                self._csv_file.close()
            self.furnace.close()
            print(f"\nLog written to {self.cfg.csv_path}")
            print("Note: hardware was left under its own PID control at the last "
                  "setpoint. Power it down at the unit if you're done.")
