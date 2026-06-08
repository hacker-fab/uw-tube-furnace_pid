"""Configuration for the furnace controller.

One flat dataclass. Tweak defaults here or override on the command line
(see main.py). All temperatures are °C, rates are °C/min, times are minutes
unless the field name says otherwise.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class Config:
    # --- Serial / Modbus ---
    port: str = "/dev/cu.usbserial-A9XUD4EB"
    baud: int = 9600
    slave_id: int = 1

    # --- Main loop timing ---
    sample_time_s: float = 1.0      # how often we read/log/command the furnace
    relay_period_s: int = 30        # DTB relay duty-cycle window (element life)

    # --- Target / ramp (the default single-segment recipe) ---
    target_temp: float = 1100.0     # where we want to end up
    ramp_rate: float = 20.0         # °C/min setpoint climb (faster => more thermal stress)
    hold_min: float = 30.0          # hold time once target is reached

    # --- Hardware temperature range (written to the DTB) ---
    temp_limit_low: float = 0.0
    temp_limit_high: float = 1150.0
    sensor_type: int = 0            # 0 = K-type thermocouple

    # --- Software safety net (independent of the DTB's own limits) ---
    safety_limit: float = 1120.0    # force 0% power above this PV
    safety_hys: float = 5.0         # must drop this far below before resuming
    max_rate: float = 35.0          # force 0% power above this heating rate
    rate_hys: float = 5.0

    # --- Cooling ---
    safe_cool_rate: float = 10.0    # °C/min setpoint walk-down during cooldown
    cool_target: float = 40.0       # cut power once setpoint reaches here

    # --- Rate estimation ---
    rate_window: int = 15           # samples used for the dT/dt regression

    # --- PID overrides (None => leave whatever is stored on the controller) ---
    kp: Optional[float] = None
    ki: Optional[float] = None
    kd: Optional[float] = None

    # --- Auto-tune ---
    auto_tune_safety: float = 50.0  # AT setpoint must be within this of current temp

    # --- Live plot ---
    plot: bool = True               # show the separate matplotlib window
    plot_update_s: float = 2.0      # how often to redraw the plot
    plot_history_s: float = 3600.0  # how much history to keep on the plot

    # --- IO ---
    csv_path: str = "furnace_log.csv"
    recipes_dir: str = "recipes"
