# UW Tube Furnace PID Controller

Simple, supervised PID/ramp control for a tube furnace driven by a **Delta DTB**
temperature controller over Modbus-ASCII (RS-485 / USB serial).

The furnace runs its own PID loop. This program **walks the setpoint** toward a
target at a configured ramp rate (°C/min), so you get a controlled climb to
e.g. 1100 °C instead of slamming full power. On top of that it provides a
software over-temp / over-rate safety cutout, CSV logging, a live dashboard, and
a non-blocking menu for changing things (including reloading recipes) while it
runs.

## Install

```bash
uv sync
```

## Run

```bash
# Ramp to 1100 °C at 20 °C/min, hold 30 min, on the real furnace
uv run python main.py --port /dev/cu.usbserial-A9XUD4EB --target 1100 --ramp 20 --hold 30

# Run a multi-segment recipe from disk
uv run python main.py --recipe recipes/example_1100.json

# Try the dashboard + menu with NO hardware attached
uv run python main.py --simulate

# Disable the live plot window (terminal dashboard only)
uv run python main.py --recipe recipes/example_1100.json --no-plot
```

A live temperature plot (measured temp + setpoint vs. time) opens in a separate
matplotlib window alongside the terminal dashboard. It degrades to a no-op if no
GUI backend is available; turn it off with `--no-plot`.

Before connecting, on the DTB itself: hold **SET**, cycle to `CoSH`, and enable
**Communication Write Enable** so the controller accepts writes.

Defaults (port, limits, safety thresholds, PID overrides) live in
[furnace/config.py](furnace/config.py).

## Live menu

Press **`m`** to open the menu. Everything is non-blocking — safety checks and
logging keep running while a menu/prompt is open.

| Key | Action |
|-----|--------|
| `p` | pause / resume the ramp |
| `c` | cool down (walk setpoint down at the safe rate, then cut power) |
| `x` | **E-STOP** — manual mode, 0 % power |
| `r` | reload a recipe from `recipes/` *(dynamic, while running)* |
| `t` | set a new target temperature |
| `g` | change the ramp rate |
| `d` | set PID gains (`P,I,D`) |
| `a` | start a hardware auto-tune |
| `q` | quit (leaves hardware under its own PID at the last setpoint) |

`Ctrl+C` triggers a controlled cooldown rather than a hard stop.

## Recipes

A recipe is a list of ramp/hold segments (see
[recipes/example_1100.json](recipes/example_1100.json)):

```json
{
  "name": "to-1100",
  "segments": [
    {"target": 600,  "ramp_rate": 25, "hold_min": 5},
    {"target": 1100, "ramp_rate": 15, "hold_min": 60}
  ]
}
```

Each segment ramps the setpoint from wherever it currently is to `target` at
`ramp_rate` °C/min, then holds for `hold_min` minutes before moving on.

## Safety

Independent of the DTB's own limits, the supervisor forces **manual mode at 0 %
power** if the temperature exceeds `safety_limit` (default 1120 °C) or the
heating rate exceeds `max_rate` (default 35 °C/min). It auto-resumes once both
fall back below the thresholds (minus hysteresis), re-anchoring the ramp from the
current temperature so the PID doesn't slam.

## Layout

```
main.py                 entry point / CLI
furnace/
  config.py             all tunables (one dataclass)
  delta_dtb.py          Modbus-ASCII driver for the Delta DTB
  recipe.py             ramp/hold segments + JSON load/save
  keyboard.py           threaded non-blocking key reader
  plot.py               live matplotlib plot in a separate window
  simulator.py          fake furnace for --simulate
  controller.py         supervisory loop: ramp, safety, logging, menu
check_ready.py          pre-run stability check (standalone)
get_temp.py             one-shot temperature read (standalone)
```

## Notes

- The old single-file `tube_furnace_controller.py` (with the heating-coil ML
  model and pattern-memory path-finding) has been replaced by this package.
- Output is logged to `furnace_log.csv`.
