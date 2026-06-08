"""Tube furnace PID/ramp controller — entry point.

Examples:
    # Ramp to 1100 °C at 20 °C/min and hold 30 min on the real furnace
    python main.py --port /dev/cu.usbserial-A9XUD4EB --target 1100 --ramp 20 --hold 30

    # Try the UI/menu with no hardware attached
    python main.py --simulate

    # Run a multi-segment recipe from disk
    python main.py --recipe recipes/my_profile.json
"""

import argparse
import sys

from furnace.config import Config
from furnace.controller import FurnaceController
from furnace.recipe import Recipe, build_simple


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Delta DTB tube furnace controller")
    p.add_argument("--port", help="serial port (default from Config)")
    p.add_argument("--target", type=float, help="target temperature °C")
    p.add_argument("--ramp", type=float, help="ramp rate °C/min")
    p.add_argument("--hold", type=float, help="hold time at target (min)")
    p.add_argument("--recipe", help="path to a recipe .json (overrides target/ramp/hold)")
    p.add_argument("--simulate", action="store_true",
                   help="run against a fake furnace (no hardware)")
    p.add_argument("--no-plot", action="store_true",
                   help="disable the live matplotlib plot window")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    cfg = Config()
    if args.port:
        cfg.port = args.port
    if args.target is not None:
        cfg.target_temp = args.target
    if args.ramp is not None:
        cfg.ramp_rate = args.ramp
    if args.hold is not None:
        cfg.hold_min = args.hold

    if args.recipe:
        recipe = Recipe.load(args.recipe)
        cfg.target_temp = recipe.final_target
    else:
        recipe = build_simple(cfg.target_temp, cfg.ramp_rate, cfg.hold_min,
                              name="default")

    if args.simulate:
        from furnace.simulator import FakeFurnace
        furnace = FakeFurnace(cfg)
    else:
        from furnace.delta_dtb import DeltaDTB
        try:
            furnace = DeltaDTB(cfg)
        except Exception as e:
            print(f"Could not open serial port {cfg.port}: {e}")
            print("Tip: use --simulate to try the UI without hardware.")
            sys.exit(1)

    from furnace.plot import make_plot
    plot = make_plot(cfg, enabled=cfg.plot and not args.no_plot)

    FurnaceController(furnace, cfg, recipe, plot=plot).run()


if __name__ == "__main__":
    main()
