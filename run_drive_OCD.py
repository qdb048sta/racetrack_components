#!/usr/bin/env python3
"""Run native TUM single-track/linear-tire tracking and save a standalone replay."""
import os
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
from pathlib import Path
from dataclasses import replace
import argparse
import json

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track", type=Path, default=ROOT/"data"/"Norisring.csv")
    parser.add_argument("--output", type=Path, default=ROOT/"drive_OCD_results")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--laps", type=float)
    parser.add_argument("--max-speed", type=float)
    parser.add_argument("--initial-offset", type=float)
    parser.add_argument("--no-independent-check", action="store_true")
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--open", action="store_true", dest="open_browser")
    args = parser.parse_args()
    from tracklab import PeriodicTrack
    from tracklab.driving_OCD.config import load_config
    from tracklab.driving_OCD.simulation import simulate
    from tracklab.driving_OCD.artifacts import save_results
    p, d = load_config(args.config)
    for arg, field in ((args.laps, "laps"), (args.max_speed, "maximum_speed_mps"), (args.initial_offset, "initial_lateral_offset_m")):
        if arg is not None:
            d = replace(d, **{field: arg})
    p.validate(); d.validate()
    road = PeriodicTrack(args.track)
    print("Native TUM OCD: WHEEL_TORQUE / PT1 / SINGLE_TRACK / LINEAR / DEFAULT", flush=True)
    print(f"Reference length: {road.length:.3f} m; target: {d.laps:g} laps", flush=True)
    def report(x):
        print(f"{100*x['progress_m']/x['target_m']:6.1f}% | t={x['time_s']:7.2f} s | "
              f"v={3.6*x['speed_mps']:5.1f} km/h | offset={x['e_left_m']:+.4f} m | solves={x['solves']}", flush=True)
    result = simulate(road, p, d, progress_callback=report)
    metrics = save_results(road, result, args.output, independent_check=not args.no_independent_check)
    if not args.no_plots:
        from tracklab.driving_OCD.plots import make_plots
        make_plots(road, args.output)
    replay = (args.output/"norisring_replay.html").resolve()
    print(json.dumps(metrics, indent=2))
    print(f"Replay: {replay}")
    if args.open_browser:
        import webbrowser
        webbrowser.open(replay.as_uri())


if __name__ == "__main__":
    main()
