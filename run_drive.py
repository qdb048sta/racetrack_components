#!/usr/bin/env python3
"""Build a physical closed-loop lap and a standalone interactive HTML replay.

Run from this directory: python run_drive.py
Existing interpolation CLI remains: python -m tracklab --output results
"""
import os
# Avoid oversubscribing small repeated nonlinear solves. User-set values take precedence.
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('MKL_NUM_THREADS','1')
from pathlib import Path
import argparse
import json
from dataclasses import replace

ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--track',type=Path,default=ROOT/'data'/'Norisring.csv')
    parser.add_argument('--output',type=Path,default=ROOT/'drive_results')
    parser.add_argument('--config',type=Path,default=None)
    parser.add_argument('--laps',type=float,default=None)
    parser.add_argument('--max-speed',type=float,default=None,help='Maximum planned speed, m/s; not a hard true-speed bound.')
    parser.add_argument('--initial-offset',type=float,default=None,help='Initial lateral offset left of the reference, m.')
    parser.add_argument('--no-independent-check',action='store_true')
    parser.add_argument('--no-plots',action='store_true')
    parser.add_argument('--open',action='store_true',dest='open_browser')
    args=parser.parse_args()
    try:
        import casadi  # noqa
    except ImportError as exc:
        raise SystemExit('Install driving dependencies: python -m pip install -r requirements-driving.txt') from exc
    from tracklab import PeriodicTrack
    from tracklab.driving.config import load_config
    from tracklab.driving.simulation import simulate
    from tracklab.driving.artifacts import save_results
    p,d=load_config(args.config)
    if args.laps is not None:d=replace(d,laps=args.laps)
    if args.max_speed is not None:d=replace(d,maximum_speed_mps=args.max_speed)
    if args.initial_offset is not None:d=replace(d,initial_lateral_offset_m=args.initial_offset)
    p.validate();d.validate()
    road=PeriodicTrack(args.track)
    print('Dynamic single-track + tracking NMPC. Not native C++; not minimum-time.',flush=True)
    print(f'Reference length: {road.length:.3f} m; lap target: {d.laps:g}',flush=True)
    def report(x):
        print(f"{100*x['progress_m']/x['target_m']:6.1f}% | t={x['time_s']:7.2f} s | "
              f"v={3.6*x['speed_mps']:5.1f} km/h | offset={x['e_left_m']:+.4f} m | solves={x['solves']}",flush=True)
    result=simulate(road,p,d,progress_callback=report)
    metrics=save_results(road,result,args.output,independent_check=not args.no_independent_check)
    if not args.no_plots:
        from tracklab.driving.plots import make_plots
        make_plots(road,args.output)
    replay=(args.output/'norisring_replay.html').resolve()
    print(json.dumps(metrics,indent=2));print(f'Open in browser: {replay}')
    if args.open_browser:
        import webbrowser
        webbrowser.open(replay.as_uri())

if __name__=='__main__':
    main()
