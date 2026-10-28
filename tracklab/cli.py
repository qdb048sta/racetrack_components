"""Command-line orchestration; computation remains in reusable modules."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from .evaluation import run_experiment
from .export import json_clean
from .plotting import plot_results

def main() -> None:
    p=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--data',type=Path,default=Path(__file__).resolve().parents[1] / 'data' / 'Norisring.csv')
    p.add_argument('--output',type=Path,default=Path('results'))
    p.add_argument('--stride',type=int,default=4)
    p.add_argument('--smoothing-lambda',type=float,default=100.)
    p.add_argument('--noise-std',type=float,default=.2)
    p.add_argument('--noise-repeats',type=int,default=10)
    p.add_argument('--no-plots',action='store_true')
    args=p.parse_args()
    result=run_experiment(args.data,args.output,args.stride,args.smoothing_lambda,args.noise_std,args.noise_repeats)
    if not args.no_plots:plot_results(args.data,args.output)
    print(json.dumps(json_clean(result),indent=2))
