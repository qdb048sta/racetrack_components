#!/usr/bin/env python3
"""Run matched-parameter double-track plants on the existing Norisring road."""
import os
for variable in ("OPENBLAS_NUM_THREADS","OMP_NUM_THREADS","MKL_NUM_THREADS"):
    os.environ[variable]="1"
from dataclasses import replace
from pathlib import Path
import argparse


def main():
    from tracklab.model_comparison.benchmark import MODEL_NAMES,run_one,collect,ROOT
    from tracklab.model_comparison.controller import Experiment
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models",nargs="+",choices=MODEL_NAMES,default=list(MODEL_NAMES))
    parser.add_argument("--scenario",choices=("nominal","demanding","both"),default="both")
    parser.add_argument("--output",type=Path,default=ROOT/"outputs/model_comparison")
    parser.add_argument("--track",type=Path,default=ROOT/"data/Norisring.csv")
    parser.add_argument("--laps",type=float,default=1.)
    parser.add_argument("--maximum-time",type=float,default=600.)
    parser.add_argument("--collect-only",action="store_true")
    args=parser.parse_args()
    if args.collect_only:
        collect(args.output);return
    scenarios=("nominal","demanding") if args.scenario=="both" else (args.scenario,)
    for scenario in scenarios:
        cfg=Experiment(name=scenario,laps=args.laps,maximum_time_s=args.maximum_time)
        if scenario=="demanding":
            cfg=replace(cfg,maximum_speed_mps=30.,lateral_acceleration_budget_mps2=6.)
        for model in args.models:
            run_one(model,cfg,args.output,args.track)
    collect(args.output)


if __name__=="__main__":
    main()
