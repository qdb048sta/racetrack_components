#!/usr/bin/env python3
"""Compare normalized source-family plants, with one fixed controller/track."""
import os
for variable in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'): os.environ[variable]='1'
from dataclasses import replace
from pathlib import Path
import argparse
import json
from tracklab.model_comparison.benchmark import ROOT,run_one,collect,json_write
from tracklab.model_comparison.controller import Experiment
from tracklab.model_comparison.thesis_models import MODEL_NAMES,LABELS,make_plant


def run_model(name,cfg,output,track):
    summary=run_one(name,cfg,output,track,plant_factory=make_plant,label=LABELS[name])
    folder=Path(output)/cfg.name/name
    if name!='native_ocd':
        plant=make_plant(name,cfg);info=plant.model_info;plant.close()
        capabilities=info['capabilities']
        if not capabilities['longitudinal_slip']: summary['max_abs_longitudinal_slip']=None
        if not capabilities['force_circle_metric']:
            if name=='liniger_bicycle':
                summary['max_effective_axle_force_ratio']=summary['max_friction_circle_utilization']
            summary['max_friction_circle_utilization']=None;summary['max_force_circle_excess_N']=None
        if not capabilities['tire_forces']:
            for key in ('min_wheel_load_N','max_wheel_load_N','max_abs_tire_slip_deg'): summary[key]=None
        summary['state_dimension']=info['state_dimension']
        summary['model_interpretation']='Normalized structural adapter, not original vehicle/controller reproduction'
        json_write(folder/'summary.json',summary)
        metadata=json.loads((folder/'metadata.json').read_text());metadata['model_info']=info
        metadata['surface']='Flat, zero wind, nominal friction1; model-specific load/aero closures in model_info'
        metadata['force_input_slip_kind']=(
            'Not modeled: tire force/slip/load arrays are zero compatibility placeholders; metrics N/A'
            if not capabilities['tire_forces'] else
            'Instantaneous lateral source-family slips; longitudinal slip/wheel spin not modeled'
            if not capabilities['longitudinal_slip'] else
            'Instantaneous source-family wheel slips; no tire-relaxation state')
        json_write(folder/'metadata.json',metadata)
    else:
        summary['state_dimension']=33;json_write(folder/'summary.json',summary)
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--models',nargs='+',choices=MODEL_NAMES,default=list(MODEL_NAMES))
    parser.add_argument('--scenario',choices=('nominal','demanding','both'),default='both')
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/thesis_comparison')
    parser.add_argument('--track',type=Path,default=ROOT/'data/Norisring.csv')
    parser.add_argument('--laps',type=float,default=1.)
    parser.add_argument('--maximum-time',type=float,default=600.)
    parser.add_argument('--collect-only',action='store_true')
    args=parser.parse_args()
    if args.collect_only:collect(args.output);return
    for scenario in (('nominal','demanding') if args.scenario=='both' else (args.scenario,)):
        cfg=Experiment(name=scenario,laps=args.laps,maximum_time_s=args.maximum_time)
        if scenario=='demanding':cfg=replace(cfg,maximum_speed_mps=30.,lateral_acceleration_budget_mps2=6.)
        for name in args.models:run_model(name,cfg,args.output,args.track)
    collect(args.output)
if __name__=='__main__':main()
