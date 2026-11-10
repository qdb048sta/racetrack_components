"""Same-input forecasts against OCD and own-state step refinement, kept distinct."""
from __future__ import annotations
import argparse
from dataclasses import fields
import hashlib,json,math,time
from pathlib import Path
import numpy as np
from .benchmark import ROOT,torque_allocation,json_write
from .controller import Experiment
from .thesis_models import MODEL_NAMES,make_plant
from .validation import (select_windows,select_refinement_windows,endpoint_errors,
    aggregate_errors,replay_window,_write_csv)


def load_run(output,scenario,model):
    folder=Path(output)/scenario/model;meta=json.loads((folder/'metadata.json').read_text())
    summary=json.loads((folder/'summary.json').read_text())
    with np.load(folder/'raw.npz',allow_pickle=False) as raw:
        data={k:np.asarray(raw[k],dtype=float) for k in ('time_s','states','inputs','station_m','internal_state')}
    if not all(np.isfinite(v).all() for v in data.values()):raise ValueError('Nonfinite saved trace')
    n=len(data['time_s'])
    if data['states'].shape!=(n,7) or data['inputs'].shape!=(n,2) or np.any(np.diff(data['time_s'])<=0):
        raise ValueError('Invalid source arrays')
    cfg=Experiment(**{k:v for k,v in meta['settings'].items() if k in {f.name for f in fields(Experiment)}})
    plant=make_plant(model,cfg)
    if model=='native_ocd':obs=data['internal_state'][:,[0,1,3,6,7,9,32]]
    else:obs=np.array([plant.observe(z) for z in data['internal_state']])
    plant.close()
    np.testing.assert_allclose(obs,data['states'],rtol=0,atol=1e-10)
    return dict(data=data,config=cfg,summary=summary,metadata=meta,
        provenance=dict(scenario=scenario,model=model,raw_sha256=hashlib.sha256((folder/'raw.npz').read_bytes()).hexdigest(),
                        metadata_sha256=hashlib.sha256((folder/'metadata.json').read_bytes()).hexdigest()))


def run_validation(output,scenarios=('nominal','demanding'),forecast_count=12,refinement_count=3):
    output=Path(output);dest=output/'validation';dest.mkdir(parents=True,exist_ok=True)
    from .thesis_report import refresh_metadata
    refresh_metadata(output)
    forecasts=[];refinements=[];fs=[];rs=[];sources=[]
    for scenario in scenarios:
        source=load_run(output,scenario,'native_ocd');cfg=source['config'];sources.append(source['provenance'])
        windows=select_windows(source,forecast_count,1.)
        for model in MODEL_NAMES[1:]:
            plant=make_plant(model,cfg);rows=[]
            for w in windows:
                row=dict(scenario=scenario,model=model,source_model='native_ocd',success=False,failure=None,**w)
                try:
                    initial=source['data']['internal_state'][w['start_index']]
                    plant.reset_from_native(initial);mapped=plant.state.copy();observed=plant.observation
                    ref0=source['data']['states'][w['start_index']]
                    row['initial_vx_mismatch_mps']=float(observed[3]-ref0[3])
                    row['initial_vy_mismatch_mps']=float(observed[4]-ref0[4])
                    row['initial_yaw_rate_mismatch_radps']=float(observed[5]-ref0[5])
                    prediction=replay_window(plant,mapped,source,w,cfg.reduced_step_s)
                    row.update(endpoint_errors(prediction,source['data']['states'][w['end_index']]))
                    row['success']=True
                except (RuntimeError,ValueError,FloatingPointError) as e:row['failure']=str(e)
                forecasts.append(row);rows.append(row)
            plant.close();fs.append(dict(scenario=scenario,model=model,**aggregate_errors(rows,'endpoint')))
            print('Forecast',scenario,model,fs[-1],flush=True)
        for model in MODEL_NAMES:
            source=load_run(output,scenario,model);cfg=source['config'];sources.append(source['provenance'])
            plant=make_plant(model,cfg);rows=[];step=cfg.native_step_s if model=='native_ocd' else cfg.reduced_step_s
            for w in select_refinement_windows(source,refinement_count,1.):
                row=dict(scenario=scenario,model=model,success=False,failure=None,coarse_step_s=step,fine_step_s=step/2,**w)
                try:
                    initial=source['data']['internal_state'][w['start_index']]
                    coarse=replay_window(plant,initial,source,w,step)
                    fine=replay_window(plant,initial,source,w,step/2)
                    row.update(endpoint_errors(coarse,source['data']['states'][w['end_index']],'same_step_replay'))
                    row.update(endpoint_errors(fine,coarse,'half_step'));row['success']=True
                except (RuntimeError,ValueError,FloatingPointError) as e:row['failure']=str(e)
                refinements.append(row);rows.append(row)
            plant.close();r=dict(scenario=scenario,model=model,**aggregate_errors(rows,'half_step'))
            r.update({k:v for k,v in aggregate_errors(rows,'same_step_replay').items() if k.startswith('same_step')})
            r['same_step_consistent']=bool(r['failed_windows']==0
                and r['same_step_replay_position_max_abs_m']<1e-6
                and r['same_step_replay_yaw_max_abs_deg']<1e-5
                and r['same_step_replay_speed_max_abs_mps']<1e-6
                and r['same_step_replay_yaw_rate_max_abs_radps']<1e-6)
            rs.append(r);print('Refinement',scenario,model,r,flush=True)
    _write_csv(dest/'window_predictions.csv',forecasts);_write_csv(dest/'integration_refinement.csv',refinements)
    report=dict(complete=True,forecast_summary=fs,integration_summary=rs,sources=sources,
        design=dict(forecast_horizon_s=1.,forecast_windows_per_model_scenario=forecast_count,
            refinement_windows_per_model_scenario=refinement_count,
            forecast_selection='Nearest timestamps at equally spaced stations 50m to lap_length-50m, independent of error',
            refinement_selection='First/last station windows and one centered at strongest absolute yaw rate',
            interpretation='Mismatch to synthetic OCD reference, not real-vehicle accuracy. Closed-loop common-controller tracking is a different experiment.',
            initial_mapping='Preserve all represented native states once; Beal roll/base conversion; Sun projects speed/delta onto no-slip manifold, with initial velocity/yaw-rate mismatch recorded.',
            omitted_native_states='Heave, pitch, unsprung and tire relaxation absent from reduced plants',
            integration='Each own full snapshot, same inputs at dt and dt/2; same-step replay checked separately'),
        all_windows_successful=all(r['success'] for r in forecasts+refinements),
        all_same_step_consistent=all(r['same_step_consistent'] for r in rs),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    json_write(dest/'summary.json',report)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,default=ROOT/'outputs/thesis_comparison')
    a=p.parse_args();run_validation(a.output)
if __name__=='__main__':main()
