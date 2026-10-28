"""Numerical result exports. Plotting is in a separate optional module."""
from pathlib import Path
import csv
import json
import numpy as np
from .replay import build_replay
from .validation import diagnose,check_recorded_input_replay


def write_csv(path,headers,rows):
    with Path(path).open('w',newline='',encoding='utf8') as f:
        w=csv.writer(f);w.writerow(headers);w.writerows(rows)


def save_results(road,result,output,independent_check=True):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    raw={k:result[k] for k in ('time_s','states','inputs','progress_m')}
    np.savez_compressed(output/'run_raw.npz',**raw)
    metrics,diag=diagnose(road,raw,result['plant'],result['solves'])
    metrics.update({k:result[k] for k in ('lap_crossing_time_s','simulation_wall_seconds')})
    np.savez_compressed(output/'diagnostics.npz',**diag)
    for name in ('solves','predictions','vehicle_config','driving_config'):
        (output/(name+'.json')).write_text(json.dumps(result[name],indent=2),encoding='utf8')
    (output/'metrics.json').write_text(json.dumps(metrics,indent=2),encoding='utf8')
    headers=['time_s','x_m','y_m','yaw_rad','vx_mps','vy_mps','yaw_rate_radps','steering_rad',
             'steering_target_rad','force_acceleration_mps2',*diag.keys()]
    values=np.column_stack([raw['time_s'],raw['states'],raw['inputs'],*[diag[k] for k in diag]])
    write_csv(output/'trajectory.csv',headers,values)
    keys=list(result['solves'][0]);write_csv(output/'solver_log.csv',keys,
                                            ([r[k] for k in keys] for r in result['solves']))
    plan=result['speed_plan']
    write_csv(output/'speed_schedule.csv',['progress_m','curvature_envelope_1pm','speed_limit_mps','planned_speed_mps'],
              np.column_stack([plan.s,plan.curvature_envelope,plan.raw_limit,plan.speed]))
    if independent_check:
        ck=check_recorded_input_replay(road,raw,result['plant'],result['driving_config']['control_period_s'])
        (output/'independent_integration_check.json').write_text(json.dumps(ck,indent=2),encoding='utf8')
    build_replay(road,raw,diag,metrics,result['predictions'],result['vehicle_config'],
                 result['driving_config'],output/'norisring_replay.html')
    return metrics
