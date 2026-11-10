"""Publish verified six-plant results, charts and an offline trajectory replay."""
from __future__ import annotations
from pathlib import Path
import argparse,hashlib,json,shutil,platform,subprocess
from datetime import datetime, timezone
import numpy as np
from tracklab import PeriodicTrack
from .benchmark import ROOT,collect,json_write
from .thesis_models import MODEL_NAMES,LABELS,make_plant
from .controller import Experiment

SHORT={'native_ocd':'Native OCD','liniger_bicycle':'Liniger bicycle','gao_fourwheel':'Gao four-wheel',
       'beal_brush':'Beal yaw-roll','sun_kinematic':'Sun kinematic','lienkamp_linear':'Lienkamp linear'}
COLORS=dict(zip(MODEL_NAMES,['#143C51','#C45C34','#267F9D','#755596','#3F8B70','#B38323']))


def plot_comparison(source,dest,road):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,
        'axes.grid':True,'grid.alpha':.2,'figure.facecolor':'white'})
    geometry=road.sample_geometry(np.linspace(0,road.length,1800))
    for scenario in ('nominal','demanding'):
        fig,axes=plt.subplots(3,2,figsize=(13,11),constrained_layout=True)
        axes[0,0].plot(*geometry['boundary_left'].T,color='.75',lw=.8)
        axes[0,0].plot(*geometry['boundary_right'].T,color='.75',lw=.8)
        axes[0,0].set_aspect('equal');axes[0,0].set(title='Recorded vehicle paths',xlabel='X [m]',ylabel='Y [m]')
        for name in MODEL_NAMES:
            with np.load(source/scenario/name/'raw.npz') as data:
                z=data['states'];s=data['station_m'];args=dict(color=COLORS[name],label=SHORT[name],lw=1.25,ls='--' if name=='lienkamp_linear' else '-')
                axes[0,0].plot(z[:,0],z[:,1],**args)
                axes[0,1].plot(s,data['cross_track_m'],**args)
                axes[1,0].plot(s,np.hypot(z[:,3],z[:,4])*3.6,**args)
                axes[1,1].plot(s,np.rad2deg(np.arctan2(z[:,4],z[:,3])),**args)
                if name not in ('sun_kinematic','lienkamp_linear','liniger_bicycle'):
                    fz=data['tire_force_vertical_N'];ratio=np.hypot(data['tire_force_longitudinal_N'],data['tire_force_lateral_N'])/fz
                    axes[2,0].plot(s,ratio.max(axis=1),**args)
                    axes[2,1].plot(s,fz.min(axis=1),**args)
        names=['Cross-track offset','Speed','Body sideslip','Contact force ratio: native / Gao / Beal','Minimum normal load: native / Gao / Beal']
        units=['Offset [m]','Speed [km/h]','Sideslip [deg]','Ratio [-]','Force [N]']
        for ax,title,unit in zip([axes[0,1],axes[1,0],axes[1,1],axes[2,0],axes[2,1]],names,units):
            ax.set(title=title,xlabel='Road station [m]',ylabel=unit,xlim=(0,road.length))
        axes[0,1].legend(ncol=2,fontsize=8);axes[2,0].axhline(1,color='.3',ls=':',lw=1)
        fig.suptitle(f'{scenario.capitalize()} Norisring schedule | six normalized model families\nCommon tracking MPC and 800 kg vehicle; source-specific force/load structures retained',fontsize=14)
        fig.savefig(dest/f'{scenario}_comparison.png',dpi=160);plt.close(fig)


def replay(source,dest,road,rows):
    geometry=road.sample_geometry(np.linspace(0,road.length,1500))
    payload=dict(geometry={k:np.round(geometry[k],4).tolist() for k in ('center','boundary_left','boundary_right')},
        colors=COLORS,names=SHORT,runs={},summary=rows)
    for scenario in ('nominal','demanding'):
        payload['runs'][scenario]={}
        for name in MODEL_NAMES:
            with np.load(source/scenario/name/'raw.npz') as d:
                ids=np.unique(np.r_[np.arange(0,len(d['time_s']),5),len(d['time_s'])-1])
                payload['runs'][scenario][name]=np.round(np.column_stack([d['time_s'],d['states'],d['station_m'],d['cross_track_m']])[ids],6).tolist()
    template=Path(__file__).with_name('replay_template.html').read_text()
    template=template.replace('One racetrack. Five double-track plants.','One racetrack. Six vehicle models.')
    template=template.replace('Norisring · Double-track model comparison','Norisring · Thesis model-family comparison')
    template=template.replace('Native MF_SIMPLE and independent MF do not enforce a friction circle.','Native MF_SIMPLE, Liniger and the Gao pure-slip adapter do not enforce a friction circle. N/A denotes a force model absent from the comparison.')
    template=template.replace('Standard Fiala is a normalized adaptation, not Bertipaglia’s complete extended Fiala model.','Gao includes only its published pure-slip tire equations; Beal uses declared common parameters and a quasi-static normal-load closure; Sun has no tire-force law.')
    template=template.replace('r.max_friction_circle_utilization.toFixed(4)','r.max_friction_circle_utilization?.toFixed(4)??"N/A"')
    template=template.replace('README.md, LITERATURE.md and RESULTS.md','README.md and RESULTS.md')
    html=template.replace('__PAYLOAD__',json.dumps(payload,separators=(',',':'),allow_nan=False).replace('</','<\\/'))
    (dest/'replay.html').write_text(html)


def refresh_metadata(source):
    source=Path(source)
    # Refresh descriptive metadata from the audited implementation, never trajectory data.
    for scenario in ('nominal','demanding'):
        for name in MODEL_NAMES:
            folder=source/scenario/name;p=folder/'metadata.json';meta=json.loads(p.read_text())
            if name!='native_ocd':
                plant=make_plant(name,Experiment());info=plant.model_info;plant.close();meta['model_info']=info
                caps=info['capabilities']
                summary_path=folder/'summary.json';summary=json.loads(summary_path.read_text())
                if not caps['force_circle_metric']:
                    if name=='liniger_bicycle' and summary.get('max_friction_circle_utilization') is not None:
                        summary['max_effective_axle_force_ratio']=summary['max_friction_circle_utilization']
                    summary['max_friction_circle_utilization']=None;summary['max_force_circle_excess_N']=None
                json_write(summary_path,summary)
                meta['force_input_slip_kind']=('N/A: no tire contact/slip/load model' if not caps['tire_forces'] else
                   'Instantaneous lateral slip; longitudinal slip/wheel spin N/A' if not caps['longitudinal_slip'] else 'Instantaneous wheel slip, no relaxation states')
            else:
                psummary=folder/'summary.json';r=json.loads(psummary.read_text());r['state_dimension']=33;json_write(psummary,r)
            json_write(p,meta)
    return collect(source)


def make_report(source,dest):
    source=Path(source);dest=Path(dest);dest.mkdir(parents=True,exist_ok=True)
    rows=refresh_metadata(source)
    if len(rows)!=12 or any(not r['completed'] for r in rows):raise RuntimeError('Require 12 completed laps; inspect failures before publication')
    val=json.loads((source/'validation/summary.json').read_text())
    if not val.get('complete') or not val.get('all_windows_successful') or not val.get('all_same_step_consistent'):
        raise RuntimeError('Need completed successful forecast and snapshot-refinement validation')
    for item in val['sources']:
        raw=source/item['scenario']/item['model']/'raw.npz'
        if hashlib.sha256(raw.read_bytes()).hexdigest()!=item['raw_sha256']:raise RuntimeError('Stale validation raw data')
        meta=raw.with_name('metadata.json')
        if hashlib.sha256(meta.read_bytes()).hexdigest()!=item['metadata_sha256']:raise RuntimeError('Stale validation metadata')
    road=PeriodicTrack(ROOT/'data/Norisring.csv')
    from tracklab.comparison_native.vehicle import NativeDoubleTrack,normalized_parameters
    native=NativeDoubleTrack(normalized_parameters());params=native.parameters();native.close()
    build=json.loads((ROOT/'tracklab/comparison_native/native/build_info.json').read_text())
    files=[ROOT/'run_thesis_comparison.py',ROOT/'tests/test_thesis_models.py',ROOT/'tracklab/comparison_native/native/upstream_double_track.json']
    files += [p for directory in ('tracklab/model_comparison','tracklab/comparison_native') for p in (ROOT/directory).rglob('*') if p.suffix in ('.py','.cpp','.html')]
    provenance=dict(upstream_ocd_commit=build['commit'],repository_base_commit='e45c711',
        report_generated_utc=datetime.now(timezone.utc).isoformat(),python=platform.python_version(),numpy=np.__version__,
        host=platform.platform(),native_normalization_overrides=normalized_parameters(),native_parameters=params,
        native_build={k:v for k,v in build.items() if k not in ('source_directory','compile_command')},
        source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(files))},
        track_sha256=hashlib.sha256((ROOT/'data/Norisring.csv').read_bytes()).hexdigest(),
        timing_interpretation='Sequential laps; native C++ versus Python adapters; host-specific elapsed wall times, not abstract model complexity or thesis MPC runtime',
        disclosure='Freshly executed source-family adapters; earlier Library PDF was used to recover adapter specifications, not numeric result data')
    json_write(dest/'experiment_provenance.json',provenance)
    for file in ('summary.json','summary.csv'):shutil.copy2(source/file,dest/file)
    hashes={}
    for scenario in ('nominal','demanding'):
        for name in MODEL_NAMES:
            for file in ('trajectory.csv','metadata.json','summary.json'):
                target=dest/f'{scenario}_{name}_{file}';shutil.copy2(source/scenario/name/file,target)
                hashes[target.name]=hashlib.sha256(target.read_bytes()).hexdigest()
            hashes[f'generated:{scenario}/{name}/raw.npz']=hashlib.sha256((source/scenario/name/'raw.npz').read_bytes()).hexdigest()
    shutil.copytree(source/'validation',dest/'validation',dirs_exist_ok=True)
    json_write(dest/'artifact_hashes.json',hashes)
    plot_comparison(source,dest,road);replay(source,dest,road,rows)
    text=['# Recorded six-model Norisring comparison','',
        'All entries below come from fresh full laps. These are normalized model-family adapters under one fixed tracking controller and prescribed speed schedule. They are not calibrated reproductions of the theses or an optimal-lap/real-car accuracy ranking. Liu is a structural tire subsystem and has no invented whole-car lap.','',
        'The unchanged Norisring CSV has length 2296.312367 m. Initial forward speed is 8 m/s. Native OCD has 33 states; Liniger 7; Gao 11; Beal 13; Sun 5; Lienkamp 7. Steering actuator, torque mapping and other additions are explicit in each metadata file and in the main equation reference.','']
    for scenario in ('nominal','demanding'):
        text += [f'## {scenario.capitalize()} schedule','',
            '| Model | Lap (s) | RMS offset (m) | Peak offset (m) | Peak force ratio | Min sampled body margin (m) | MPC failures |',
            '|---|---:|---:|---:|---:|---:|---:|']
        for model in MODEL_NAMES:
            r=next(r for r in rows if r['experiment']==scenario and r['model']==model)
            ratio='N/A' if r['max_friction_circle_utilization'] is None else f"{r['max_friction_circle_utilization']:.4f}"
            text.append(f"| {SHORT[model]} | {r['lap_time_s']:.3f} | {r['rms_cross_track_m']:.4f} | {r['max_abs_cross_track_m']:.4f} | {ratio} | {r['minimum_sampled_body_margin_m']:.3f} | {r['mpc_failures']} |")
        text += ['',f'![{scenario.capitalize()} comparison]({scenario}_comparison.png)','']
    text += ['## One-second forecasts under identical native inputs','',
        'Twelve predetermined station windows per scenario replay the native held commands after one initialization. All represented states are copied. Beal converts CG velocities to its source roll-base velocities. Sun cannot represent independent lateral speed/yaw rate, so its initial projection mismatch is recorded explicitly. These errors measure mismatch to the simulated native reference.','',
        '| Model | Nominal endpoint position RMSE (m) | Demanding endpoint position RMSE (m) | Demanding worst endpoint (m) |',
        '|---|---:|---:|---:|']
    for model in MODEL_NAMES[1:]:
        a=next(r for r in val['forecast_summary'] if r['scenario']=='nominal' and r['model']==model)
        b=next(r for r in val['forecast_summary'] if r['scenario']=='demanding' and r['model']==model)
        text.append(f"| {SHORT[model]} | {a['endpoint_position_rmse_m']:.6f} | {b['endpoint_position_rmse_m']:.6f} | {b['endpoint_position_max_abs_m']:.6f} |")
    replay_max=max(r['same_step_replay_position_max_abs_m'] for r in val['integration_summary'])
    native_max=max(r['half_step_position_max_abs_m'] for r in val['integration_summary'] if r['model']=='native_ocd')
    reduced_max=max(r['half_step_position_max_abs_m'] for r in val['integration_summary'] if r['model']!='native_ocd')
    text += ['',f'All 120 forecast windows and 36 numerical-refinement windows succeeded. Same-step full-state replays differed by at most {replay_max:.3e} m. Halving the integration step changed one-second endpoints by at most {native_max:.3e} m for native OCD and {reduced_max:.3e} m for the normalized adapters. Refinement includes each model\'s strongest recorded yaw-rate window. These are local numerical checks, not global error bounds.','',
        '## Measured implementation cost','',
        'Elapsed plant-step time includes its numerical integration. The controller is identical in every experiment. C++ native versus Python adapter costs must not be read as the computational complexity or MPC solver performance of the original theses.','',
        '| Model | Nominal plant ms / simulated s | Demanding plant ms / simulated s | Demanding median common MPC solve (ms) |',
        '|---|---:|---:|---:|']
    for model in MODEL_NAMES:
        a=next(r for r in rows if r['experiment']=='nominal' and r['model']==model)
        b=next(r for r in rows if r['experiment']=='demanding' and r['model']==model)
        text.append(f"| {SHORT[model]} | {a['plant_wall_ms_per_sim_second']:.2f} | {b['plant_wall_ms_per_sim_second']:.2f} | {b['median_mpc_solve_ms']:.2f} |")
    text += ['', '## Interpretation and reproducibility','',
        '- Independent MF and Liniger do not mathematically enforce a combined-slip friction circle. Gao here implements only its published pure-slip expressions; its original combined-slip modification is not claimed reproduced.',
        '- Beal\'s radial brush peak is matched to nominal friction 1 while preserving sliding/static ratio 11/12. Dynamic roll is retained in body equations, but normal loads use a declared quasi-static closure independent of roll states.',
        '- Sun matches the common predictor\'s kinematic structure closely, so small closed-loop tracking error is expected. It supplies no tire forces, saturation, wheel loads or tire slips. Those metrics are N/A.',
        '- Lienkamp is a scheduled extension of a constant-speed linear lateral model. Its linear force law is unsaturated, and its logged longitudinal force is an allocated body-force surrogate, so force-circle utilization is N/A.',
        '- The contact-force ratio is sqrt(Fx^2+Fy^2)/Fz with common nominal friction 1. Liniger wheel slots contain half axle forces; its rear longitudinal force already subtracts body drag/rolling resistance. Its effective net-axle ratio is retained separately as max_effective_axle_force_ratio and is not reported as a physical contact-force metric.',
        '- Road containment is sampled at 40 perimeter points for a 4.4 m by 1.9 m body every 0.02 s. It is not a continuous-time or swept-volume certificate.',
        '- All adapters use maximum RK4 substep 0.001 s, native upstream DoPri uses 0.0005 s. Adapter post-step wheel-speed and steering projection are extra finite-step safeguards; native only has its component derivative clamps.',
        '- Every CSV is downsampled to 10 Hz from 50 Hz raw arrays. The raw hashes are retained; raw arrays regenerate through the commands below.','',
        '### Commands','', '```bash',
        'python -m pip install -r requirements-driving.txt',
        'python -m tracklab.comparison_native.build_native',
        'python -m unittest discover -s tests -p test_thesis_models.py -v',
        'python -m unittest tracklab.comparison_native.test_native -v',
        'python run_thesis_comparison.py --scenario both',
        'python -m tracklab.model_comparison.thesis_validation',
        'python -m tracklab.model_comparison.thesis_report', '```','',
        'Download [replay.html](replay.html) and open it locally for the self-contained interactive replay. The model trajectories, scenario selection, timeline and follow view use the recorded data. See [provenance](experiment_provenance.json), [validation](validation/summary.json) and [exact summary CSV](summary.csv).','']
    (dest/'RESULTS.md').write_text('\n'.join(text))
    return rows,val


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input',type=Path,default=ROOT/'outputs/thesis_comparison')
    p.add_argument('--publish',type=Path,default=ROOT/'documentation/model_comparison/recorded_theses');a=p.parse_args();make_report(a.input,a.publish)
if __name__=='__main__':main()
