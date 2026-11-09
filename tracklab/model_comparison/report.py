"""Make figures, exact result tables and an offline replay from completed runs."""
from __future__ import annotations
from pathlib import Path
from dataclasses import asdict
import argparse
import hashlib
import json
import shutil
import numpy as np

from tracklab import PeriodicTrack
from .benchmark import ROOT, MODEL_NAMES, LABELS, collect, json_write
from .models import ReducedDoubleTrack,VehicleParameters

COLORS={"native_ocd":"#172b4d","mf_independent":"#d85b27","mf_ellipse":"#2785b2",
        "fiala":"#6a52a3","mf_static":"#388766"}
SHORT={"native_ocd":"Native OCD","mf_independent":"Planar MF","mf_ellipse":"MF ellipse",
       "fiala":"Standard Fiala","mf_static":"MF static loads"}


def plots(source, destination, road):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size":10,"axes.spines.top":False,"axes.spines.right":False,
                         "axes.grid":True,"grid.alpha":.18,"figure.facecolor":"#ffffff"})
    s=np.linspace(0,road.length,1800);q=road.sample_geometry(s)
    for scenario in ("nominal","demanding"):
        fig,axes=plt.subplots(3,2,figsize=(13.5,12),constrained_layout=True)
        ax=axes[0,0]
        ax.plot(*q["boundary_left"].T,color=".75",lw=.8)
        ax.plot(*q["boundary_right"].T,color=".75",lw=.8)
        ax.set_aspect("equal");ax.set_title("Actual driven trajectories on Norisring")
        ax.set_xlabel("X [m]");ax.set_ylabel("Y [m]")
        for name in MODEL_NAMES:
            file=source/scenario/name/"raw.npz"
            if not file.exists():continue
            d=np.load(file);z=d["states"];ss=d["station_m"];col=COLORS[name]
            style="--" if name=="mf_static" else "-"
            args={"color":col,"lw":1.35,"ls":style,"label":SHORT[name]}
            ax.plot(z[:,0],z[:,1],**args)
            axes[0,1].plot(ss,d["cross_track_m"],**args)
            axes[1,0].plot(ss,np.hypot(z[:,3],z[:,4])*3.6,**args)
            axes[1,1].plot(ss,np.rad2deg(np.arctan2(z[:,4],z[:,3])),**args)
            util=np.hypot(d["tire_force_longitudinal_N"],d["tire_force_lateral_N"])/d["tire_force_vertical_N"]
            axes[2,0].plot(ss,util.max(axis=1),**args)
            axes[2,1].plot(ss,d["tire_force_vertical_N"].min(axis=1),**args)
        titles=["Cross-track error","Speed","Body sideslip angle","Maximum wheel force / normal load","Minimum wheel normal load"]
        labels=["Lateral error [m]","Speed [km/h]","Sideslip [deg]","Force ratio [-]","Normal force [N]"]
        for ax,title,label in zip([axes[0,1],axes[1,0],axes[1,1],axes[2,0],axes[2,1]],titles,labels):
            ax.set_title(title);ax.set_xlabel("Road station [m]");ax.set_ylabel(label);ax.set_xlim(0,road.length)
        axes[2,0].axhline(1,color=".3",ls=":",lw=1)
        axes[0,1].legend(ncol=2,fontsize=8,loc="best")
        fig.suptitle(f"Double-track model comparison • {scenario} speed schedule\nSame car parameters, road and tracking MPC; normalized μ = 1",fontsize=15)
        fig.savefig(destination/f"{scenario}_comparison.png",dpi=170)
        plt.close(fig)

    fig,axes=plt.subplots(1,2,figsize=(12,4.8),constrained_layout=True)
    alpha=np.deg2rad(np.linspace(0,16,321))
    loads=np.full(4,2000.)
    for name in ("mf_independent","mf_ellipse","fiala"):
        p=ReducedDoubleTrack(name)
        for kappa,ls in ((0.,"-"),(.02,"--")):
            data=np.array([p.tire_forces(np.full(4,a),np.full(4,kappa),loads)[1][0]/2000 for a in alpha])
            axes[0].plot(np.rad2deg(alpha),data,color=COLORS[name],ls=ls,
                         label=f"{SHORT[name]}, κ={kappa:g}")
        kappas=np.linspace(0,.1,300)
        forces=np.array([[v[0] for v in p.tire_forces(np.full(4,np.deg2rad(6)),np.full(4,k),loads)] for k in kappas])/2000
        axes[1].plot(forces[:,0],forces[:,1],color=COLORS[name],label=SHORT[name])
    theta=np.linspace(0,np.pi/2,201)
    axes[1].plot(np.cos(theta),np.sin(theta),":",color=".3",label="μ = 1 circle")
    axes[0].set(xlabel="Force-input slip angle [deg]",ylabel="Lateral force / normal load",title="Normalized steady tire laws")
    axes[1].set(xlabel="Longitudinal force / normal load",ylabel="Lateral force / normal load",title="Combined-force sweep at α = 6°",aspect="equal")
    for ax in axes:ax.legend(fontsize=8);ax.set_ylim(0,1.1)
    fig.savefig(destination/"tire_law_comparison.png",dpi=180);plt.close(fig)


def replay(source,destination,road,rows):
    geometry=road.sample_geometry(np.linspace(0,road.length,1500))
    payload={"geometry":{k:np.round(geometry[k],4).tolist() for k in ("center","boundary_left","boundary_right")},
             "colors":COLORS,"names":SHORT,"runs":{},"summary":rows}
    for scenario in ("nominal","demanding"):
        payload["runs"][scenario]={}
        for name in MODEL_NAMES:
            p=source/scenario/name/"raw.npz"
            if not p.exists():continue
            d=np.load(p);ids=np.unique(np.r_[np.arange(0,len(d["time_s"]),5),len(d["time_s"])-1])
            payload["runs"][scenario][name]=np.round(np.column_stack([d["time_s"],d["states"],d["station_m"],d["cross_track_m"]])[ids],6).tolist()
    template=Path(__file__).with_name("replay_template.html").read_text()
    content=template.replace("__PAYLOAD__",json.dumps(payload,separators=(",",":"),allow_nan=False).replace("</","<\\/"))
    (destination/"replay.html").write_text(content,encoding="utf-8")


def make_report(source,destination):
    source=Path(source);destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
    rows=collect(source)
    if len(rows)!=10 or any(not r["completed"] for r in rows):
        raise RuntimeError("Publishing requires all ten completed model/scenario runs; inspect failures first")
    validation_path=source/"validation/summary.json"
    validation=json.loads(validation_path.read_text()) if validation_path.exists() else {}
    if not validation.get("complete",False) or not validation.get("all_windows_successful",False):
        raise RuntimeError("Publishing requires completed forecast and numerical refinement checks")
    canonical_hash=hashlib.sha256((ROOT/"data/Norisring.csv").read_bytes()).hexdigest()
    for row in rows:
        metadata=json.loads((source/row["experiment"]/row["model"]/"metadata.json").read_text())
        if metadata["track_sha256"]!=canonical_hash or metadata["settings"]["laps"]!=1.:
            raise RuntimeError("This recorded-study report requires full laps on the unchanged Norisring CSV")
    for item in validation.get("forecast_sources",[])+validation.get("integration_sources",[]):
        raw_path=source/item["raw_file"]
        if hashlib.sha256(raw_path.read_bytes()).hexdigest()!=item["raw_sha256"]:
            raise RuntimeError(f"Validation is stale for {item['raw_file']}; run it again")
    road=PeriodicTrack(ROOT/"data/Norisring.csv")
    from tracklab.comparison_native.vehicle import NativeDoubleTrack,normalized_parameters
    native=NativeDoubleTrack(normalized_parameters())
    native_parameters=native.parameters();native.close()
    source_files=[ROOT/"run_model_comparison.py",ROOT/"tests/test_model_comparison_physics.py"]
    for folder in ("tracklab/model_comparison","tracklab/comparison_native"):
        source_files.extend(p for p in (ROOT/folder).rglob("*") if p.suffix in (".py",".cpp",".html"))
    build_path=ROOT/"tracklab/comparison_native/native/build_info.json"
    build=json.loads(build_path.read_text()) if build_path.exists() else {}
    json_write(destination/"experiment_provenance.json",{
        "upstream_ocd_commit":"94f8fb187fb0ed22bba1d809bd74f66d1ff75af4",
        "repository_base_commit":"e67066071572d9dbda95604239cc7a43d9615208",
        "native_normalization_overrides":normalized_parameters(),
        "native_parameters":native_parameters,"reduced_parameters":asdict(VehicleParameters()),
        "native_build":{k:v for k,v in build.items() if k not in ("source_directory","compile_command")},
        "source_sha256":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(source_files))},
        "track_sha256":hashlib.sha256((ROOT/"data/Norisring.csv").read_bytes()).hexdigest(),
        "tire_slip_semantics":{"native_ocd":"relaxed force-input states","reduced":"instantaneous kinematic force inputs"},
    })
    for name in ("summary.json","summary.csv"):
        shutil.copy2(source/name,destination/name)
    artifact_hashes={}
    for scenario in ("nominal","demanding"):
        for model in MODEL_NAMES:
            run=source/scenario/model
            for filename in ("trajectory.csv","metadata.json","summary.json"):
                if (run/filename).exists():
                    target=destination/f"{scenario}_{model}_{filename}"
                    shutil.copy2(run/filename,target)
                    artifact_hashes[target.name]=hashlib.sha256(target.read_bytes()).hexdigest()
            artifact_hashes[f"generated:{scenario}/{model}/raw.npz"]=hashlib.sha256((run/"raw.npz").read_bytes()).hexdigest()
    if (source/"validation").is_dir():
        shutil.copytree(source/"validation",destination/"validation",dirs_exist_ok=True)
    plots(source,destination,road);replay(source,destination,road,rows)
    json_write(destination/"artifact_hashes.json",artifact_hashes)
    text=["# Recorded Norisring comparison", "", "See [method and equations](../README.md), [literature](../LITERATURE.md), and [source/parameter provenance](experiment_provenance.json).", "",
          "All values below are measured from the executed normalized plants. Lap times belong to a prescribed speed schedule and common tracking controller; they are not minimum lap times or an accuracy ranking.",""]
    for scenario in ("nominal","demanding"):
        text += [f"## {scenario.capitalize()} schedule", "", "| Plant | Lap (s) | RMS offset (m) | Max offset (m) | Max body sideslip (°) | Peak force ratio | Min sampled body margin (m) | MPC failures |",
                 "|---|---:|---:|---:|---:|---:|---:|---:|"]
        for model in MODEL_NAMES:
            r=next(r for r in rows if r["experiment"]==scenario and r["model"]==model)
            text.append(f"| {SHORT[model]} | {r['lap_time_s']:.3f} | {r['rms_cross_track_m']:.4f} | {r['max_abs_cross_track_m']:.4f} | {r['max_abs_sideslip_deg']:.3f} | {r['max_friction_circle_utilization']:.4f} | {r['minimum_sampled_body_margin_m']:.3f} | {r['mpc_failures']} |")
        text += ["",f"![{scenario} comparison]({scenario}_comparison.png)",""]
    text += ["## One-second model predictions under identical inputs", "",
        "Twelve windows per scenario start from the native body's state, actual wheel speeds and steering, and replay the native commands without reoptimization.","",
        "| Reduced plant | Nominal position RMSE (m) | Demanding position RMSE (m) | Demanding maximum error (m) |",
        "|---|---:|---:|---:|"]
    for model in MODEL_NAMES[1:]:
        nominal=next(r for r in validation["forecast_summary"] if r["model"]==model and r["scenario"]=="nominal")
        demanding=next(r for r in validation["forecast_summary"] if r["model"]==model and r["scenario"]=="demanding")
        text.append(f"| {SHORT[model]} | {nominal['endpoint_position_rmse_m']:.6f} | {demanding['endpoint_position_rmse_m']:.6f} | {demanding['endpoint_position_max_abs_m']:.6f} |")
    forecast_count=sum(r["successful_windows"] for r in validation["forecast_summary"])
    integration_count=sum(r["successful_windows"] for r in validation["integration_summary"])
    replay_error=max(r["same_step_replay_position_max_abs_m"] for r in validation["integration_summary"])
    native_error=max(r["half_step_position_max_abs_m"] for r in validation["integration_summary"] if r["model"]=="native_ocd")
    reduced_error=max(r["half_step_position_max_abs_m"] for r in validation["integration_summary"] if r["model"]!="native_ocd")
    text += ["", f"All {forecast_count} forecast windows and {integration_count} numerical-refinement windows succeeded. Original-step snapshot replays reproduce endpoints within {replay_error:.3e} m. Halving integration steps changes the selected native one-second endpoints by at most {native_error:.3e} m and reduced endpoints by at most {reduced_error:.3e} m. The refinement set includes tight-turn windows centered on each run's strongest yaw rate; these are local checks rather than global error bounds.","",
        "## Interpretation", "",
        "- The circle ratio is sqrt(Fx²+Fy²)/Fz because the experiment normalizes μ to one. Independent MF is not constrained by this circle.",
        "- Body sideslip is atan2(vy,vx). Tight low-speed turns can have appreciable geometric body sideslip while tire slip remains modest.",
        "- Native force-input slips are relaxed dynamic states; reduced force-input slips are instantaneous kinematic values. The raw slip fields must be read with that distinction.",
        "- Containment uses 40 body-perimeter samples every 0.02 s. The separate CSV traces are downsampled to 0.1 s; they are not the complete boundary-check sample set.",
        "- Native C++ and Python integration times depend on implementation and host contention. They do not establish a model-complexity or physical-fidelity ordering.", "",
        "## Held-input and integration checks", "", "See [validation summary](validation/summary.json) and [one-second prediction errors](validation/window_predictions.csv). These replays do not solve MPC again. Numerical step refinement uses each model's own state and controls; model discrepancy uses common native initial conditions and inputs.","",
        "## Interactive replay", "", "Download [replay.html](replay.html) and open it locally. It embeds the recorded trajectories and needs no server or network. Controls switch scenario/plant, play speed, and a follow-camera view.","",
        "## Regeneration", "", "The checked-in CSVs contain 10 Hz observations and tire data; metrics were computed from generated 50 Hz raw.npz files. Re-running the documented commands regenerates those full state snapshots and detailed solve logs. Raw arrays and compiled libraries are build outputs rather than repository contents. artifact_hashes.json records the executed raw-array hashes as well as the published CSV hashes.",""]
    (destination/"RESULTS.md").write_text("\n".join(text),encoding="utf-8")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input",type=Path,default=ROOT/"outputs/model_comparison")
    parser.add_argument("--publish",type=Path,default=ROOT/"documentation/model_comparison/recorded")
    args=parser.parse_args();make_report(args.input,args.publish)


if __name__=="__main__":main()
