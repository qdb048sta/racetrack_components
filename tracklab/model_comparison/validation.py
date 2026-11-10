"""Same-input forecast and integration checks for saved Norisring experiments.

No controller is run here. Every recorded input is held over precisely its
original sample interval. Each window is initialized once from a saved full
state; internal states then evolve continuously until the window ends.

Forecasts compare reduced models against the recorded native OCD trajectory.
Integration checks instead compare each model against itself at half its
original step, using that model's own recorded states and commands. These are
different experiments and are deliberately reported in separate tables.

Usage:
  python -m tracklab.model_comparison.validation --output outputs/model_comparison
  python -m tracklab.model_comparison.validation --output outputs/model_comparison --mode forecast
  python -m tracklab.model_comparison.validation --output outputs/model_comparison --mode refinement
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import fields
import hashlib
import json
import math
from pathlib import Path
import platform
import time

import numpy as np

from .benchmark import MODEL_NAMES, ROOT, make_plant, torque_allocation
from .controller import Experiment

REDUCED_MODELS = tuple(name for name in MODEL_NAMES if name != "native_ocd")
NATIVE_TO_REDUCED = np.array([0, 1, 3, 6, 7, 9, 28, 29, 30, 31, 32])
NATIVE_TO_OBSERVATION = np.array([0, 1, 3, 6, 7, 9, 32])
ERROR_FIELDS = ("position_m", "yaw_deg", "speed_mps", "yaw_rate_radps")


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def _write_csv(path, rows):
    if not rows:
        return
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    names = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=names)
        writer.writeheader()
        writer.writerows(rows)


def load_run(output, scenario, model):
    """Require a complete, finite source trace and retain its byte provenance."""
    folder = Path(output) / scenario / model
    raw_path, metadata_path, summary_path = (folder / name for name in
                                            ("raw.npz", "metadata.json", "summary.json"))
    for path in (raw_path, metadata_path, summary_path):
        if not path.is_file():
            raise FileNotFoundError(f"Required completed-run file is missing: {path}")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    with np.load(raw_path, allow_pickle=False) as raw:
        data = {name: np.asarray(raw[name], dtype=float) for name in
                ("time_s", "states", "inputs", "station_m", "internal_state")}
    count = len(data["time_s"])
    dimension = 33 if model == "native_ocd" else 11
    shapes = {"time_s": (count,), "states": (count, 7), "inputs": (count, 2),
              "station_m": (count,), "internal_state": (count, dimension)}
    if count < 3:
        raise ValueError(f"Too few samples for validation: {folder}")
    for name, shape in shapes.items():
        if data[name].shape != shape or not np.isfinite(data[name]).all():
            raise ValueError(f"Invalid {name} in {raw_path}: expected finite {shape}")
    if np.any(np.diff(data["time_s"]) <= 0):
        raise ValueError(f"Timestamps must strictly increase: {raw_path}")
    observation = (data["internal_state"][:, NATIVE_TO_OBSERVATION] if model == "native_ocd"
                   else data["internal_state"][:, [0, 1, 2, 3, 4, 5, 10]])
    if not np.allclose(observation, data["states"], rtol=0, atol=1e-9):
        raise ValueError(f"Full states and saved observations disagree: {raw_path}")
    cfg_fields = {f.name for f in fields(Experiment)}
    settings = metadata["settings"]
    cfg = Experiment(**{key: value for key, value in settings.items() if key in cfg_fields})
    return {
        "data": data, "config": cfg, "summary": summary, "metadata": metadata,
        "provenance": {
            "scenario": scenario, "model": model,
            "raw_file": str(raw_path.relative_to(output)), "raw_sha256": _sha256(raw_path),
            "metadata_sha256": _sha256(metadata_path),
            "source_lap_completed": bool(summary["completed"]),
            "source_end_time_s": float(data["time_s"][-1]),
            "source_end_station_m": float(data["station_m"][-1]),
            "recorded_integrator_step_s": cfg.native_step_s if model == "native_ocd" else cfg.reduced_step_s,
        },
    }


def select_windows(run, count=12, horizon_s=1.0, station_margin_m=50.0):
    """Choose nearest recorded starts to equally spaced physical stations.

    Selection depends only on the source track, station samples and requested
    duration, never on prediction errors. Endpoints use the nearest timestamp
    to the requested horizon; actual durations are retained in every row.
    """
    data = run["data"]
    times, stations = data["time_s"], data["station_m"]
    if not isinstance(count, int) or isinstance(count, bool) or count < 1:
        raise ValueError("Window count must be a positive integer")
    if not np.isfinite(horizon_s) or horizon_s <= 0 or station_margin_m < 0:
        raise ValueError("Use a positive horizon and nonnegative station margin")
    candidates = np.flatnonzero(times + horizon_s <= times[-1] + 1e-10)
    if not len(candidates):
        raise ValueError("Source run is shorter than the forecast horizon")
    length = float(run["summary"]["reference_length_m"])
    low = max(float(stations[candidates].min()), station_margin_m)
    high = min(float(stations[candidates].max()), length-station_margin_m)
    if high <= low:
        raise ValueError("Source trace does not cover the requested station margins")
    targets = np.linspace(low, high, count)
    starts = [int(candidates[np.argmin(np.abs(stations[candidates]-target))]) for target in targets]
    if len(set(starts)) != count:
        raise ValueError("Insufficient distinct source samples for requested windows")
    windows = []
    for ordinal, (target, start) in enumerate(zip(targets, starts), 1):
        end = int(np.argmin(np.abs(times-(times[start]+horizon_s))))
        if end <= start:
            raise ValueError("Forecast window has no time interval")
        windows.append({"window": ordinal, "target_station_m": float(target),
                        "start_index": start, "end_index": end,
                        "start_station_m": float(stations[start]),
                        "end_station_m": float(stations[end]),
                        "start_time_s": float(times[start]), "end_time_s": float(times[end]),
                        "duration_s": float(times[end]-times[start]),
                        "input_intervals": end-start})
    return windows


def select_refinement_windows(run, count=3, horizon_s=1.0):
    """Include the strongest recorded turn, not only equally spaced straights.

    Keep the end station windows, replacing the middle selected window with
    one centered on the greatest absolute yaw rate. Selection is based solely
    on the source motion, never on the resulting integration differences.
    """
    windows = select_windows(run, count, horizon_s)
    if count < 3:
        return windows
    data = run["data"]
    times, stations = data["time_s"], data["station_m"]
    candidates = np.flatnonzero((times >= windows[0]["start_time_s"]+horizon_s/2)
                               & (times <= windows[-1]["start_time_s"]+horizon_s/2))
    peak = int(candidates[np.argmax(np.abs(data["states"][candidates, 5]))])
    start = int(np.argmin(np.abs(times-(times[peak]-horizon_s/2))))
    end = int(np.argmin(np.abs(times-(times[start]+horizon_s))))
    middle = count//2
    windows[middle] = {
        "window": middle+1, "target_station_m": float(stations[start]),
        "start_index": start, "end_index": end,
        "start_station_m": float(stations[start]), "end_station_m": float(stations[end]),
        "start_time_s": float(times[start]), "end_time_s": float(times[end]),
        "duration_s": float(times[end]-times[start]), "input_intervals": end-start,
        "selection": "window centered on greatest absolute recorded yaw rate",
        "selection_peak_yaw_rate_radps": float(data["states"][peak, 5]),
    }
    for i, window in enumerate(windows):
        if i != middle:
            window["selection"] = "fixed station window"
    return windows


def replay_window(plant, full_state, source, window, integration_step_s):
    """Reset once, then replay the source inputs without feedback/replanning."""
    if not np.isfinite(integration_step_s) or integration_step_s <= 0:
        raise ValueError("Integration step must be positive")
    plant.reset(full_state)
    data = source["data"]
    times, inputs = data["time_s"], data["inputs"]
    start, end = window["start_index"], window["end_index"]
    result = np.asarray(plant.observation, dtype=float)
    for index in range(start, end):
        duration = float(times[index+1]-times[index])
        command = inputs[index]
        torque = torque_allocation(command[1])
        substeps = max(1, math.ceil(duration/integration_step_s-1e-10))
        result = np.asarray(plant.step(torque, float(command[0]), duration,
                                      substeps=substeps), dtype=float)
        if result.shape != (7,) or not np.isfinite(result).all():
            raise FloatingPointError("Replay returned an invalid common observation")
    return result


def endpoint_errors(predicted, reference, prefix="endpoint"):
    """Euclidean position error and signed errors for angular/scalar outputs."""
    delta_xy = np.asarray(predicted[:2])-np.asarray(reference[:2])
    yaw = math.atan2(math.sin(predicted[2]-reference[2]), math.cos(predicted[2]-reference[2]))
    return {
        prefix+"_position_m": float(np.linalg.norm(delta_xy)),
        prefix+"_x_m": float(delta_xy[0]), prefix+"_y_m": float(delta_xy[1]),
        prefix+"_yaw_deg": math.degrees(yaw),
        prefix+"_speed_mps": float(np.hypot(*predicted[3:5])-np.hypot(*reference[3:5])),
        prefix+"_yaw_rate_radps": float(predicted[5]-reference[5]),
        prefix+"_steering_rad": float(predicted[6]-reference[6]),
    }


def aggregate_errors(rows, prefix):
    successful = [row for row in rows if row["success"]]
    result = {"window_count": len(rows), "successful_windows": len(successful),
              "failed_windows": len(rows)-len(successful)}
    for field in ERROR_FIELDS:
        name, unit = field.rsplit("_", 1)
        values = np.array([row[prefix+"_"+field] for row in successful], dtype=float)
        result[f"{prefix}_{name}_rmse_{unit}"] = (float(np.sqrt(np.mean(values*values)))
                                                 if len(values) else None)
        result[f"{prefix}_{name}_max_abs_{unit}"] = (float(np.max(np.abs(values)))
                                                    if len(values) else None)
    return result


def forecast_checks(output, scenarios, window_count=12, horizon_s=1., progress=True):
    rows, summaries, provenance = [], [], []
    for scenario in scenarios:
        source = load_run(output, scenario, "native_ocd")
        provenance.append(source["provenance"])
        windows = select_windows(source, window_count, horizon_s)
        cfg = source["config"]
        for model in REDUCED_MODELS:
            plant = make_plant(model, cfg)
            model_rows = []
            try:
                for window in windows:
                    row = {"scenario": scenario, "model": model, **window,
                           "source_model": "native_ocd", "success": False, "failure": None,
                           "integration_step_s": cfg.reduced_step_s}
                    initial = source["data"]["internal_state"][window["start_index"]][NATIVE_TO_REDUCED]
                    reference = source["data"]["states"][window["end_index"]]
                    try:
                        result = replay_window(plant, initial, source, window, cfg.reduced_step_s)
                        row.update(endpoint_errors(result, reference))
                        row["success"] = True
                    except (RuntimeError, ValueError, FloatingPointError) as error:
                        row["failure"] = str(error)
                    rows.append(row); model_rows.append(row)
            finally:
                plant.close()
            summary = {"scenario": scenario, "model": model,
                       **aggregate_errors(model_rows, "endpoint")}
            summaries.append(summary)
            if progress:
                print(f"Forecast {scenario}/{model}: {summary}", flush=True)
    return rows, summaries, provenance


def refinement_checks(output, scenarios, window_count=3, horizon_s=1., progress=True):
    rows, summaries, provenance = [], [], []
    for scenario in scenarios:
        for model in MODEL_NAMES:
            source = load_run(output, scenario, model)
            provenance.append(source["provenance"])
            windows = select_refinement_windows(source, window_count, horizon_s)
            cfg = source["config"]
            step = cfg.native_step_s if model == "native_ocd" else cfg.reduced_step_s
            plant = make_plant(model, cfg)
            model_rows = []
            try:
                for window in windows:
                    row = {"scenario": scenario, "model": model, **window,
                           "source_model": model, "success": False, "failure": None,
                           "coarse_step_s": step, "fine_step_s": step/2}
                    initial = source["data"]["internal_state"][window["start_index"]]
                    reference = source["data"]["states"][window["end_index"]]
                    try:
                        coarse = replay_window(plant, initial, source, window, step)
                        fine = replay_window(plant, initial, source, window, step/2)
                        row.update(endpoint_errors(coarse, reference, "same_step_replay"))
                        row.update(endpoint_errors(fine, coarse, "half_step"))
                        row.update(endpoint_errors(fine, reference, "fine_vs_recorded"))
                        row["success"] = True
                    except (RuntimeError, ValueError, FloatingPointError) as error:
                        row["failure"] = str(error)
                    rows.append(row); model_rows.append(row)
            finally:
                plant.close()
            summary = {"scenario": scenario, "model": model,
                       "coarse_step_s": step, "fine_step_s": step/2,
                       **aggregate_errors(model_rows, "half_step")}
            for prefix in ("same_step_replay", "fine_vs_recorded"):
                summary.update({k:v for k,v in aggregate_errors(model_rows, prefix).items()
                                if k.startswith(prefix)})
            summary["same_step_replay_consistent"] = bool(
                summary["failed_windows"] == 0 and
                summary["same_step_replay_position_max_abs_m"] < 1e-6 and
                summary["same_step_replay_yaw_max_abs_deg"] < 1e-5 and
                summary["same_step_replay_speed_max_abs_mps"] < 1e-6 and
                summary["same_step_replay_yaw_rate_max_abs_radps"] < 1e-6)
            summaries.append(summary)
            if progress:
                print(f"Refinement {scenario}/{model}: {summary}", flush=True)
    return rows, summaries, provenance


def run_validation(output, scenarios=("nominal", "demanding"), mode="all",
                   forecast_windows=12, refinement_windows=3, horizon_s=1., progress=True):
    """Create/refresh validation reports without changing any simulation logs."""
    if mode not in {"all", "forecast", "refinement"}:
        raise ValueError("Unknown validation mode")
    output = Path(output).resolve()
    destination = output / "validation"
    summary_path = destination / "summary.json"
    report = json.loads(summary_path.read_text()) if summary_path.is_file() else {}
    # Keep provenance for an already executed section when the other section
    # is refreshed with a revised validation/reporting script.
    previous_source_hash = report.get("validation_source_sha256")
    for section, key in (("forecast_summary", "forecast_validation_source_sha256"),
                         ("integration_summary", "integration_validation_source_sha256")):
        if section in report and key not in report:
            report[key] = previous_source_hash
    report.update({
        "format_version": 1,
        "complete": False, "status": "running",
        "design": {
            "requested_scenarios": list(scenarios),
            "requested_horizon_s": horizon_s,
            "forecast_windows_per_scenario_and_model": forecast_windows,
            "refinement_windows_per_scenario_and_model": refinement_windows,
            "selection": "Forecasts: nearest samples to equally spaced start stations from 50 m to track length minus 50 m, restricted to complete windows; independent of observed errors.",
            "refinement_selection": "First and last station windows, plus one centered on greatest absolute recorded yaw rate; selected from source motion rather than numerical error. This includes a tight-turn case rather than checking only straights.",
            "forecast_reference": "Recorded native OCD full-state run, with unchanged actual interval inputs; no controller reoptimization.",
            "reduced_initialization": "Native [X,Y,psi,vx,vy,r] and four wheel speeds plus actual steering copied exactly; omitted suspension and relaxed-slip states cannot be represented by the reduced models.",
            "integration_reference": "Each model's own saved full states and its own recorded input history; same-step replay is checked separately before interpreting half-step differences.",
            "input_semantics": "inputs[i] is held over [time_s[i],time_s[i+1]); four torques use the experiment's original static-load allocation.",
            "interpretation": "Endpoint errors over selected one-second windows measure simulation model mismatch to OCD, not real-vehicle accuracy, global error bounds or lap-time optimality.",
        },
        "validation_source_sha256": _sha256(__file__),
        "models_source_sha256": _sha256(Path(__file__).with_name("models.py")),
        "python": platform.python_version(), "numpy": np.__version__,
    })
    _write_json(summary_path, report)
    if mode in {"all", "forecast"}:
        begin = time.perf_counter()
        rows, summaries, sources = forecast_checks(output, scenarios, forecast_windows, horizon_s, progress)
        _write_csv(destination / "window_predictions.csv", rows)
        report.update({"forecast_summary": summaries, "forecast_sources": sources,
                       "forecast_validation_source_sha256": _sha256(__file__),
                       "forecast_wall_seconds": time.perf_counter()-begin})
        _write_json(summary_path, report)
    if mode in {"all", "refinement"}:
        begin = time.perf_counter()
        rows, summaries, sources = refinement_checks(output, scenarios, refinement_windows, horizon_s, progress)
        _write_csv(destination / "integration_refinement.csv", rows)
        report.update({"integration_summary": summaries, "integration_sources": sources,
                       "integration_validation_source_sha256": _sha256(__file__),
                       "refinement_wall_seconds": time.perf_counter()-begin})
        _write_json(summary_path, report)
    report["complete"] = (
        len(report.get("forecast_summary", [])) == len(scenarios)*len(REDUCED_MODELS)
        and len(report.get("integration_summary", [])) == len(scenarios)*len(MODEL_NAMES))
    report["all_windows_successful"] = all(
        item["failed_windows"] == 0 for key in ("forecast_summary", "integration_summary")
        for item in report.get(key, []))
    report["status"] = "complete" if report["complete"] else "partial"
    _write_json(summary_path, report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"outputs/model_comparison")
    parser.add_argument("--scenarios", nargs="+", choices=("nominal", "demanding"),
                        default=["nominal", "demanding"])
    parser.add_argument("--mode", choices=("all", "forecast", "refinement"), default="all")
    parser.add_argument("--forecast-windows", type=int, default=12)
    parser.add_argument("--refinement-windows", type=int, default=3)
    parser.add_argument("--horizon", type=float, default=1.)
    args = parser.parse_args()
    run_validation(args.output, args.scenarios, args.mode, args.forecast_windows,
                   args.refinement_windows, args.horizon)


if __name__ == "__main__":
    main()
