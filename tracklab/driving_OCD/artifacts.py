"""Native-state, controller, provenance and standalone HTML outputs."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import numpy as np
from ..driving.artifacts import write_csv
from .native import SOURCE_COMMIT, MODEL_NAME, STATE_NAMES
from .replay import build_replay
from .validation import diagnose, check_recorded_input_replay


def save_results(road, result, output, independent_check=True):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    raw = {k: result[k] for k in ("time_s", "states", "native_states", "inputs", "progress_m")}
    np.savez_compressed(output/"run_raw.npz", **raw)
    metrics, diag = diagnose(road, raw, result["plant"], result["solves"])
    metrics.update({k: result[k] for k in ("lap_crossing_time_s", "simulation_wall_seconds")})
    metrics.update(native_model=MODEL_NAME, upstream_commit=SOURCE_COMMIT)
    np.savez_compressed(output/"diagnostics.npz", **diag)
    for name in ("solves", "predictions", "vehicle_config", "driving_config"):
        (output/(name+".json")).write_text(json.dumps(result[name], indent=2, allow_nan=False), encoding="utf-8")
    (output/"metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False), encoding="utf-8")
    parameters = result["plant"].native.parameters()
    (output/"native_parameters.json").write_text(json.dumps(parameters, indent=2), encoding="utf-8")
    library = result["plant"].native.library_path
    source = Path(__file__).with_name("native")/"upstream.json"
    provenance = {"repository": "https://github.com/TUMFTM/Open-Car-Dynamics", "commit": SOURCE_COMMIT,
                  "composition": MODEL_NAME, "native_state_columns": list(STATE_NAMES),
                  "native_library_sha256": hashlib.sha256(library.read_bytes()).hexdigest(),
                  "source_manifest": json.loads(source.read_text(encoding="utf-8")),
                  "plant_integration": "Unmodified upstream DoPri; zero-duration output/cache synchronization at adapter boundaries",
                  "prediction": "Source-matched continuous equations, two-stage Radau IIA collocation",
                  "road_geometry": "Original PeriodicTrack, interpolation and geometry modules unchanged"}
    (output/"provenance.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8")
    headers = ["time_s", "x_m", "y_m", "yaw_rad", "vx_mps", "vy_mps", "yaw_rate_radps", "steering_rad",
               "steering_target_rad", "force_acceleration_mps2", *diag]
    write_csv(output/"trajectory.csv", headers, np.column_stack([raw["time_s"], raw["states"], raw["inputs"], *diag.values()]))
    write_csv(output/"native_states.csv", ["time_s", *STATE_NAMES], np.column_stack([raw["time_s"], raw["native_states"]]))
    keys = list(result["solves"][0])
    write_csv(output/"solver_log.csv", keys, ([r[k] for k in keys] for r in result["solves"]))
    plan = result["speed_plan"]
    write_csv(output/"speed_schedule.csv", ["progress_m", "curvature_envelope_1pm", "speed_limit_mps", "planned_speed_mps"],
              np.column_stack([plan.s, plan.curvature_envelope, plan.raw_limit, plan.speed]))
    if independent_check:
        check = check_recorded_input_replay(road, raw, result["plant"], result["driving_config"]["control_period_s"])
        (output/"independent_integration_check.json").write_text(json.dumps(check, indent=2), encoding="utf-8")
    build_replay(road, raw, diag, metrics, result["predictions"], result["vehicle_config"],
                 result["driving_config"], output/"norisring_replay.html")
    return metrics
