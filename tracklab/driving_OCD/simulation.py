"""Closed-loop native TUM single-track/linear-tire simulation in time."""
from __future__ import annotations
import math
import time
from dataclasses import asdict
import numpy as np
from .config import VehicleConfig, DrivingConfig
from .vehicle import OCDVehicle
from .planning import CurvatureSpeedPlan
from .controller import TrackingMPC


def axle_friction(plant, road, z):
    p = plant.config
    direction = np.array([np.cos(z[2]), np.sin(z[2])])
    points = np.array([z[:2]+p.lf_m*direction, z[:2]-p.lr_m*direction])
    mu = road.surface.sample_xy(points)["friction_multiplier"]
    return float(mu[0]), float(mu[1])


def simulate(road, vehicle_config=None, driving_config=None, progress_callback=None):
    p = VehicleConfig() if vehicle_config is None else vehicle_config
    d = DrivingConfig() if driving_config is None else driving_config
    p.validate(); d.validate()
    plant, plan = OCDVehicle(p, d.integration_step_s), CurvatureSpeedPlan(road, d)
    controller = TrackingMPC(plant, road, plan, d)
    q0 = road.sample_geometry(0.)
    xy = q0["center"]+d.initial_lateral_offset_m*q0["normal_left"]
    z = plant.initial_state([*xy, float(q0["heading_rad"]), min(d.initial_speed_mps, float(plan(0))), 0, 0, 0])
    previous = np.zeros(2)
    states, native_states, times, commands = [plant.observe(z)], [z.copy()], [0.], [previous.copy()]
    progresses, solves, predictions = [0.], [], []
    t, s, target = 0., 0., road.length*d.laps
    start, s_last_report, finished = time.perf_counter(), -1e3, False
    for _ in range(math.ceil(d.maximum_time_s/d.control_period_s)):
        u, info = controller.solve(z, s, previous)
        commands[-1] = u.copy()
        info.update(time_s=t, progress_m=s)
        solves.append({k: v for k, v in info.items() if k not in ("predicted_xy_m", "reference_s_m")})
        predictions.append({"time_s": t, "xy_m": info["predicted_xy_m"]})
        nlog = int(round(d.control_period_s/d.logging_step_s))
        s_start, progress_rate = s, max(.1, float(np.hypot(z[3], z[4])))
        for j in range(nlog):
            z = plant.step(z, u, d.logging_step_s, *axle_friction(plant, road, z))
            t += d.logging_step_s
            if z[3] < .5 or abs(z[4]) > 8 or abs(z[5]) > 3:
                raise RuntimeError("Native vehicle left the forward-driving operating range.")
            native_states.append(z.copy()); states.append(plant.observe(z)); times.append(t); commands.append(u.copy())
            progresses.append(s_start+(j+1)*d.logging_step_s*progress_rate)
        projected = road.project(z[:2], hint_s=s+progress_rate*d.control_period_s, window_m=d.projection_window_m)
        new_s = projected["s_m"]
        if abs(new_s-s) > max(10, 3*progress_rate*d.control_period_s):
            raise RuntimeError("Unexpected projection branch jump.")
        for j in range(nlog):
            progresses[-nlog+j] = s+(new_s-s)*(j+1)/nlog
        s, previous = new_s, u
        q = road.sample_geometry(s)
        if not -float(q["width_right_m"])-1 <= projected["e_left_m"] <= float(q["width_left_m"])+1:
            raise RuntimeError(f"Native car left the road neighborhood at {s:.2f} m.")
        if progress_callback and (s-s_last_report > 200 or s >= target):
            progress_callback({"progress_m": s, "target_m": target, "time_s": t,
                               "speed_mps": float(np.hypot(z[3], z[4])),
                               "e_left_m": projected["e_left_m"], "solves": len(solves)})
            s_last_report = s
        if s >= target:
            finished = True
            break
    if not finished:
        raise RuntimeError(f"Time limit reached at {s:.1f}/{target:.1f} m.")
    ts, ss = np.asarray(times), np.asarray(progresses)
    crossing = np.flatnonzero(ss >= target)[0]
    lap_time = float(np.interp(target, ss[crossing-1:crossing+1], ts[crossing-1:crossing+1]))
    return {"time_s": ts, "states": np.asarray(states), "native_states": np.asarray(native_states),
            "inputs": np.asarray(commands), "progress_m": ss, "solves": solves, "predictions": predictions,
            "speed_plan": plan, "plant": plant, "vehicle_config": asdict(p), "driving_config": asdict(d),
            "lap_crossing_time_s": lap_time, "simulation_wall_seconds": time.perf_counter()-start}
