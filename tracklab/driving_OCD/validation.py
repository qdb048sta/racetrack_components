"""Road diagnostics and an independent integration check of the native plant."""
from __future__ import annotations
import time
import numpy as np
from scipy.integrate import trapezoid
from ..geometry import wrap_angle
from ..driving.validation import project_near_batch, footprint_points
from .simulation import axle_friction
from .vehicle import OCDVehicle


def diagnose(road, raw, plant, solves):
    ts, zs, ns, us = raw["time_s"], raw["states"], raw["native_states"], raw["inputs"]
    s, e, resid, _ = project_near_batch(road, zs[:, :2], raw["progress_m"])
    q = road.sample_geometry(s)
    p, bp = plant.config, footprint_points(plant.config)
    cs, sn = np.cos(zs[:, 2]), np.sin(zs[:, 2])
    minimum, residual, outside = np.inf, float(np.max(resid)), 0
    for start in range(0, len(ts), 500):
        end = min(start+500, len(ts)); n = end-start
        rotation = np.stack([np.stack([cs[start:end], -sn[start:end]], axis=-1),
                             np.stack([sn[start:end], cs[start:end]], axis=-1)], axis=1)
        points = np.einsum("nij,kj->nki", rotation, bp)+zs[start:end, None, :2]
        _, be, rr, bu = project_near_batch(road, points, np.broadcast_to(s[start:end, None], (n, len(bp))))
        widths = road.width(bu)
        margin = np.minimum(widths[..., 1]-be, widths[..., 0]+be)
        minimum, outside = min(minimum, float(np.min(margin))), outside+int(np.count_nonzero(margin < 0))
        residual = max(residual, float(np.max(rr)))
    values = [plant.diagnostics(z, u, *axle_friction(plant, road, z)) for z, u in zip(ns, us)]
    dyn = {k: np.array([v[k] for v in values]) for k in values[0]}
    beta, speed = np.arctan2(zs[:, 4], zs[:, 3]), np.hypot(zs[:, 3], zs[:, 4])
    metrics = {
        "completed_laps_by_projection": float((s[-1]-s[0])/road.length),
        "reference_length_m": float(road.length), "actual_distance_integrated_m": float(trapezoid(speed, ts)),
        "maximum_speed_mps": float(np.max(speed)), "minimum_speed_mps": float(np.min(speed)),
        "maximum_abs_cross_track_error_m": float(np.max(np.abs(e))), "rms_cross_track_error_m": float(np.sqrt(np.mean(e**2))),
        "maximum_abs_sideslip_deg": float(np.rad2deg(np.max(np.abs(beta)))),
        "maximum_abs_steering_deg": float(np.rad2deg(np.max(np.abs(zs[:, 6]+p.steering_static_offset_rad)))),
        "maximum_abs_yaw_rate_radps": float(np.max(np.abs(zs[:, 5]))),
        "maximum_abs_lateral_acceleration_mps2": float(np.max(np.abs(dyn["ay_body_mps2"]))),
        "maximum_abs_front_slip_deg": float(np.rad2deg(np.max(np.abs(dyn["alpha_front_rad"])))),
        "maximum_abs_rear_slip_deg": float(np.rad2deg(np.max(np.abs(dyn["alpha_rear_rad"])))),
        "maximum_grip_utilization": float(max(np.max(dyn["front_grip_utilization"]), np.max(dyn["rear_grip_utilization"]))),
        "minimum_center_cross_section_margin_m": float(np.min(np.minimum(q["width_left_m"]-e, q["width_right_m"]+e))),
        "minimum_sampled_body_cross_section_margin_m": minimum, "body_perimeter_samples_per_state": len(bp),
        "body_samples_outside_local_road": outside, "maximum_projection_tangential_residual_m": residual,
        "minimum_center_frenet_jacobian": float(np.min(1-q["curvature_1pm"]*e)),
        "number_of_integrated_states": len(ts), "number_of_mpc_solves": len(solves),
        "successful_mpc_solves": sum(v["success"] for v in solves),
        "max_solver_constraint_violation": max(v["constraint_violation"] for v in solves),
        "median_mpc_solve_time_s": float(np.median([v["solve_seconds"] for v in solves])),
        "max_mpc_solve_time_s": float(max(v["solve_seconds"] for v in solves)),
        "native_state_count": 11,
        "diagnostic_limitations": "Sampled local-width checks; linear tires have no friction saturation. Grip utilization is diagnostic only.",
    }
    diag = {"progress_m": s, "cross_track_error_m": e, "heading_error_rad": wrap_angle(zs[:, 2]-q["heading_rad"]),
            "sideslip_rad": beta, "speed_mps": speed, **dyn}
    return metrics, diag


def check_recorded_input_replay(road, raw, plant, control_period_s):
    """Open-loop replay with half the native DoPri substep and identical controls.

    This tests native timestep sensitivity, including its preserved cached-stage
    composition. It does not equate that composition with a different integrator.
    """
    finer = OCDVehicle(plant.config, plant.integration_step_s/2)
    cur = raw["native_states"][0].copy()
    ts, us = raw["time_s"], raw["inputs"]
    maximum, max_position = np.zeros(11), 0.
    start = time.perf_counter()
    try:
        for i in range(len(ts)-1):
            cur = finer.step(cur, us[i], float(ts[i+1]-ts[i]), *axle_friction(finer, road, cur))
            delta = np.abs(cur-raw["native_states"][i+1])
            maximum = np.maximum(maximum, delta)
            max_position = max(max_position, float(np.linalg.norm(cur[:2]-raw["native_states"][i+1, :2])))
    finally:
        finer.native.close()
    return {"method": "Recorded controls; unmodified native DoPri at half the original substep",
            "original_native_step_s": plant.integration_step_s, "refined_native_step_s": plant.integration_step_s/2,
            "logged_intervals": len(ts)-1, "maximum_endpoint_position_difference_m": max_position,
            "maximum_endpoint_state_component_differences": maximum.tolist(), "wall_seconds": time.perf_counter()-start,
            "limitations": "Open-loop timestep refinement; no controller reoptimization or real-vehicle validation."}
