"""Tracking NMPC with all eleven OCD states and implicit Radau IIA prediction.

The native linear-tire/wheel ODE is stiff. Two-stage, third-order, L-stable
collocation avoids the unstable large explicit RK steps of a reduced predictor.
Actual vehicle motion always comes from the upstream C++ plant.
"""
from __future__ import annotations
import math
import time
import numpy as np
from ..geometry import wrap_angle


class TrackingMPC:
    def __init__(self, plant, road, speed_plan, config):
        import casadi as ca
        self.plant, self.road, self.speed_plan, self.config = plant, road, speed_plan, config
        p, N, dt = plant.config, config.horizon_steps, config.control_period_s
        op = ca.Opti()
        X, U = op.variable(11, N+1), op.variable(2, N)
        init, ref = op.parameter(11), op.parameter(11, N+1)
        normals, widths, mus, previous = op.parameter(2, N+1), op.parameter(2, N+1), op.parameter(2, N), op.parameter(2)
        f = plant.symbolic_rhs()
        op.subject_to(X[:, 0] == init)
        objective, stages = 0, []
        for j in range(N):
            left = X[:, j]
            h = dt/config.prediction_substeps
            for k in range(config.prediction_substeps):
                middle = op.variable(11)
                right = X[:, j+1] if k == config.prediction_substeps-1 else op.variable(11)
                f1, f2 = f(middle, U[:, j], mus[:, j]), f(right, U[:, j], mus[:, j])
                op.subject_to(middle == left+h*(5*f1/12-f2/12))
                op.subject_to(right == left+h*(3*f1/4+f2/4))
                stages.append((middle, j, (k+1/3)/config.prediction_substeps))
                if k < config.prediction_substeps-1:
                    stages.append((right, j, (k+1)/config.prediction_substeps))
                left = right
            du = U[:, j]-(previous if j == 0 else U[:, j-1])
            objective += 5*du[0]**2+.03*du[1]**2+.02*U[0, j]**2+.008*U[1, j]**2
        for j in range(N+1):
            dx, normal = X[:2, j]-ref[:2, j], normals[:, j]
            e, along = ca.dot(normal, dx), ca.dot(ca.vertcat(normal[1], -normal[0]), dx)
            scale = 3 if j == N else 1
            objective += scale*(35*e**2+.5*along**2+12*(X[2, j]-ref[2, j])**2
                                +1.8*(X[3, j]-ref[3, j])**2+.15*(X[4, j]-ref[4, j])**2
                                +.8*(X[5, j]-ref[5, j])**2+.15*(X[10, j]-ref[10, j])**2)
            op.subject_to(op.bounded(-widths[0, j]+config.corridor_center_margin_m,
                                    e, widths[1, j]-config.corridor_center_margin_m))
        op.subject_to(op.bounded(-p.steering_limit_rad, U[0, :], p.steering_limit_rad))
        op.subject_to(op.bounded(-p.braking_limit_mps2, U[1, :], p.acceleration_limit_mps2))
        op.subject_to(op.bounded(-p.steering_limit_rad, X[10, :], p.steering_limit_rad))
        op.subject_to(op.bounded(1., X[3, :], max(25., config.maximum_speed_mps+5)))
        op.subject_to(op.bounded(-3., X[5, :], 3.))
        op.subject_to(op.bounded(0., X[6:10, :], 2*max(25., config.maximum_speed_mps+5)/p.wheel_radius_m))
        op.minimize(objective)
        op.solver("ipopt", {"expand": True, "print_time": False},
                  {"print_level": 0, "sb": "yes", "max_iter": config.maximum_solver_iterations,
                   "tol": 1e-6, "acceptable_tol": 1e-5, "acceptable_iter": 3,
                   "warm_start_init_point": "yes", "mu_init": .01})
        self.op, self.X, self.U, self.init, self.ref = op, X, U, init, ref
        self.normals, self.widths, self.mus, self.previous, self.stages = normals, widths, mus, previous, stages
        self.last_global_prediction = self.last_inputs = None

    def solve(self, z, s, previous_input):
        p, d = self.plant.config, self.config
        N = d.horizon_steps
        ss, vv = self.speed_plan.references(s, np.hypot(z[3], z[4]), N, d.control_period_s)
        q = self.road.sample_geometry(ss)
        origin, theta = q["center"][0], float(q["heading_rad"][0])
        ct, st = math.cos(theta), math.sin(theta)
        rot = np.array([[ct, -st], [st, ct]])
        centers, normals = (q["center"]-origin)@rot, q["normal_left"]@rot
        headings = np.unwrap(q["heading_rad"]-theta)
        headings -= 2*np.pi*round(headings[0]/(2*np.pi))
        k = q["curvature_1pm"]
        beta = np.arctan(p.lr_m*k)-vv**2*k/(p.gravity_mps2*p.cornering_coefficient_per_rad)
        steer = np.clip(np.arctan(p.wheelbase_m*k), -p.steering_limit_rad, p.steering_limit_rad)
        body = np.column_stack([centers, headings-beta, vv*np.cos(beta), vv*np.sin(beta), vv*k, steer])
        reference = np.array([self.plant.initial_state(row) for row in body]).T
        initial = np.asarray(z, dtype=float).copy()
        initial[:2], initial[2] = (z[:2]-origin)@rot, float(wrap_angle(z[2]-theta))
        reference[2] += 2*np.pi*round((initial[2]-reference[2, 0])/(2*np.pi))
        axes = q["tangent"][:-1]
        mf = self.road.surface.sample_xy(q["center"][:-1]+p.lf_m*axes)["friction_multiplier"]
        mr = self.road.surface.sample_xy(q["center"][:-1]-p.lr_m*axes)["friction_multiplier"]
        op = self.op
        op.set_value(self.init, initial)
        op.set_value(self.ref, reference)
        op.set_value(self.normals, normals.T)
        op.set_value(self.widths, np.vstack([q["width_right_m"], q["width_left_m"]]))
        op.set_value(self.mus, np.vstack([mf, mr]))
        op.set_value(self.previous, previous_input)
        if self.last_global_prediction is None:
            guess = reference.copy()
            resist = p.rolling_resistance_coefficient*p.gravity_mps2 + .5*p.air_density_kgpm3*p.drag_coefficient_area_m2*vv[:-1]**2/p.mass_kg
            controls = np.vstack([steer[:-1], resist])
        else:
            glob = np.column_stack([self.last_global_prediction[:, 1:], self.last_global_prediction[:, -1]])
            guess = glob.copy()
            guess[:2] = ((glob[:2].T-origin)@rot).T
            guess[2] = np.unwrap(glob[2]-theta)
            guess[2] += 2*np.pi*round((initial[2]-guess[2, 0])/(2*np.pi))
            controls = np.column_stack([self.last_inputs[:, 1:], self.last_inputs[:, -1]])
        guess[:, 0] = initial
        op.set_initial(self.X, guess)
        op.set_initial(self.U, controls)
        for stage, j, fraction in self.stages:
            op.set_initial(stage, (1-fraction)*guess[:, j]+fraction*guess[:, j+1])
        start = time.perf_counter()
        try:
            solution = op.solve()
        except RuntimeError as exc:
            raise RuntimeError(f"OCD tracking MPC failed at {s:.2f} m: {op.stats().get('return_status')}") from exc
        elapsed = time.perf_counter()-start
        states, inputs = np.asarray(solution.value(self.X)), np.asarray(solution.value(self.U))
        gv = np.asarray(solution.value(op.g)).ravel()
        lower, upper = np.asarray(solution.value(op.lbg)).ravel(), np.asarray(solution.value(op.ubg)).ravel()
        violation = float(max(0., np.max(lower-gv), np.max(gv-upper)))
        if violation > 2e-5 or not np.isfinite(states).all():
            raise RuntimeError(f"Unacceptable OCD MPC residual {violation} at {s:.2f} m.")
        glob = states.copy()
        glob[:2], glob[2] = (states[:2].T@rot.T+origin).T, states[2]+theta
        self.last_global_prediction, self.last_inputs = glob, inputs
        info = {"success": bool(op.stats()["success"]), "status": op.stats()["return_status"],
                "iterations": int(op.stats()["iter_count"]), "solve_seconds": elapsed,
                "constraint_violation": violation, "objective": float(solution.value(op.f)),
                "predicted_xy_m": glob[:2].T.tolist(), "reference_s_m": ss.tolist(),
                "reference_speed_mps": float(self.speed_plan(s))}
        return inputs[:, 0].copy(), info
