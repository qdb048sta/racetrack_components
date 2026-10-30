"""Native OCD plant and a source-matched CasADi predictor.

The actual plant calls the compiled, unmodified upstream C++ components. The
symbolic equations below are used only for MPC and are checked against the native
derivatives. Source: pinned single-track equations, linear.cpp, default.cpp,
drivetrain_direct_torque_eqns.cpp and steering_actuator_pt1_model.cpp.
"""
from __future__ import annotations

import math
import numpy as np
from .config import VehicleConfig

STATE_COLUMNS = ("x_m", "y_m", "yaw_rad", "vx_mps", "vy_mps", "yaw_rate_radps",
                 "omega_fl_radps", "omega_fr_radps", "omega_rl_radps", "omega_rr_radps",
                 "steering_rad")


class OCDVehicle:
    """11 native states, with commands [steering target, requested force / mass]."""

    def __init__(self, config: VehicleConfig | None = None, integration_step_s: float = .0002):
        from .native import NativeVehicle
        self.config = VehicleConfig() if config is None else config
        self.config.validate()
        if not np.isfinite(integration_step_s) or integration_step_s <= 0:
            raise ValueError("Native integration step must be finite and positive.")
        self.integration_step_s = integration_step_s
        self.native = NativeVehicle(self.config.native_parameters())

    @staticmethod
    def observe(state):
        """The seven display states used by the existing replay, preserving order."""
        z = np.asarray(state, dtype=float)
        return np.concatenate([z[..., :6], z[..., 10:11]], axis=-1)

    def initial_state(self, body):
        body = np.asarray(body, dtype=float)
        if body.shape != (7,) or not np.isfinite(body).all():
            raise ValueError("Initial body state must contain seven finite values.")
        p = self.config
        delta = body[6] + p.steering_static_offset_rad
        vf = body[3]*np.cos(delta) + (body[4]+p.lf_m*body[5])*np.sin(delta)
        vr = body[3]
        wheels = np.maximum([vf, vf, vr, vr], 0) / p.wheel_radius_m
        return np.r_[body[:6], wheels, body[6]]

    def torques(self, command):
        """Control allocation, outside the plant: four commanded wheel torques."""
        p = self.config
        force = p.mass_kg * np.asarray(command)[1]
        front = p.lr_m / p.wheelbase_m
        return force * p.wheel_radius_m * np.array([front/2, front/2, (1-front)/2, (1-front)/2])

    @staticmethod
    def _friction(mu_front, mu_rear):
        mu = np.array([mu_front, mu_front, mu_rear, mu_rear], dtype=float)
        if not np.isfinite(mu).all() or np.any(mu <= 0):
            raise ValueError("Road friction multipliers must be finite and positive.")
        return mu

    def rhs(self, state, command, mu_front=1., mu_rear=1.):
        derivative, _ = self.native.evaluate(np.asarray(state, dtype=float), self.torques(command),
                                             float(command[0]), self._friction(mu_front, mu_rear))
        return derivative

    def diagnostics(self, state, command, mu_front=1., mu_rear=1.):
        _, d = self.native.evaluate(np.asarray(state, dtype=float), self.torques(command),
                                    float(command[0]), self._friction(mu_front, mu_rear))
        alpha, slip = d["tire_slip_angle_rad"], d["tire_longitudinal_slip"]
        fx, fy, fz = d["tire_force_longitudinal_N"], d["tire_force_lateral_N"], d["tire_force_vertical_N"]
        result = {"alpha_front_rad": float(np.mean(alpha[:2])), "alpha_rear_rad": float(np.mean(alpha[2:])),
                  "fx_front_n": float(sum(fx[:2])), "fx_rear_n": float(sum(fx[2:])),
                  "fy_front_n": float(sum(fy[:2])), "fy_rear_n": float(sum(fy[2:])),
                  "fz_front_n": float(sum(fz[:2])), "fz_rear_n": float(sum(fz[2:])),
                  "ax_body_mps2": float(d["acceleration_mps2"][0]),
                  "ay_body_mps2": float(d["acceleration_mps2"][1]),
                  "yaw_moment_nm": float(d["yaw_acceleration_radps2"]*self.config.yaw_inertia_kgm2)}
        for axle, section, mu in (("front", slice(0, 2), mu_front), ("rear", slice(2, 4), mu_rear)):
            result[axle+"_grip_utilization"] = float(np.hypot(sum(fx[section]), sum(fy[section])) /
                                                       (self.config.nominal_friction*mu*sum(fz[section])))
        for j, name in enumerate(("fl", "fr", "rl", "rr")):
            result["longitudinal_slip_"+name] = float(slip[j])
            result["omega_"+name+"_radps"] = float(state[6+j])
        return result

    def step(self, state, command, dt, mu_front=1., mu_rear=1.):
        p = self.config
        state, command = np.asarray(state, dtype=float), np.asarray(command, dtype=float)
        if state.shape != (11,) or command.shape != (2,) or not np.isfinite(state).all() or not np.isfinite(command).all():
            raise ValueError("Expected finite native state[11] and command[2].")
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive.")
        if abs(command[0]) > p.steering_limit_rad+1e-8 or not -p.braking_limit_mps2-1e-8 <= command[1] <= p.acceleration_limit_mps2+1e-8:
            raise ValueError("Command outside configured limits.")
        mu = self._friction(mu_front, mu_rear)
        self.native.reset(state, mu)
        result = self.native.step(self.torques(command), float(command[0]), dt,
                                  substeps=math.ceil(dt/self.integration_step_s-1e-10), friction=mu)
        if not np.isfinite(result).all():
            raise FloatingPointError("Nonfinite native OCD state.")
        return result

    def symbolic_rhs(self):
        """Exact continuous component laws; the controller discretizes with Radau IIA."""
        import casadi as ca
        p = self.config
        z, u, mu = ca.SX.sym("z", 11), ca.SX.sym("u", 2), ca.SX.sym("mu", 2)
        vx, vy, r, delta = z[3], z[4], z[5], z[10]+p.steering_static_offset_rad
        cf, sf = ca.cos(delta), ca.sin(delta)
        front_vx = cf*vx+sf*(vy+p.lf_m*r)
        front_vy = -sf*vx+cf*(vy+p.lf_m*r)
        rear_vx, rear_vy = vx, vy-p.lr_m*r
        vtx, vty = [front_vx]*2+[rear_vx]*2, [front_vy]*2+[rear_vy]*2
        aero_x = -.5*p.air_density_kgpm3*p.aero_area_m2*p.aero_drag_coefficient*vx*ca.fabs(vx)
        aero_y = -.5*p.air_density_kgpm3*p.aero_area_m2*p.aero_drag_coefficient*vy*ca.fabs(vy)
        aero_z = .5*p.air_density_kgpm3*p.aero_area_m2*p.aero_lift_coefficient*vx**2
        total_load = p.mass_kg*p.gravity_mps2-aero_z
        front_load = (total_load*p.lr_m-aero_z*p.aero_center_offset_m)/p.wheelbase_m
        loads = [ca.fmax(.001, front_load/2)]*2+[ca.fmax(.001, (total_load-front_load)/2)]*2
        torque_fractions = [p.lr_m/p.wheelbase_m/2]*2+[p.lf_m/p.wheelbase_m/2]*2
        fx, fy = [], []
        at_rest = ca.logic_and(vx**2 < .001, vy**2 < .001)
        for j in range(4):
            at_rest = ca.logic_and(at_rest, z[6+j]**2 < .001)
        for j in range(4):
            alpha = -ca.atan2(vty[j], ca.fmax(1., vtx[j]))
            slip = ca.fmin(1., ca.fmax(-1., (p.wheel_radius_m*z[6+j]-vtx[j])/ca.fmax(3., vtx[j])))
            scale = mu[0 if j < 2 else 1]
            fx.append(ca.if_else(at_rest, 0., scale*p.longitudinal_slip_coefficient*slip*loads[j]))
            fy.append(ca.if_else(at_rest, 0., scale*p.cornering_coefficient_per_rad*alpha*loads[j]))
        fx_front, fy_front = fx[0]+fx[1], fy[0]+fy[1]
        front_x, front_y = cf*fx_front-sf*fy_front, sf*fx_front+cf*fy_front
        body_fx, body_fy = front_x+fx[2]+fx[3]+aero_x, front_y+fy[2]+fy[3]+aero_y
        yaw_moment = p.lf_m*front_y-p.lr_m*(fy[2]+fy[3])
        cp, sp = ca.cos(z[2]), ca.sin(z[2])
        rhs = [vx*cp-vy*sp, vx*sp+vy*cp, r,
               body_fx/p.mass_kg+r*vy, body_fy/p.mass_kg-r*vx, yaw_moment/p.yaw_inertia_kgm2]
        extra_inertia = ca.fmax(0., 15*p.wheel_inertia_rear_kgm2*(1.-ca.sum1(z[6:10])/80.))
        for j in range(4):
            load_torque = (fx[j]+loads[j]*p.rolling_resistance_coefficient)*p.wheel_radius_m
            torque = p.mass_kg*u[1]*p.wheel_radius_m*torque_fractions[j]
            inertia = (p.wheel_inertia_front_kgm2 if j < 2 else p.wheel_inertia_rear_kgm2)+extra_inertia
            rate = (torque-load_torque)/inertia
            rhs.append(ca.if_else(z[6+j] < .00001, ca.fmax(0., rate), rate))
        steering_rate = ca.fmin(p.steering_rate_limit_radps, ca.fmax(-p.steering_rate_limit_radps,
                            (u[0]-z[10])/p.steering_time_constant_s))
        steering_rate = ca.if_else(z[10] <= -p.steering_limit_rad, ca.fmax(0., steering_rate), steering_rate)
        steering_rate = ca.if_else(z[10] >= p.steering_limit_rad, ca.fmin(0., steering_rate), steering_rate)
        rhs.append(steering_rate)
        return ca.Function("ocd_native_component_rhs", [z, u, mu], [ca.vertcat(*rhs)])
