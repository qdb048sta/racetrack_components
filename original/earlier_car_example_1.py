#!/usr/bin/env python3
"""Example 1: dry double-lane-change road; modified single track and linear tires.

Standalone teaching adaptation of TUMFTM/Open-Car-Dynamics at:
94f8fb187fb0ed22bba1d809bd74f66d1ff75af4 (not a native C++ binding).
Original relevant implementation: Copyright 2026 Simon Sagmeister, Apache-2.0.
This file is MODIFIED: Python/CasADi translation, reduced plant, new road,
ideal common wheel-slip input, steering-rate input, and minimum-time MPC.
Full source provenance and modifications are in SOURCE_MAP below.

Install: python -m pip install numpy scipy matplotlib casadi
Run:     python example_1_linear_tire_mpc.py --output results
Test:    python example_1_linear_tire_mpc.py --self-test

An actual minimum-time objective is used: integral dX / Xdot. No prescribed
speed profile, no progress-reward surrogate, and no tracking term in the cost.
Re-solve all remaining road after every spatial segment and apply ONLY the
first control. Sampling is in space, NOT a constant time sampling period.
IPOPT produces local numerical solutions; global minimum time is NOT certified.
Road containment is conservative (a circumscribed vehicle disc) and is checked
at integration substeps with an explicit scaled 0.002 numerical backoff.
Independent DOP853 simulation checks the original limits on a finer grid;
these sampled checks are not a proof of continuous-time feasibility/stability.
The full upstream 28-state double-track plant and native build are NOT tested.
This is educational offline simulation, not a controller for a physical car.
"""
from __future__ import annotations

import os
# Avoid large BLAS thread pools in small NLPs; respects explicit user settings.
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any

try:
    import casadi as ca
    import numpy as np
    from scipy.integrate import solve_ivp
except ImportError as exc:
    raise SystemExit("Missing dependency. Run: python -m pip install numpy scipy matplotlib casadi") from exc

EXAMPLE = 1
COMMIT = "94f8fb187fb0ed22bba1d809bd74f66d1ff75af4"
ROOT = f"https://github.com/TUMFTM/Open-Car-Dynamics/blob/{COMMIT}/"
SOURCE_MAP = {
    "body_rhs": {
        "source": ROOT + "lib/ocd_vehicle_dynamics_single_track_cpp/include/ocd_vehicle_dynamics_single_track_cpp/vehicle_dynamics_equations_impl.hpp#L333-L349",
        "what": "Preserve the six planar body equations; use X as independent variable and append steering state.",
    },
    "wheel_velocities": {
        "source": ROOT + "lib/ocd_vehicle_dynamics_double_track_cpp/include/ocd_vehicle_dynamics_double_track_cpp/vehicle_dynamics_equations_impl.hpp#L103-L140",
        "what": "u_i=vx-y_i*r, v_i=vy+x_i*r; rotate by -delta_i. Example 1 sets y_i=0, example 2 retains track width.",
    },
    "slip_angle": {
        "source": ROOT + "lib/ocd_vehicle_dynamics_single_track_cpp/include/ocd_vehicle_dynamics_single_track_cpp/vehicle_dynamics_equations_impl.hpp#L138-L148",
        "what": "alpha=-atan2(v_tire_y,max(1,v_tire_x)); forward-speed constraints keep the max branch inactive at solutions.",
    },
    "linear_tire": {
        "source": ROOT + "lib/ocd_tire_models_cpp/src/linear.cpp#L4-L19",
        "what": "Fx=C_sx*kappa*Fz, Fy=C_alpha*alpha*Fz. Use positive restoring C_alpha=14, not upstream default -0.1. Add domain and friction-circle constraints.",
    },
    "nonlinear_tire": {
        "source": ROOT + "lib/ocd_tire_models_cpp/src/mf_simple_extended.cpp#L4-L31",
        "what": "Retain combined-slip construction and load-sensitive D; replace the small-slip branch with smooth sqrt(kappa^2+tan(alpha)^2+epsilon^2).",
    },
    "magic_formula": {
        "source": ROOT + "lib/ocd_tire_models_cpp/src/mf_simple.cpp#L4-L34",
        "what": "Retain sin(C*atan(B*s-E*(B*s-atan(B*s)))) and lateral tan(alpha). Set D=1 rather than the default 1.7.",
    },
    "road_interface": {
        "source": ROOT + "lib/ocd_types_cpp/include/ocd_types_cpp/types.hpp#L6-L13",
        "what": "A new Road object samples lambda_mue at each tire. Road height is identically zero in these demos.",
    },
    "steering": {
        "source": ROOT + "lib/ocd_steering_actuator_pt1_cpp/src/steering_actuator_pt1_model.cpp#L9-L22",
        "what": "Replace PT1 lag by bounded steering-rate actuation delta_dot=nu; Gerdts (7.18). This is not an identical actuator.",
    },
    "drivetrain": {
        "source": ROOT + "lib/ocd_drivetrain_fx_cpp/include/ocd_drivetrain_fx_cpp/drivetrain_fx_model.hpp#L34-L51",
        "what": "REPLACED by ideal common all-wheel slip-ratio actuation. No wheel inertia, RWD-LSD, engine, or gear dynamics retained.",
    },
}
BOOK_MAP = {
    "Gerdts": "2nd ed.: Problem 1.1.8 pp13-14; change of time (1.53) p38; RK Sec4.1.2 p161; full discretization Problem5.1.1 p215; NMPC Alg6.3.3 p345; car Example7.2.4 pp394-399, equations(7.12)-(7.23). Spatial reparameterization here is a chain-rule construction, not a quoted numbered theorem.",
    "Nocedal_Wright": "2nd ed.: Thm2.1 Taylor p14; Ch8 derivatives p193; Thm12.1 KKT p321; Alg18.1 SQP p532 (alternative, NOT this solver); Ch19 nonlinear interior-point methods p563 (this solver family).",
    "Gruene_Pannek": "2nd ed.: (2.6),(2.8),Thm2.7 pp16-21 sampling; Alg3.1 p47 receding horizon; Def3.6 p52 constraint functions; Ch8 p221 economic objectives; Sec12.1 pp367-379 transcription. Thm3.5 p51 / Thm4.11 p81 / Thm5.13 p103 are conditional theory, not certificates proved by this script.",
}


@dataclass(frozen=True)
class CarParams:
    # Kept from the inspected upstream single-track defaults.
    mass: float = 800.0
    iz: float = 1000.0
    wheelbase: float = 3.2
    lf: float = 1.724
    gravity: float = 9.81
    # New educational vehicle/road parameters, not calibrated AV21 values.
    track: float = 1.6
    cg_height: float = 0.32
    half_length: float = 2.15
    half_width: float = 0.85
    cd_area: float = 0.75
    rho_air: float = 1.225
    rolling_coefficient: float = 0.015
    c_alpha: float = 14.0       # normalized by Fz, units 1/rad
    c_kappa: float = 25.0      # normalized longitudinal stiffness
    bx: float = 20.0
    cx: float = 2.0
    ex: float = 0.7
    by: float = 10.0
    cy: float = 1.6
    ey: float = -1.9
    d0: float = 1.0
    pdy2: float = -0.10
    slip_epsilon: float = 1e-7
    load_tau: float = 0.18
    speed_min: float = 6.0
    speed_max: float = 30.0
    steer_max: float = 0.30
    steer_rate_max: float = 0.50
    acceleration_max: float = 4.0
    braking_max: float = 7.0
    power_max: float = 70000.0
    beta_max: float = 0.12
    min_wheel_load: float = 200.0
    safety_margin: float = 0.06
    constraint_backoff: float = 0.002  # scaled guard used only by the optimizer

    @property
    def lr(self) -> float:
        return self.wheelbase - self.lf

    @property
    def enclosing_radius(self) -> float:
        return math.hypot(self.half_length, self.half_width)


@dataclass(frozen=True)
class Road:
    """Planar graph road |Y-c(X)| <= half_width, z_road=0.

    slope_bound is a GLOBAL upper bound on |dc/dX| (sum of tanh slopes).
    Thus |Y-c(X)|+R*sqrt(1+slope_bound^2)<=half_width guarantees that a
    disc of radius R centered at (X,Y) is wholly inside the graph corridor.
    It follows from the mean-value bound and Cauchy-Schwarz, not from an
    unsupported claim that checking four corners always suffices.
    """
    example: int = EXAMPLE

    @property
    def length(self) -> float:
        return 112.0 if self.example == 1 else 144.0

    @property
    def half_width(self) -> float:
        return 4.1 if self.example == 1 else 5.0

    @property
    def slope_bound(self) -> float:
        return 0.5 if self.example == 1 else 1.0

    def center(self, x: Any, backend: Any = np) -> Any:
        if self.example == 1:
            return 2.0 * (backend.tanh((x - 32.0) / 8.0) - backend.tanh((x - 76.0) / 8.0))
        return 2.0 * (backend.tanh((x - 32.0) / 8.0)
                      - 2.0 * backend.tanh((x - 72.0) / 8.0)
                      + backend.tanh((x - 112.0) / 8.0))

    def slope(self, x: Any) -> Any:
        if self.example == 1:
            return 0.25 * ((1 - np.tanh((x - 32.0) / 8.0)**2)
                           - (1 - np.tanh((x - 76.0) / 8.0)**2))
        return 0.25 * ((1 - np.tanh((x - 32.0) / 8.0)**2)
                       - 2*(1 - np.tanh((x - 72.0) / 8.0)**2)
                       + (1 - np.tanh((x - 112.0) / 8.0)**2))

    def friction(self, x: Any, y: Any = 0.0, backend: Any = np) -> Any:
        # y is an interface argument for future two-dimensional friction maps.
        if self.example == 1:
            return 1.0 + 0.0 * x
        wet_window = 0.5 * (backend.tanh((x - 45.0) / 4.0)
                            - backend.tanh((x - 100.0) / 4.0))
        return 1.0 - 0.40 * wet_window

    def height(self, x: Any, y: Any) -> Any:
        return 0.0 * x


class VehicleModel:
    """Modified source equations with a differentiable CasADi representation.

    Spatial state z=[Y,psi,vx,vy,r,delta] (example1), plus [ax_load,ay_load]
    (example2). Time-domain Xdot is evaluated separately; spatial derivative
    is dz/dX=(dz/dt)/Xdot. Input u=[steering_rate, common_wheel_slip_ratio].
    Example2 is a PLANAR reduction of double-track, NOT its 28-state plant.
    """
    def __init__(self, p: CarParams, road: Road):
        self.p, self.road = p, road
        self.nx = 6 if road.example == 1 else 8
        self.names = ["Y_m", "psi_rad", "vx_mps", "vy_mps", "r_radps", "delta_rad"]
        self.scale = np.array([5.0, 0.3, 20.0, 2.0, 0.5, 0.3])
        if self.nx == 8:
            self.names += ["load_ax_mps2", "load_ay_mps2"]
            self.scale = np.r_[self.scale, 4.0, 4.0]
        self.u_scale = np.array([p.steer_rate_max, 0.05])
        self.alpha_max = 0.065 if road.example == 1 else 0.16
        self.kappa_max = 0.07 if road.example == 1 else 0.12
        self.util_max = 0.95 if road.example == 1 else 0.985
        self._build()

    def _tire(self, kappa: Any, alpha: Any, fz: Any, mu: Any) -> tuple[Any, Any, Any]:
        p = self.p
        if self.road.example == 1:
            return mu*fz*p.c_kappa*kappa, mu*fz*p.c_alpha*alpha, mu*fz
        # Adapted from mf_simple_extended.cpp; this regularization is a CHANGE.
        lat = ca.tan(alpha)
        slip = ca.sqrt(kappa*kappa + lat*lat + p.slip_epsilon**2)
        nominal = p.mass*p.gravity/4.0
        d = p.d0 + p.pdy2*(fz - nominal)/nominal
        def mf(b: float, c: float, e: float) -> Any:
            bs = b*slip
            return ca.sin(c*ca.atan(bs-e*(bs-ca.atan(bs))))
        peak = mu*fz*d
        return peak*mf(p.bx,p.cx,p.ex)*kappa/slip, peak*mf(p.by,p.cy,p.ey)*lat/slip, peak

    def _build(self) -> None:
        p = self.p
        x = ca.SX.sym("X")
        z = ca.SX.sym("z", self.nx)
        u = ca.SX.sym("u", 2)
        y, psi, vx, vy, r, delta = [z[j] for j in range(6)]
        nu, kappa = u[0], u[1]
        front = p.mass*p.gravity*p.lr/p.wheelbase
        rear = p.mass*p.gravity*p.lf/p.wheelbase
        b = 0.0 if self.road.example == 1 else p.track
        fzs = [front/2, front/2, rear/2, rear/2]
        if self.nx == 8:
            # NEW reduction of roll/pitch/suspension, not upstream exact physics.
            ax_load, ay_load = z[6], z[7]
            front = front - p.mass*p.cg_height*ax_load/p.wheelbase
            rear = rear + p.mass*p.cg_height*ax_load/p.wheelbase
            transfer_f = p.mass*p.cg_height*(p.lr/p.wheelbase)*ay_load/p.track
            transfer_r = p.mass*p.cg_height*(p.lf/p.wheelbase)*ay_load/p.track
            fzs = [front/2-transfer_f, front/2+transfer_f,
                   rear/2-transfer_r, rear/2+transfer_r]
        fxsum, fysum, moment = 0, 0, 0
        alphas, forces_x, forces_y, peaks, mus, tire_u = [], [], [], [], [], []
        for i, (xi, yi) in enumerate([(p.lf,b/2),(p.lf,-b/2),(-p.lr,b/2),(-p.lr,-b/2)]):
            di = delta if i < 2 else 0.0
            ub, vb = vx-r*yi, vy+r*xi
            ut = ca.cos(di)*ub+ca.sin(di)*vb
            vt = -ca.sin(di)*ub+ca.cos(di)*vb
            # Exact upstream branch for forward speeds; feasibility requires ut>=3.
            alpha = -ca.atan2(vt, ca.fmax(1.0, ut))
            wheel_x = x + ca.cos(psi)*xi-ca.sin(psi)*yi
            wheel_y = y + ca.sin(psi)*xi+ca.cos(psi)*yi
            mu = self.road.friction(wheel_x,wheel_y,ca)
            fx, fy, peak = self._tire(kappa,alpha,fzs[i],mu)
            fx_b = ca.cos(di)*fx-ca.sin(di)*fy
            fy_b = ca.sin(di)*fx+ca.cos(di)*fy
            fxsum += fx_b
            fysum += fy_b
            moment += xi*fy_b-yi*fx_b
            alphas.append(alpha); forces_x.append(fx); forces_y.append(fy)
            peaks.append(peak); mus.append(mu); tire_u.append(ut)
        # No downforce/wind; explicitly simplified aero. Rolling loss moved from
        # native wheel-load torque into the body equation after removing wheels.
        drag = 0.5*p.rho_air*p.cd_area*vx*ca.sqrt(vx*vx+1e-8)
        resistance = p.rolling_coefficient*p.mass*p.gravity
        total_fx = fxsum-drag-resistance
        total_fy = fysum
        ax, ay = total_fx/p.mass, total_fy/p.mass
        progress = vx*ca.cos(psi)-vy*ca.sin(psi)
        ydot = vx*ca.sin(psi)+vy*ca.cos(psi)
        timed = ca.vertcat(ydot,r,ax+r*vy,ay-r*vx,moment/p.iz,nu)
        if self.nx == 8:
            timed = ca.vertcat(timed,(ax-z[6])/p.load_tau,(ay-z[7])/p.load_tau)
        # Numerical extension outside feasible domain. progress>=4 is enforced.
        denom = ca.fmax(1.0,progress)
        self.time_rhs = ca.Function("time_rhs",[x,z,u],[timed,progress])
        self.spatial = ca.Function("spatial",[x,z,u],[timed/denom,1/denom])
        alpha_vec = ca.vertcat(*alphas)
        fxv, fyv, pk = ca.vertcat(*forces_x), ca.vertcat(*forces_y), ca.vertcat(*peaks)
        util2 = (fxv**2+fyv**2)/(pk**2+1e-10)
        room = self.road.half_width-p.enclosing_radius*math.sqrt(1+self.road.slope_bound**2)
        error = y-self.road.center(x,ca)
        beta = ca.atan2(vy, vx)
        nominal_power_proxy = sum(forces_x)*vx  # deliberate reduced-powertrain proxy
        cons = ca.vertcat(
            (room-p.safety_margin-error)/self.road.half_width,
            (room-p.safety_margin+error)/self.road.half_width,
            (vx-p.speed_min)/20, (p.speed_max-vx)/20,
            (progress-4.0)/20,
            (p.beta_max-beta), (p.beta_max+beta),
            (0.60-psi), (0.60+psi),
            (0.90-r), (0.90+r),
            (p.steer_max-delta), (p.steer_max+delta),
            (p.acceleration_max-ax)/10, (p.braking_max+ax)/10,
            (p.power_max-nominal_power_proxy)/p.power_max,
            self.alpha_max-alpha_vec,self.alpha_max+alpha_vec,
            self.util_max**2-util2,
            (ca.vertcat(*fzs)-p.min_wheel_load)/2000,
            (ca.vertcat(*tire_u)-3.0)/20,
        )
        self.constraints = ca.Function("constraints",[x,z,u],[cons])
        self.diagnostics = ca.Function("diagnostics",[x,z,u],[
            ca.vertcat(progress,ax,ay,beta,error,room-ca.fabs(error),
                       alpha_vec,fxv,fyv,ca.vertcat(*fzs),ca.vertcat(*mus),ca.sqrt(util2),nominal_power_proxy)])
        self.tire_test = ca.Function("tire_test",[ca.SX.sym("dummy")],[0])
        kk, aa, zz, mm = ca.SX.sym("kap"),ca.SX.sym("alpha"),ca.SX.sym("fz"),ca.SX.sym("mu")
        tx,ty,tp = self._tire(kk,aa,zz,mm)
        self.tire = ca.Function("tire",[kk,aa,zz,mm],[ca.vertcat(tx,ty,tp)])
        self.jacobian = ca.Function("jacobian",[x,z,u],[ca.jacobian(timed,z)])

    def initial_state(self, speed: float = 8.0) -> np.ndarray:
        z = np.zeros(self.nx)
        z[0] = self.road.center(0.0)
        z[1] = math.atan(self.road.slope(0.0))
        z[2] = speed
        return z

    def make_step(self, ds: float, subdivisions: int) -> ca.Function:
        """RK4 multiple-shooting transition, with integrated time and guards."""
        x = ca.MX.sym("X")
        z = ca.MX.sym("z",self.nx)
        u = ca.MX.sym("u",2)
        zz = z
        dt = 0
        checks = [self.constraints(x,zz,u)-self.p.constraint_backoff]
        h = ds/subdivisions
        for j in range(subdivisions):
            xx = x+j*h
            k1,t1 = self.spatial(xx,zz,u)
            k2,t2 = self.spatial(xx+h/2,zz+h*k1/2,u)
            k3,t3 = self.spatial(xx+h/2,zz+h*k2/2,u)
            k4,t4 = self.spatial(xx+h,zz+h*k3,u)
            zz = zz+h*(k1+2*k2+2*k3+k4)/6
            dt += h*(t1+2*t2+2*t3+t4)/6
            checks.append(self.constraints(xx+h,zz,u)-self.p.constraint_backoff)
        return ca.Function("rk4_step",[x,z,u],[zz,dt,ca.vertcat(*checks)])


class MinimumTimeMPC:
    """Shrinking, full-remaining-course, spatial MPC; local NLP via IPOPT."""
    def __init__(self, model: VehicleModel, cells: int, subdivisions: int):
        self.model = model
        self.cells = cells
        self.ds = model.road.length/cells
        self.step = model.make_step(self.ds,subdivisions)
        self.previous: tuple[np.ndarray,np.ndarray] | None = None

    def guess(self, current_x: float, n: int, actual: np.ndarray) -> tuple[np.ndarray,np.ndarray]:
        m = self.model
        if self.previous is not None:
            oldz,oldu = self.previous
            z = oldz[:,1:].copy()
            u = oldu[:,1:].copy()
            if z.shape == (m.nx,n+1) and u.shape == (2,n):
                z[:,0] = actual
                return z,u
        xgrid = current_x+self.ds*np.arange(n+1)
        speed = max(8.0,actual[2])
        z = np.zeros((m.nx,n+1))
        z[0] = m.road.center(xgrid)
        heading = np.arctan(m.road.slope(xgrid))
        curvature = np.gradient(heading,self.ds)
        z[2] = speed
        z[4] = speed*curvature
        alpha_rear = speed**2*curvature/(m.p.c_alpha*m.p.gravity)
        z[3] = m.p.lr*z[4]-speed*alpha_rear
        z[1] = heading-np.arctan2(z[3],z[2])
        z[5] = m.p.wheelbase*curvature
        if m.nx == 8:
            z[7] = speed**2*curvature
        z[:,0] = actual
        u = np.zeros((2,n))
        u[0] = np.clip(np.diff(z[5])*speed/self.ds,-.3,.3)
        u[1] = 0.002
        return z,u

    def solve(self,current_x: float,n: int,actual: np.ndarray) -> tuple[np.ndarray,dict[str,Any]]:
        m = self.model
        begin = time.perf_counter()
        zs = ca.MX.sym("zs",m.nx,n+1)
        us = ca.MX.sym("us",2,n)
        z0 = ca.MX.sym("z0",m.nx)
        z = ca.repmat(ca.DM(m.scale),1,n+1)*zs
        u = ca.repmat(ca.DM(m.u_scale),1,n)*us
        eq = [(z[:,0]-z0)/ca.DM(m.scale)]
        ineq = []
        objective = 0
        for k in range(n):
            nxt,dt,ck = self.step(current_x+k*self.ds,z[:,k],u[:,k])
            eq.append((z[:,k+1]-nxt)/ca.DM(m.scale))
            ineq.append(ck)
            objective += dt
        gg = ca.vertcat(*eq,*ineq)
        neq = m.nx*(n+1)
        ng = int(gg.numel())
        w = ca.vertcat(ca.vec(zs),ca.vec(us))
        nlp = {"x":w,"p":z0,"f":objective,"g":gg}
        options = {"print_time":False,"ipopt.print_level":0,"ipopt.sb":"yes",
                   "ipopt.max_iter":600,"ipopt.tol":2e-8,"ipopt.acceptable_tol":2e-7,
                   "ipopt.bound_relax_factor":0.0}
        solver = ca.nlpsol("minimum_time", "ipopt", nlp, options)
        lo_z = np.array([-20,-.60,m.p.speed_min,-5,-.90,-m.p.steer_max]+([-8,-10] if m.nx==8 else []))/m.scale
        hi_z = np.array([20,.60,m.p.speed_max,5,.90,m.p.steer_max]+([8,10] if m.nx==8 else []))/m.scale
        lbx = np.r_[np.tile(lo_z,n+1),np.tile([-1,-m.kappa_max/m.u_scale[1]],n)]
        ubx = np.r_[np.tile(hi_z,n+1),np.tile([1,m.kappa_max/m.u_scale[1]],n)]
        zg,ug = self.guess(current_x,n,actual)
        initial = np.r_[(zg/m.scale[:,None]).ravel(order="F"),(ug/m.u_scale[:,None]).ravel(order="F")]
        lbg=np.zeros(ng);ubg=np.r_[np.zeros(neq),np.full(ng-neq,np.inf)]
        result = solver(x0=initial,p=actual,lbx=lbx,ubx=ubx,lbg=lbg,ubg=ubg)
        stats = solver.stats()
        sol = np.asarray(result["x"]).reshape(-1)
        gv = np.asarray(result["g"]).reshape(-1)
        violation = max(float(np.max(np.abs(gv[:neq]))),float(max(0,-np.min(gv[neq:]))),
                        float(max(0,np.max(lbx-sol))),float(max(0,np.max(sol-ubx))))
        if not stats["success"] or violation > 2e-5:
            raise RuntimeError(f"NLP failed at X={current_x:.3f}: {stats['return_status']}; violation={violation:.3g}. No fallback control was applied.")
        cut = m.nx*(n+1)
        zz = sol[:cut].reshape((m.nx,n+1),order="F")*m.scale[:,None]
        uu = sol[cut:].reshape((2,n),order="F")*m.u_scale[:,None]
        self.previous=(zz,uu)
        return uu[:,0], {"X_m":current_x,"horizon_intervals":n,"objective_remaining_time_s":float(result["f"]),
                         "iterations":int(stats["iter_count"]),"return_status":stats["return_status"],
                         "max_scaled_constraint_violation":violation,
                         "build_and_solve_wall_s":time.perf_counter()-begin}


def self_test(model: VehicleModel) -> dict[str,Any]:
    """Executable checks; no downloaded repository or binary is required."""
    m,p = model,model.p
    z = m.initial_state()
    fz = p.mass*p.gravity/4
    a = np.asarray(m.tire(0,0,fz,1)).ravel()
    assert np.all(np.isfinite(a)) and np.max(np.abs(a[:2]))<1e-10
    a = np.asarray(m.tire(.01,.03,fz,1)).ravel()
    b = np.asarray(m.tire(-.01,-.03,fz,1)).ravel()
    assert np.allclose(a[:2],-b[:2],rtol=1e-10,atol=1e-10)
    wet = np.asarray(m.tire(.01,.03,fz,.6)).ravel()
    assert np.allclose(wet[:2],.6*a[:2])
    z[5]=.02
    deriv,_=m.time_rhs(0,z,np.array([0.,0.]))
    assert float(deriv[4])>0, "Positive steering must initially generate positive yaw acceleration."
    z[5]=0; z[3]=.1
    deriv,_=m.time_rhs(0,z,np.array([0.,0.]))
    assert float(deriv[3])<0, "Lateral drift must generate a restoring acceleration at zero yaw rate."
    z=m.initial_state()
    diag=np.asarray(m.diagnostics(0,z,np.array([0.,.002]))).ravel()
    assert abs(sum(diag[18:22])-p.mass*p.gravity)<1e-8
    if m.nx==8:
        z[6:8]=[2.,3.]
        diag=np.asarray(m.diagnostics(0,z,np.array([0.,.002]))).ravel()
        assert abs(sum(diag[18:22])-p.mass*p.gravity)<1e-8
        assert diag[18]<diag[19] and diag[20]<diag[21]
    # Check CasADi derivative against central differences at a smooth point.
    z=m.initial_state(); z[3]=.1; z[4]=.01; z[5]=.02;u=np.array([.01,.01])
    jac=np.asarray(m.jacobian(10,z,u))
    num=np.empty_like(jac)
    for k in range(m.nx):
        h=1e-6*max(1,abs(z[k]));zp=z.copy();zm=z.copy();zp[k]+=h;zm[k]-=h
        fp=np.asarray(m.time_rhs(10,zp,u)[0]).ravel();fm=np.asarray(m.time_rhs(10,zm,u)[0]).ravel()
        num[:,k]=(fp-fm)/(2*h)
    err=float(np.max(np.abs(jac-num)))
    assert np.allclose(jac,num,rtol=3e-5,atol=2e-5),err
    # Geometric containment: test a full circumscribed disc at its safe boundary.
    xg=np.linspace(0,m.road.length,51)
    theta=np.linspace(0,2*np.pi,361)
    radius=p.enclosing_radius
    safe=m.road.half_width-radius*math.sqrt(1+m.road.slope_bound**2)
    for x0 in xg:
        for side in [-1.,1.]:
            y0=float(m.road.center(x0))+side*safe
            xx=x0+radius*np.cos(theta);yy=y0+radius*np.sin(theta)
            assert np.max(np.abs(yy-m.road.center(xx))) <= m.road.half_width+1e-10
    return {"passed":True,"checks":["zero slip","odd tire force","friction scaling","steering yaw sign",
            "restoring lateral force","normal-load conservation","AD versus finite differences",
            "conservative footprint containment"],"jacobian_max_absolute_error":err}


def simulate(args: argparse.Namespace) -> dict[str,Any]:
    road=Road()
    p=CarParams(speed_max=args.speed_cap)
    m=VehicleModel(p,road)
    test=self_test(m)
    cells=args.cells or (28 if EXAMPLE==1 else 36)
    if cells<8:
        raise ValueError("Use at least 8 cells; coarse meshes need extra refinement checks.")
    mpc=MinimumTimeMPC(m,cells,args.substeps)
    state=m.initial_state(args.initial_speed)
    records=[]; solver_records=[];total_time=0.
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    last_control=np.zeros(2)
    # A tiny tightening is already imposed in the optimizer's road constraints.
    # Other dense violations above tolerance cause an explicit failure, never a success label.
    dense_min=math.inf
    for k in range(cells):
        x0=k*mpc.ds;x1=(k+1)*mpc.ds
        if args.disturbance and k==cells//3:
            state[3]+=args.disturbance
        control,stat=mpc.solve(x0,cells-k,state)
        solver_records.append(stat)
        def fun(x:float,zt:np.ndarray)->np.ndarray:
            dz,dt=m.spatial(x,zt[:-1],control)
            return np.r_[np.asarray(dz).ravel(),float(dt)]
        sol=solve_ivp(fun,(x0,x1),np.r_[state,0.0],method="DOP853",rtol=2e-10,atol=2e-11,
                      dense_output=True,max_step=mpc.ds/16)
        if not sol.success:
            raise RuntimeError(f"Plant integration failed: {sol.message}")
        sample_x=np.linspace(x0,x1,41)
        sample=sol.sol(sample_x)
        for j,xx in enumerate(sample_x):
            zj=sample[:-1,j]
            cc=np.asarray(m.constraints(xx,zj,control)).ravel()
            dense_min=min(dense_min,float(np.min(cc)))
            dd=np.asarray(m.diagnostics(xx,zj,control)).ravel()
            row={"time_s":total_time+float(sample[-1,j]),"X_m":float(xx)}
            row.update(dict(zip(m.names,map(float,zj))))
            row.update({"steering_rate_radps":float(control[0]),"kappa_command":float(control[1]),
                        "progress_mps":dd[0],"ax_mps2":dd[1],"ay_mps2":dd[2],"beta_rad":dd[3],
                        "center_error_m":dd[4],"footprint_bound_clearance_m":dd[5],
                        "max_tire_utilisation":float(np.max(dd[26:30])),
                        "min_mu":float(np.min(dd[22:26])),"min_wheel_load_N":float(np.min(dd[18:22])),
                        "nominal_power_proxy_W":float(dd[30]),"max_abs_tire_slip_rad":float(np.max(np.abs(dd[6:10])))})
            records.append(row)
        total_time+=float(sol.y[-1,-1]);state=sol.y[:-1,-1];last_control=control
        if k%5==0 or k==cells-1:
            print(f"MPC {k+1:3d}/{cells}: X={x1:7.2f} m, elapsed={total_time:7.3f} s, "
                  f"vx={state[2]:6.2f} m/s, solver={stat['return_status']}",flush=True)
    # Dense integration is deliberately independent of the RK4 prediction grid.
    complete=bool(abs(records[-1]["X_m"]-road.length)<1e-8)
    worst_violation=max(0.,-dense_min)
    feasible=bool(worst_violation<=args.validation_tolerance)
    max_nlp=max(r["max_scaled_constraint_violation"] for r in solver_records)
    metrics={
        "example":EXAMPLE,"source_commit":COMMIT,"native_upstream_compiled":False,
        "model":"modified single-track + linear tires" if EXAMPLE==1 else "planar double-track reduction + smooth combined-slip tires + filtered load transfer",
        "objective":"minimum remaining traversal time, no tracking/progress surrogate",
        "mpc":"full-remaining-road shrinking spatial horizon; one first control interval applied",
        "solver":"CasADi IPOPT local nonlinear optimization","global_optimality_certified":False,
        "reached_finish":complete,"dense_validation_passed":feasible,
        "road_length_X_m":road.length,"actual_traversal_time_s":total_time,
        "initial_predicted_time_s":solver_records[0]["objective_remaining_time_s"],
        "mpc_solves":len(solver_records),"all_solver_calls_successful":True,
        "max_NLP_scaled_violation":max_nlp,"max_dense_scaled_violation":worst_violation,
        "validation_tolerance":args.validation_tolerance,
        "min_conservative_footprint_clearance_m":min(r["footprint_bound_clearance_m"] for r in records),
        "max_speed_mps":max(r["vx_mps"] for r in records),
        "max_ax_mps2":max(r["ax_mps2"] for r in records),
        "min_ax_mps2":min(r["ax_mps2"] for r in records),
        "max_nominal_power_proxy_W":max(r["nominal_power_proxy_W"] for r in records),
        "max_abs_tire_slip_rad":max(r["max_abs_tire_slip_rad"] for r in records),
        "max_abs_beta_rad":max(abs(r["beta_rad"]) for r in records),
        "max_tire_utilisation":max(r["max_tire_utilisation"] for r in records),
        "minimum_wheel_load_N":min(r["min_wheel_load_N"] for r in records),
        "minimum_friction_scale":min(r["min_mu"] for r in records),
        "total_NLP_build_and_solve_wall_s":sum(r["build_and_solve_wall_s"] for r in solver_records),
        "cells":cells,"RK4_substeps_per_cell":args.substeps,"disturbance_vy_mps":args.disturbance,
        "versions":{"python":sys.version.split()[0],"numpy":np.__version__,"casadi":ca.__version__},
        "parameters":asdict(p),"unit_tests":test,
        "limitations":["local optimality only","reduced plant; no native C++ parity validation",
                       "spatial/event-based sampling, not a fixed-time real-time controller",
                       "sampled feasibility checks are not a continuous-time or robustness proof",
                       "ideal slip actuator; no wheel/drivetrain dynamics",
                       "exit-speed free; no stopping-at-finish requirement"],
    }
    with (out/"trajectory.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
    (out/"solver_log.json").write_text(json.dumps(solver_records,indent=2))
    (out/"metrics.json").write_text(json.dumps(metrics,indent=2))
    (out/"source_map.json").write_text(json.dumps({"source":SOURCE_MAP,"books":BOOK_MAP},indent=2))
    if not args.no_plots:
        plot_results(records,road,p,out,args.show)
    print(json.dumps({k:metrics[k] for k in ["reached_finish","dense_validation_passed","actual_traversal_time_s",
                                           "max_dense_scaled_violation","min_conservative_footprint_clearance_m"]},indent=2))
    if not (complete and feasible):
        raise RuntimeError("Validation failed. Logs saved; this run must NOT be reported as a successful feasible maneuver.")
    return metrics


def plot_results(records:list[dict[str,Any]],road:Road,p:CarParams,out:Path,show:bool)->None:
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    # Each chart is its own figure; default matplotlib colours are used.
    x=np.array([r["X_m"] for r in records]);y=np.array([r["Y_m"] for r in records])
    grid=np.linspace(0,road.length,800)
    fig,ax=plt.subplots(figsize=(11,4))
    ax.plot(grid,road.center(grid)-road.half_width,label="lower road edge")
    ax.plot(grid,road.center(grid)+road.half_width,label="upper road edge")
    ax.plot(grid,road.center(grid),linestyle="--",label="road center (NOT a tracking objective)")
    ax.plot(x,y,label="MPC trajectory")
    ax.set(xlabel="X [m]",ylabel="Y [m]",title=f"Example {EXAMPLE}: minimum-time MPC path")
    ax.legend();ax.grid(True);fig.tight_layout();fig.savefig(out/"path.png",dpi=170)
    fig,ax=plt.subplots(figsize=(10,4))
    ax.plot(x,[r["vx_mps"] for r in records],label="body longitudinal speed")
    ax.set(xlabel="X [m]",ylabel="speed [m/s]",title="Optimized speed, not a prescribed speed profile")
    ax.grid(True);fig.tight_layout();fig.savefig(out/"speed.png",dpi=170)
    fig,ax=plt.subplots(figsize=(10,4))
    ax.plot(x,[r["min_mu"] for r in records],label="minimum friction scale at tires")
    ax.plot(x,[r["max_tire_utilisation"] for r in records],label="maximum tire-force utilisation")
    ax.set(xlabel="X [m]",ylabel="dimensionless",title="Road friction and tire utilisation")
    ax.legend();ax.grid(True);fig.tight_layout();fig.savefig(out/"tire_utilisation.png",dpi=170)
    if show:plt.show()
    plt.close("all")


def main()->None:
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output",default=f"example_{EXAMPLE}_results")
    parser.add_argument("--cells",type=int,default=0,help="Number of spatial cells on the full road (default 28/36).")
    parser.add_argument("--substeps",type=int,default=8,help="RK4 substeps per cell (default 8).")
    parser.add_argument("--speed-cap",type=float,default=30.0)
    parser.add_argument("--initial-speed",type=float,default=8.0)
    parser.add_argument("--disturbance",type=float,default=0.0,help="Optional vy impulse at one third of the road [m/s].")
    parser.add_argument("--validation-tolerance",type=float,default=3e-4,help="Maximum accepted normalized dense constraint violation.")
    parser.add_argument("--self-test",action="store_true")
    parser.add_argument("--show",action="store_true")
    parser.add_argument("--no-plots",action="store_true")
    args=parser.parse_args()
    if args.substeps<2 or not 6<=args.initial_speed<=args.speed_cap:
        parser.error("Require substeps>=2 and 6<=initial-speed<=speed-cap.")
    if args.self_test:
        print(json.dumps(self_test(VehicleModel(CarParams(speed_max=args.speed_cap),Road())),indent=2))
        return
    try:
        simulate(args)
    except (RuntimeError,ValueError) as exc:
        print(f"ERROR: {exc}",file=sys.stderr)
        raise SystemExit(1) from exc


# Full licence retained so this demo may be redistributed as one file.
__license_text__ = r"""

                                 Apache License
                           Version 2.0, January 2004
                        http://www.apache.org/licenses/

   TERMS AND CONDITIONS FOR USE, REPRODUCTION, AND DISTRIBUTION

   1. Definitions.

      "License" shall mean the terms and conditions for use, reproduction,
      and distribution as defined by Sections 1 through 9 of this document.

      "Licensor" shall mean the copyright owner or entity authorized by
      the copyright owner that is granting the License.

      "Legal Entity" shall mean the union of the acting entity and all
      other entities that control, are controlled by, or are under common
      control with that entity. For the purposes of this definition,
      "control" means (i) the power, direct or indirect, to cause the
      direction or management of such entity, whether by contract or
      otherwise, or (ii) ownership of fifty percent (50%) or more of the
      outstanding shares, or (iii) beneficial ownership of such entity.

      "You" (or "Your") shall mean an individual or Legal Entity
      exercising permissions granted by this License.

      "Source" form shall mean the preferred form for making modifications,
      including but not limited to software source code, documentation
      source, and configuration files.

      "Object" form shall mean any form resulting from mechanical
      transformation or translation of a Source form, including but
      not limited to compiled object code, generated documentation,
      and conversions to other media types.

      "Work" shall mean the work of authorship, whether in Source or
      Object form, made available under the License, as indicated by a
      copyright notice that is included in or attached to the work
      (an example is provided in the Appendix below).

      "Derivative Works" shall mean any work, whether in Source or Object
      form, that is based on (or derived from) the Work and for which the
      editorial revisions, annotations, elaborations, or other modifications
      represent, as a whole, an original work of authorship. For the purposes
      of this License, Derivative Works shall not include works that remain
      separable from, or merely link (or bind by name) to the interfaces of,
      the Work and Derivative Works thereof.

      "Contribution" shall mean any work of authorship, including
      the original version of the Work and any modifications or additions
      to that Work or Derivative Works thereof, that is intentionally
      submitted to Licensor for inclusion in the Work by the copyright owner
      or by an individual or Legal Entity authorized to submit on behalf of
      the copyright owner. For the purposes of this definition, "submitted"
      means any form of electronic, verbal, or written communication sent
      to the Licensor or its representatives, including but not limited to
      communication on electronic mailing lists, source code control systems,
      and issue tracking systems that are managed by, or on behalf of, the
      Licensor for the purpose of discussing and improving the Work, but
      excluding communication that is conspicuously marked or otherwise
      designated in writing by the copyright owner as "Not a Contribution."

      "Contributor" shall mean Licensor and any individual or Legal Entity
      on behalf of whom a Contribution has been received by Licensor and
      subsequently incorporated within the Work.

   2. Grant of Copyright License. Subject to the terms and conditions of
      this License, each Contributor hereby grants to You a perpetual,
      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
      copyright license to reproduce, prepare Derivative Works of,
      publicly display, publicly perform, sublicense, and distribute the
      Work and such Derivative Works in Source or Object form.

   3. Grant of Patent License. Subject to the terms and conditions of
      this License, each Contributor hereby grants to You a perpetual,
      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
      (except as stated in this section) patent license to make, have made,
      use, offer to sell, sell, import, and otherwise transfer the Work,
      where such license applies only to those patent claims licensable
      by such Contributor that are necessarily infringed by their
      Contribution(s) alone or by combination of their Contribution(s)
      with the Work to which such Contribution(s) was submitted. If You
      institute patent litigation against any entity (including a
      cross-claim or counterclaim in a lawsuit) alleging that the Work
      or a Contribution incorporated within the Work constitutes direct
      or contributory patent infringement, then any patent licenses
      granted to You under this License for that Work shall terminate
      as of the date such litigation is filed.

   4. Redistribution. You may reproduce and distribute copies of the
      Work or Derivative Works thereof in any medium, with or without
      modifications, and in Source or Object form, provided that You
      meet the following conditions:

      (a) You must give any other recipients of the Work or
          Derivative Works a copy of this License; and

      (b) You must cause any modified files to carry prominent notices
          stating that You changed the files; and

      (c) You must retain, in the Source form of any Derivative Works
          that You distribute, all copyright, patent, trademark, and
          attribution notices from the Source form of the Work,
          excluding those notices that do not pertain to any part of
          the Derivative Works; and

      (d) If the Work includes a "NOTICE" text file as part of its
          distribution, then any Derivative Works that You distribute must
          include a readable copy of the attribution notices contained
          within such NOTICE file, excluding those notices that do not
          pertain to any part of the Derivative Works, in at least one
          of the following places: within a NOTICE text file distributed
          as part of the Derivative Works; within the Source form or
          documentation, if provided along with the Derivative Works; or,
          within a display generated by the Derivative Works, if and
          wherever such third-party notices normally appear. The contents
          of the NOTICE file are for informational purposes only and
          do not modify the License. You may add Your own attribution
          notices within Derivative Works that You distribute, alongside
          or as an addendum to the NOTICE text from the Work, provided
          that such additional attribution notices cannot be construed
          as modifying the License.

      You may add Your own copyright statement to Your modifications and
      may provide additional or different license terms and conditions
      for use, reproduction, or distribution of Your modifications, or
      for any such Derivative Works as a whole, provided Your use,
      reproduction, and distribution of the Work otherwise complies with
      the conditions stated in this License.

   5. Submission of Contributions. Unless You explicitly state otherwise,
      any Contribution intentionally submitted for inclusion in the Work
      by You to the Licensor shall be under the terms and conditions of
      this License, without any additional terms or conditions.
      Notwithstanding the above, nothing herein shall supersede or modify
      the terms of any separate license agreement you may have executed
      with Licensor regarding such Contributions.

   6. Trademarks. This License does not grant permission to use the trade
      names, trademarks, service marks, or product names of the Licensor,
      except as required for reasonable and customary use in describing the
      origin of the Work and reproducing the content of the NOTICE file.

   7. Disclaimer of Warranty. Unless required by applicable law or
      agreed to in writing, Licensor provides the Work (and each
      Contributor provides its Contributions) on an "AS IS" BASIS,
      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
      implied, including, without limitation, any warranties or conditions
      of TITLE, NON-INFRINGEMENT, MERCHANTABILITY, or FITNESS FOR A
      PARTICULAR PURPOSE. You are solely responsible for determining the
      appropriateness of using or redistributing the Work and assume any
      risks associated with Your exercise of permissions under this License.

   8. Limitation of Liability. In no event and under no legal theory,
      whether in tort (including negligence), contract, or otherwise,
      unless required by applicable law (such as deliberate and grossly
      negligent acts) or agreed to in writing, shall any Contributor be
      liable to You for damages, including any direct, indirect, special,
      incidental, or consequential damages of any character arising as a
      result of this License or out of the use or inability to use the
      Work (including but not limited to damages for loss of goodwill,
      work stoppage, computer failure or malfunction, or any and all
      other commercial damages or losses), even if such Contributor
      has been advised of the possibility of such damages.

   9. Accepting Warranty or Additional Liability. While redistributing
      the Work or Derivative Works thereof, You may choose to offer,
      and charge a fee for, acceptance of support, warranty, indemnity,
      or other liability obligations and/or rights consistent with this
      License. However, in accepting such obligations, You may act only
      on Your own behalf and on Your sole responsibility, not on behalf
      of any other Contributor, and only if You agree to indemnify,
      defend, and hold each Contributor harmless for any liability
      incurred by, or claims asserted against, such Contributor by reason
      of your accepting any such warranty or additional liability.

   END OF TERMS AND CONDITIONS

   APPENDIX: How to apply the Apache License to your work.

      To apply the Apache License to your work, attach the following
      boilerplate notice, with the fields enclosed by brackets "[]"
      replaced with your own identifying information. (Don't include
      the brackets!)  The text should be enclosed in the appropriate
      comment syntax for the file format. We also recommend that a
      file or class name and description of purpose be included on the
      same "printed page" as the copyright notice for easier
      identification within third-party archives.

   Copyright [yyyy] [name of copyright owner]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
"""

if __name__=="__main__":
    main()
