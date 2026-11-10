"""Normalized planar four-wheel plants for a structural model comparison.

These are newly implemented comparison models, not reproductions of a thesis's
calibrated vehicle or controller.  The 11 states are X,Y,psi,vx,vy,r, four wheel
speeds, and front steering.  They omit OCD's suspension, roll/pitch and tire
relaxation states.  Default geometry, drivetrain and pure-slip Magic Formula
shape parameters follow TUM Open-Car-Dynamics revision
94f8fb187fb0ed22bba1d809bd74f66d1ff75af4; peak friction is explicitly normalized
from that source's D=1.7 to D=1.0.

Tire alternatives:
* mf_independent: the upstream MF_SIMPLE algebraic force laws, without its
  transient slip states.  It deliberately does NOT enforce a friction circle.
* mf_ellipse: the same pure-slip forces with a longitudinal-priority friction
  ellipse, as in Berntorp (2014), printed p.70, equation (4.9).
* fiala: classic saturated Fiala lateral force with remaining circle capacity,
  motivated by Bertipaglia (2025), ch.4, equations (4.4)/(4.8).  This is NOT his
  extended post-peak law.  The longitudinal force remains the same MF law, and
  its small-slip lateral stiffness is normalized to OCD at the reference load.

Sources:
https://github.com/TUMFTM/Open-Car-Dynamics/tree/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4
https://lucris.lub.lu.se/ws/files/3352295/4407278.pdf
https://research.tudelft.nl/files/235083711/Bertipaglia_2024_PhD_Dissertation.pdf

Coordinates: x forward, y left, yaw positive counterclockwise.  Wheel order is
FL,FR,RL,RR.  alpha=-atan2(v_tire_y,max(1,v_tire_x)), so positive alpha produces
positive Fy.  This is the opposite slip-angle sign from the Fiala thesis.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Sequence

import numpy as np

SOURCE_COMMIT = "94f8fb187fb0ed22bba1d809bd74f66d1ff75af4"
STATE_NAMES = ("x_m", "y_m", "yaw_rad", "vx_mps", "vy_mps", "yaw_rate_radps",
               "omega_fl_radps", "omega_fr_radps", "omega_rl_radps", "omega_rr_radps",
               "steering_rad")
WHEELS = ("FL", "FR", "RL", "RR")


@dataclass(frozen=True)
class VehicleParameters:
    """Shared demonstration parameters, not a fitted thesis vehicle dataset."""
    mass_kg: float = 800.0
    yaw_inertia_kgm2: float = 1000.0
    wheelbase_m: float = 3.2
    lf_m: float = 1.724
    front_track_m: float = 1.6
    rear_track_m: float = 1.5
    cg_height_m: float = 0.3
    wheel_radius_m: float = 0.3
    wheel_inertia_front_kgm2: float = 1.15
    wheel_inertia_rear_kgm2: float = 1.35
    rolling_resistance_coefficient: float = 0.025
    air_density_kgpm3: float = 1.225
    drag_coefficient_area_m2: float = 1.0
    lift_coefficient_area_m2: float = -1.5
    aero_center_offset_m: float = 0.0
    steering_limit_rad: float = 0.5
    steering_rate_limit_radps: float = 0.5
    steering_time_constant_s: float = 0.06
    gravity_mps2: float = 9.81
    nominal_friction: float = 1.0
    longitudinal_B: float = 20.0
    longitudinal_C: float = 2.0
    longitudinal_E: float = 0.7
    lateral_B: float = 10.0
    lateral_C: float = 1.6
    lateral_E: float = -1.9
    fiala_load_shape_c2: float = 3.5
    body_length_m: float = 4.4
    body_width_m: float = 1.9

    @property
    def lr_m(self) -> float:
        return self.wheelbase_m - self.lf_m

    @property
    def front_weight_fraction(self) -> float:
        return self.lr_m / self.wheelbase_m

    def validate(self) -> None:
        values = asdict(self)
        if not all(np.isfinite(v) for v in values.values()):
            raise ValueError("All vehicle parameters must be finite")
        signed = {"lift_coefficient_area_m2", "aero_center_offset_m",
                  "longitudinal_E", "lateral_E"}
        nonnegative = {"cg_height_m", "rolling_resistance_coefficient",
                       "drag_coefficient_area_m2"}
        for name, value in values.items():
            if name in signed:
                continue
            if value < 0 if name in nonnegative else value <= 0:
                raise ValueError(f"Invalid vehicle parameter {name}")
        if not 0 < self.lf_m < self.wheelbase_m:
            raise ValueError("Require 0 < lf < wheelbase")
        if not 0 < self.steering_limit_rad < math.pi / 2:
            raise ValueError("Steering limit must be between zero and pi/2")


def _finite_vector(value: Sequence[float], length: int, name: str) -> np.ndarray:
    out = np.asarray(value, dtype=float)
    if out.shape != (length,) or not np.isfinite(out).all():
        raise ValueError(f"{name} must contain {length} finite values")
    return out


def magic_formula(slip, B: float, C: float, E: float):
    """Unit-peak pure-slip shape; |output| <= 1, derivative(0)=B*C."""
    b = B * np.asarray(slip)
    return np.sin(C * np.arctan(b - E * (b - np.arctan(b))))


def fiala_lateral(alpha, stiffness, capacity):
    """Classic saturated Fiala with OCD's positive-Fy slip-angle convention.

    q=C*tan(alpha)/(3*capacity).  For |q|<1, Fy=capacity*
    (3*q-3*q*abs(q)+q**3); otherwise Fy=capacity*sign(q).
    Using the tangent threshold avoids confusing alpha with tan(alpha).
    """
    alpha, stiffness, capacity = np.broadcast_arrays(alpha, stiffness, capacity)
    demand = stiffness * np.tan(alpha)
    q = np.minimum(np.abs(demand) / np.maximum(3.0 * capacity, 1e-12), 1.0)
    return np.sign(demand) * capacity * (3.0 * q - 3.0 * q*q + q*q*q)


class ReducedDoubleTrack:
    """11-state wheel-torque plant with instantaneous algebraic tire forces.

    ``step`` and ``reset`` return seven observations [X,Y,psi,vx,vy,r,delta].
    ``state`` is the full 11-state snapshot; reset accepts that snapshot too.
    Quasi-static load transfer solves a force/load algebraic fixed point; it
    has no suspension oscillation and no tire-relaxation delay.  The lateral
    roll-moment split follows the static axle weight fractions, an explicit
    benchmark assumption rather than a measured anti-roll-bar calibration.
    """

    def __init__(self, tire_model: str = "mf_independent",
                 load_transfer: str = "quasistatic",
                 parameters: VehicleParameters | None = None,
                 integration_step_s: float = 0.001,
                 fiala_load_sensitivity: bool = False):
        if tire_model not in {"mf_independent", "mf_ellipse", "fiala"}:
            raise ValueError("Unknown tire_model")
        if load_transfer not in {"static", "quasistatic"}:
            raise ValueError("load_transfer must be static or quasistatic")
        if not np.isfinite(integration_step_s) or integration_step_s <= 0:
            raise ValueError("integration_step_s must be finite and positive")
        self.parameters = parameters or VehicleParameters()
        self.parameters.validate()
        self.tire_model = tire_model
        self.load_transfer = load_transfer
        self.integration_step_s = float(integration_step_s)
        self.fiala_load_sensitivity = bool(fiala_load_sensitivity)
        p = self.parameters
        self._wheel_x = np.array([p.lf_m, p.lf_m, -p.lr_m, -p.lr_m])
        self._wheel_y = np.array([p.front_track_m/2, -p.front_track_m/2,
                                  p.rear_track_m/2, -p.rear_track_m/2])
        self._inertia = np.array([p.wheel_inertia_front_kgm2]*2 +
                                 [p.wheel_inertia_rear_kgm2]*2)
        self._static_loads = p.mass_kg*p.gravity_mps2*np.array(
            [p.front_weight_fraction/2]*2 + [(1-p.front_weight_fraction)/2]*2)
        self._load_dx = p.mass_kg*p.cg_height_m/(2*p.wheelbase_m)*np.array([-1., -1., 1., 1.])
        self._load_dy = p.mass_kg*p.cg_height_m*np.array(
            [-p.front_weight_fraction/p.front_track_m, p.front_weight_fraction/p.front_track_m,
             -(1-p.front_weight_fraction)/p.rear_track_m, (1-p.front_weight_fraction)/p.rear_track_m])
        self._state = np.zeros(11)
        self._torques = np.zeros(4)
        self._steering = 0.0
        self._friction = np.ones(4)

    @staticmethod
    def observe(state: Sequence[float]) -> np.ndarray:
        z = _finite_vector(state, 11, "state")
        return np.r_[z[:6], z[10]]

    @property
    def observation(self) -> np.ndarray:
        return self.observe(self._state)

    @property
    def state(self) -> np.ndarray:
        return self._state.copy()

    @property
    def full_state(self) -> np.ndarray:
        return self.state

    @property
    def diagnostics(self) -> dict:
        return self.evaluate(self._state, self._torques, self._steering,
                             self._friction)[1]

    def reset(self, state: Sequence[float], steering: float | None = None,
              friction: Sequence[float] = (1., 1., 1., 1.)) -> np.ndarray:
        z = np.asarray(state, dtype=float)
        if z.shape == (6,):
            z = np.r_[z, 0. if steering is None else steering]
        if z.shape == (7,):
            z = _finite_vector(z, 7, "observation")
            vxw, _ = self.wheel_velocities(z[:6], z[6])
            z = np.r_[z[:6], np.maximum(vxw, 0.)/self.parameters.wheel_radius_m, z[6]]
        z = _finite_vector(z, 11, "state")
        if abs(z[10]) > self.parameters.steering_limit_rad + 1e-10 or np.any(z[6:10] < 0):
            raise ValueError("Initial steering/wheel speeds outside model domain")
        self._state = z.copy()
        self._torques = np.zeros(4)
        self._steering = float(z[10])
        self._friction = self._check_friction(friction)
        return self.observation

    @staticmethod
    def _check_friction(friction):
        mu = _finite_vector(friction, 4, "friction")
        if np.any(mu < 0):
            raise ValueError("Friction multipliers must be nonnegative")
        return mu.copy()

    def wheel_velocities(self, body_state, steering):
        z = np.asarray(body_state)
        vx = z[3] - z[5]*self._wheel_y
        vy = z[4] + z[5]*self._wheel_x
        c, s = math.cos(steering), math.sin(steering)
        vtx, vty = vx.copy(), vy.copy()
        vtx[:2] = c*vx[:2] + s*vy[:2]
        vty[:2] = -s*vx[:2] + c*vy[:2]
        return vtx, vty

    def normal_loads(self, ground_ax: float, ground_ay: float,
                     speed_mps: float) -> tuple[np.ndarray, bool]:
        """Loads with conserved total weight/downforce and unilateral contact.

        ground_ax/ay mean the summed tire forces divided by mass.  Using ground
        forces accounts for the couple between tire traction and drag applied
        at the CoG.  Downforce acts at the configured aero center.  When an
        algebraic load becomes negative, contact is capped and the diagnostic
        flag marks the model outside its four-contact operating envelope.
        """
        p = self.parameters
        down = -0.5*p.air_density_kgpm3*p.lift_coefficient_area_m2*speed_mps**2
        total = p.mass_kg*p.gravity_mps2 + down
        if total <= 0:
            raise FloatingPointError("Aero lift exceeds weight; planar contact model invalid")
        front = total*p.front_weight_fraction + down*p.aero_center_offset_m/p.wheelbase_m
        if self.load_transfer == "quasistatic":
            front -= p.mass_kg*ground_ax*p.cg_height_m/p.wheelbase_m
        raw_front = front
        front = min(total, max(0., front))
        rear = total-front
        df = dr = 0.
        if self.load_transfer == "quasistatic":
            roll_moment = p.mass_kg*ground_ay*p.cg_height_m
            df = roll_moment*p.front_weight_fraction/p.front_track_m
            dr = roll_moment*(1-p.front_weight_fraction)/p.rear_track_m
        outside = raw_front < 0 or raw_front > total or abs(df) > front/2 or abs(dr) > rear/2
        df = min(front/2, max(-front/2, df))
        dr = min(rear/2, max(-rear/2, dr))
        return np.array([front/2-df, front/2+df, rear/2-dr, rear/2+dr]), outside

    def tire_forces(self, alpha, kappa, loads, friction=(1., 1., 1., 1.)):
        """Return tire-frame Fx,Fy for four independently evaluated wheels."""
        p = self.parameters
        loads = np.asarray(loads)
        peak = p.nominal_friction*np.asarray(friction)*loads
        fx = peak*magic_formula(kappa, p.longitudinal_B, p.longitudinal_C, p.longitudinal_E)
        if self.tire_model.startswith("mf_"):
            fy = peak*magic_formula(np.tan(alpha), p.lateral_B, p.lateral_C, p.lateral_E)
            if self.tire_model == "mf_ellipse":
                # Berntorp Eq.4.9: preserve Fx; reduce Fy by remaining grip.
                ratio = np.divide(fx, peak, out=np.zeros_like(fx), where=peak > 0)
                fy *= np.sqrt(np.maximum(0., 1.-ratio*ratio))
        else:
            # Same reference small-slip slope as the normalized native MF.
            stiffness = p.nominal_friction*p.lateral_B*p.lateral_C*loads
            if self.fiala_load_sensitivity:
                # Bertipaglia Eq.4.5 shape, normalized to the SAME nominal
                # slope at each wheel's static load; not his fitted c1/Fz0.
                c2 = p.fiala_load_shape_c2
                shape = np.sin(2*np.arctan(loads/(c2*self._static_loads)))
                stiffness = (p.nominal_friction*p.lateral_B*p.lateral_C*self._static_loads
                             *shape/math.sin(2*math.atan(1/c2)))
            # lambda_mue scales the entire tire force as in upstream OCD.
            stiffness = stiffness*np.asarray(friction)
            capacity = np.sqrt(np.maximum(0., peak*peak-fx*fx))
            fy = fiala_lateral(alpha, stiffness, capacity)
        return fx, fy

    def _forces(self, z, torques, friction):
        p = self.parameters
        vx, vy, r, delta = z[3], z[4], z[5], z[10]
        vtx, vty = self.wheel_velocities(z, delta)
        alpha = -np.arctan2(vty, np.maximum(1., vtx))
        kappa = np.clip((p.wheel_radius_m*z[6:10]-vtx)/np.maximum(3., vtx), -1., 1.)
        c, s = math.cos(delta), math.sin(delta)
        drag_x = -0.5*p.air_density_kgpm3*p.drag_coefficient_area_m2*vx*abs(vx)
        drag_y = -0.5*p.air_density_kgpm3*p.drag_coefficient_area_m2*vy*abs(vy)

        def evaluate_load(ax, ay):
            loads, contact_limited = self.normal_loads(ax, ay, vx)
            fx, fy = self.tire_forces(alpha, kappa, loads, friction)
            bx, by = fx.copy(), fy.copy()
            bx[:2] = c*fx[:2] - s*fy[:2]
            by[:2] = s*fx[:2] + c*fy[:2]
            return loads, fx, fy, bx, by, contact_limited

        # Solve Fz = load_transfer(sum Fx/Fy) at the current physical state;
        # this is algebraic, not a load state carried between RK stages.
        # With constant normalized stiffness all these tire laws are homogeneous
        # in Fz.  Hence a = B(Fz0+L*a)/m is a 2x2 linear system, NOT an iterative
        # approximation.  Iterate only for optional load-sensitive Fiala or
        # when unilateral contact clipping makes the relation piecewise linear.
        ax, ay = 0., 0.
        residual = 0.
        iterations = 1
        direct_solution = False
        if not (self.tire_model == "fiala" and self.fiala_load_sensitivity):
            ux, uy = self.tire_forces(alpha, kappa, np.ones(4), friction)
            bx_unit, by_unit = ux.copy(), uy.copy()
            bx_unit[:2] = c*ux[:2]-s*uy[:2]
            by_unit[:2] = s*ux[:2]+c*uy[:2]
            base_loads, _ = self.normal_loads(0., 0., vx)
            if self.load_transfer == "quasistatic":
                aa = 1.-float(bx_unit @ self._load_dx)/p.mass_kg
                ab = -float(bx_unit @ self._load_dy)/p.mass_kg
                ba = -float(by_unit @ self._load_dx)/p.mass_kg
                bb = 1.-float(by_unit @ self._load_dy)/p.mass_kg
                ra = float(bx_unit @ base_loads)/p.mass_kg
                rb = float(by_unit @ base_loads)/p.mass_kg
                det = aa*bb-ab*ba
                if abs(det) < 1e-10:
                    raise FloatingPointError("Singular quasi-static load equilibrium")
                ax, ay = (bb*ra-ab*rb)/det, (aa*rb-ba*ra)/det
            loads, contact_limited = self.normal_loads(ax, ay, vx)
            if not contact_limited:
                fx, fy = ux*loads, uy*loads
                bx, by = bx_unit*loads, by_unit*loads
                direct_solution = True
                if self.load_transfer == "quasistatic":
                    residual = max(abs(float(np.sum(bx))/p.mass_kg-ax),
                                   abs(float(np.sum(by))/p.mass_kg-ay))
        if not direct_solution and self.load_transfer == "quasistatic":
            ax, ay = 0., 0.
            for iterations in range(1, 21):
                values = evaluate_load(ax, ay)
                ax_new = float(np.sum(values[3]))/p.mass_kg
                ay_new = float(np.sum(values[4]))/p.mass_kg
                residual = max(abs(ax_new-ax), abs(ay_new-ay))
                ax, ay = ax_new, ay_new
                if residual < 1e-7:
                    break
            if residual >= 1e-5:
                raise FloatingPointError(f"Quasi-static load fixed point did not converge: {residual:g} m/s2")
        if not direct_solution:
            loads, fx, fy, bx, by, contact_limited = evaluate_load(ax, ay)
        force_x, force_y = float(np.sum(bx)), float(np.sum(by))
        ax_total, ay_total = (force_x+drag_x)/p.mass_kg, (force_y+drag_y)/p.mass_kg
        yaw_moment = float(self._wheel_x @ by - self._wheel_y @ bx)
        return (loads, fx, fy, alpha, kappa, vtx, ax_total, ay_total, yaw_moment,
                residual, iterations, contact_limited, drag_x, drag_y)

    def evaluate(self, state, torques, steering, friction=(1., 1., 1., 1.)):
        z = _finite_vector(state, 11, "state")
        torque = _finite_vector(torques, 4, "torques")
        mu = self._check_friction(friction)
        if not np.isfinite(steering):
            raise ValueError("Steering must be finite")
        if abs(steering) > self.parameters.steering_limit_rad + 1e-10:
            raise ValueError("Steering command outside configured limit")
        return self._evaluate(z, torque, float(steering), mu, diagnostics=True)

    def _evaluate(self, z, torques, steering, friction, diagnostics=False):
        p = self.parameters
        (loads, fx, fy, alpha, kappa, vtx, ax, ay, mz, residual, iterations,
         contact_limited, drag_x, drag_y) = self._forces(z, torques, friction)
        vx, vy, r, delta = z[3], z[4], z[5], z[10]
        cp, sp = math.cos(z[2]), math.sin(z[2])
        rhs = np.empty(11)
        rhs[:6] = (vx*cp-vy*sp, vx*sp+vy*cp, r, ax+r*vy, ay-r*vx, mz/p.yaw_inertia_kgm2)
        extra_inertia = max(0., 15*p.wheel_inertia_rear_kgm2*(1.-float(np.mean(z[6:10]))/20.))
        load_torque = (fx+p.rolling_resistance_coefficient*loads)*p.wheel_radius_m
        wheel_rate = (torques-load_torque)/(self._inertia+extra_inertia)
        rhs[6:10] = np.where(z[6:10] < 1e-5, np.maximum(wheel_rate, 0.), wheel_rate)
        rate = min(p.steering_rate_limit_radps, max(-p.steering_rate_limit_radps,
                    (steering-delta)/p.steering_time_constant_s))
        if delta >= p.steering_limit_rad:
            rate = min(0., rate)
        if delta <= -p.steering_limit_rad:
            rate = max(0., rate)
        rhs[10] = rate
        if not diagnostics:
            return rhs
        peak = p.nominal_friction*friction*loads
        utilization = np.divide(np.hypot(fx, fy), peak, out=np.zeros(4), where=peak > 0)
        details = {
            "acceleration_mps2": np.array([ax, ay, 0.]),
            "yaw_acceleration_radps2": mz/p.yaw_inertia_kgm2,
            "tire_slip_angle_rad": alpha.copy(),
            "tire_longitudinal_slip": kappa.copy(),
            "tire_force_longitudinal_N": fx.copy(),
            "tire_force_lateral_N": fy.copy(),
            "tire_force_vertical_N": loads.copy(),
            "wheel_load_torque_Nm": load_torque.copy(),
            "wheel_speeds_radps": z[6:10].copy(),
            "steering_angle_rad": np.array([delta, delta, 0., 0.]),
            "tire_grip_utilization": utilization,
            "load_balance_residual_mps2": residual,
            "load_balance_iterations": iterations,
            "contact_load_limited": bool(contact_limited),
            "roll_rad": 0., "pitch_rad": 0.,
            "aerodynamic_force_N": np.array([drag_x, drag_y,
                 0.5*p.air_density_kgpm3*p.lift_coefficient_area_m2*vx*vx]),
        }
        return rhs, details

    def step(self, torques, steering: float, dt: float,
             substeps: int | None = None, friction=(1., 1., 1., 1.)) -> np.ndarray:
        torque = _finite_vector(torques, 4, "torques")
        mu = self._check_friction(friction)
        p = self.parameters
        if (not np.isfinite(dt) or dt <= 0 or not np.isfinite(steering)
                or abs(steering) > p.steering_limit_rad+1e-10):
            raise ValueError("Positive finite dt and a steering command inside its bounds are required")
        if substeps is None:
            substeps = max(1, math.ceil(dt/self.integration_step_s-1e-12))
        if isinstance(substeps, bool) or int(substeps) != substeps or substeps < 1:
            raise ValueError("substeps must be a positive integer")
        h = dt/int(substeps)
        z = self._state.copy()
        for _ in range(int(substeps)):
            k1 = self._evaluate(z, torque, steering, mu)
            k2 = self._evaluate(z+h*k1/2, torque, steering, mu)
            k3 = self._evaluate(z+h*k2/2, torque, steering, mu)
            k4 = self._evaluate(z+h*k3, torque, steering, mu)
            z += h*(k1+2*k2+2*k3+k4)/6
            # Forward-motion wheel contact, matching the native drivetrain.
            z[6:10] = np.maximum(z[6:10], 0.)
            z[10] = min(p.steering_limit_rad, max(-p.steering_limit_rad, z[10]))
        if not np.isfinite(z).all():
            raise FloatingPointError("Reduced-model integration produced nonfinite states")
        self._state, self._torques, self._steering, self._friction = z, torque.copy(), float(steering), mu
        return self.observation

    def close(self):
        """Uniform lifecycle with the native adapter; no external allocation."""

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
