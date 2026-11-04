"""Numerical and road adapters shared by the independent paper comparison branches."""
from pathlib import Path
import json
import numpy as np


def rk4(rhs, x, u, dt):
    if dt <= 0:
        raise ValueError("Step must be positive")
    k1 = rhs(x, u)
    k2 = rhs(x + dt * k1 / 2, u)
    k3 = rhs(x + dt * k2 / 2, u)
    k4 = rhs(x + dt * k3, u)
    return x + dt * (k1 + 2*k2 + 2*k3 + k4) / 6


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def ipopt_options():
    return {"print_time": False}, {"print_level": 0, "sb": "yes", "max_iter": 500,
                                  "tol": 1e-7, "acceptable_tol": 1e-6,
                                  "bound_relax_factor": 0.}


def solve_checked(op):
    """Propagate solve failures: an unsuccessful solve is never a valid command."""
    sol = op.solve()
    if not sol.stats()["success"]:
        raise RuntimeError(sol.stats()["return_status"])
    g = np.array(sol.value(op.g), dtype=float).ravel()
    lo = np.array(sol.value(op.lbg), dtype=float).ravel()
    hi = np.array(sol.value(op.ubg), dtype=float).ravel()
    violation = max(0., float(np.max(lo-g)), float(np.max(g-hi)))
    if not np.isfinite(g).all() or violation > 1e-5:
        raise RuntimeError(f"Constraint residual {violation:g}")
    return sol, violation


def road_reference(road, station, speed, dt, horizon, heading_anchor=None):
    s = station + speed * dt * np.arange(horizon+1)
    q = road.sample_geometry(s)
    heading = np.unwrap(q["heading_rad"])
    if heading_anchor is not None:
        heading += 2*np.pi*np.round((heading_anchor-heading[0])/(2*np.pi))
    return q, heading


def write_run(output, columns, rows, summary, road=None, xy_columns=None):
    """Persist inspectable trajectories, numerical diagnostics and a static plot."""
    out = Path(output)
    out.mkdir(parents=True, exist_ok=True)
    data = np.asarray(rows, dtype=float)
    if not np.isfinite(data).all():
        raise FloatingPointError("Nonfinite results")
    np.savetxt(out / "trajectory.csv", data, delimiter=",", header=",".join(columns), comments="")
    (out / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if road is not None and xy_columns is not None:
        fig, ax = plt.subplots(figsize=(8, 5))
        q = road.sample_geometry(np.linspace(0, road.length, 1500))
        for name in ["boundary_left", "boundary_right"]:
            ax.plot(*q[name].T, color="0.7", lw=.7)
        ax.plot(*data[:, xy_columns].T, label=summary["experiment"])
        ax.set(xlabel="X [m]", ylabel="Y [m]", aspect="equal")
        xy=data[:,xy_columns]
        ax.set_xlim(xy[:,0].min()-10,xy[:,0].max()+10)
        ax.set_ylim(xy[:,1].min()-10,xy[:,1].max()+10)
        ax.legend();ax.grid(alpha=.2)
    else:
        count=min(data.shape[1]-1,3)
        fig, axes=plt.subplots(count,1,figsize=(8,2.4*count),sharex=True,squeeze=False)
        for j,ax in enumerate(axes[:,0],1):
            ax.plot(data[:,0],data[:,j]);ax.set_ylabel(columns[j]);ax.grid(alpha=.2)
        axes[-1,0].set_xlabel(columns[0])
    fig.tight_layout(); fig.savefig(out / "trajectory.png", dpi=150); plt.close(fig)
    return summary
