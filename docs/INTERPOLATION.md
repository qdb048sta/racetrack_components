# Interpolation and road geometry equations

[Project overview](../README.md) · [Road API and workflow](ROAD.md) · [Driving equations](DRIVING.md)

Both driving packages use the same [data loader](../tracklab/data.py), [interpolants](../tracklab/interpolation.py), [geometry](../tracklab/geometry.py), and [road API](../tracklab/road.py). Neither vehicle model changes the track coordinates. Both run scripts construct `PeriodicTrack(args.track)`, whose centerline defaults to `periodic_cubic`; the road API also accepts `periodic_quintic`. The other methods below belong to the offline comparison, with cyclic PCHIP additionally used for road widths.

**Source distance and curve parameter.** For ordered points $p_i=(x_i,y_i)$, interpret indices cyclically, including the segment from the last point back to the first. [The loader](../tracklab/data.py#L15) removes a repeated final endpoint and computes

$$
d_i=\lVert p_{i+1}-p_i\rVert,\qquad u_0=0,\qquad
u_i=\sum_{j=0}^{i-1}d_j,\qquad U=\sum_{i=0}^{n-1}d_i.
$$

Thus $u$ measures distance along the source polyline. It is neither simulation time nor exact distance along the fitted centerline $C(u)=(x(u),y(u))$. Coordinates are fitted independently against these stations. Every `Curve` query wraps its argument to $u_0+((u-u_0)\bmod U)$.

**Seven fitted-curve methods.** For one coordinate $z$, write $h_i=u_{i+1}-u_i$, $q_i=(z_{i+1}-z_i)/h_i$, and $\tau=(u-u_i)/h_i$. The closing sample is $(u_n,z_n)=(U,z_0)$.

| Method | Construction | Continuity in $u$ |
|---|---|---|
| `linear` | Straight segments through every source point | $C^0$ |
| `periodic_cubic` | Exact periodic piecewise cubic | $C^2$ |
| `pchip_cyclic` | Cubic Hermite pieces with PCHIP slopes | $C^1$ |
| `akima_cyclic` | Cubic Hermite pieces with Akima slopes | $C^1$ |
| `makima_cyclic` | Cubic Hermite pieces with modified Akima slopes | $C^1$ |
| `periodic_quintic` | Exact periodic piecewise quintic | $C^4$ |
| `periodic_smoothing_cubic` | Regularized periodic cubic fit | $C^2$ |

Linear interpolation is $z(u)=(1-\tau)z_i+\tau z_{i+1}$. Its tangent jumps at corners; the API deliberately returns undefined curvature (`NaN`) rather than interpreting zero segment curvature as valid corner curvature.

The cubic and quintic splines have interval polynomials

$$
z_i(u)=\sum_{j=0}^{k}a_{ij}(u-u_i)^j,\qquad k\in\{3,5\}.
$$

They interpolate the supplied coordinates. Derivatives through order $k-1$ match between pieces and across the lap seam. [The implementation](../tracklab/interpolation.py#L70) delegates the coefficient solve to SciPy `CubicSpline(..., bc_type='periodic')` or `make_interp_spline(..., k=5, bc_type='periodic')`.

PCHIP, Akima, and Makima use the same cubic Hermite polynomial, with different knot slopes $m_i$:

$$
\begin{aligned}
z(u)={}&(2\tau^3-3\tau^2+1)z_i+(\tau^3-2\tau^2+\tau)h_i m_i\\
&+(-2\tau^3+3\tau^2)z_{i+1}+(\tau^3-\tau^2)h_i m_{i+1}.
\end{aligned}
$$

PCHIP sets $m_i=0$ when $q_{i-1}q_i\leq0$; otherwise it uses the weighted harmonic mean

$$
\frac{w_1+w_2}{m_i}=\frac{w_1}{q_{i-1}}+\frac{w_2}{q_i},\qquad
w_1=2h_i+h_{i-1},\quad w_2=h_i+2h_{i-1}.
$$

This preserves each scalar coordinate's local monotonicity; it does not establish that the planar curve has no intersections. See the [SciPy PCHIP slope definition](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.PchipInterpolator.html).

Akima uses

$$
m_i=\frac{w_1q_{i-1}+w_2q_i}{w_1+w_2},\qquad
w_1=|q_{i+1}-q_i|,\quad w_2=|q_{i-1}-q_{i-2}|.
$$

Makima uses the same slope formula with

$$
w_1=|q_{i+1}-q_i|+\tfrac12|q_{i+1}+q_i|,\qquad
w_2=|q_{i-1}-q_{i-2}|+\tfrac12|q_{i-1}+q_{i-2}|.
$$

SciPy supplies fallback slopes when the weights are degenerate or too small; see its [Akima/Makima definitions](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.Akima1DInterpolator.html). For these three cyclic methods, the repo prepends the last three samples shifted by $-U$ and appends the first three shifted by $+U$. This gives the seam the correct neighboring slopes; wrapping query positions alone would not do that. Their second derivatives, and therefore curvature, may jump at knots.

The smoothing method solves for fitted nodal positions $V=(v_i)$:

$$
\min_V\left[\sum_i\lVert v_i-p_i\rVert^2+
\lambda\int_0^U\lVert C_V''(u)\rVert^2\,du\right],\qquad
(I+\lambda K)V=P,\quad K_{jk}=\int_0^U B_j''(u)B_k''(u)\,du.
$$

Here $C_V$ is the periodic cubic through $V$ and $B_j$ are its cardinal basis functions. [The smoothing matrix](../tracklab/interpolation.py#L48) integrates the products exactly with two Gauss nodes per source interval, then uses a Cholesky solve. The parameter $\lambda$ has units $\mathrm m^3$ with these coordinates; it is a second-derivative penalty in $u$, rather than the geometric curvature energy $\int\kappa^2ds$ or a FITPACK residual budget. Smoothing is confined to the benchmark: moving the driving centerline would require preserving or reprojecting the original road borders.

**Arc length and local geometry.** [ArcLengthMap](../tracklab/geometry.py#L36) builds

$$
s(u)=\int_0^u\lVert C'(q)\rVert\,dq,\qquad L=s(U).
$$

The implementation uses eight subintervals per source segment and five-point Gauss quadrature per subinterval. Forward and inverse PCHIP maps approximate $s(u)$ and $u(s)$. Road queries first wrap $s$ modulo $L$, then evaluate the curve at the numerically inverted parameter. The inversion is not an analytic arc-length reparameterization: its $C^1$ interpolation means a quintic centerline alone does not make every sampled field $C^2$ in $s$.

For nonzero $\lVert C'(u)\rVert$, the [local frame](../tracklab/geometry.py#L14) is

$$
t=\frac{C'}{\lVert C'\rVert},\qquad n=(-t_y,t_x),\qquad
\psi_{\mathrm{road}}=\operatorname{atan2}(t_y,t_x),\qquad
\kappa=\frac{x'y''-y'x''}{(x'^2+y'^2)^{3/2}}.
$$

Primes here mean derivatives with respect to $u$; the curvature formula remains valid even though $u$ is not fitted-curve arc length. Positive curvature turns left. Heading is a principal angle and may wrap at $\pm\pi$.

The CSV widths are ordered **right, left**. [WidthProfile](../tracklab/road.py#L20) fits a cyclic scalar PCHIP to each, giving $w_R(u)$ and $w_L(u)$. The sampled road borders are

$$
b_R=C-w_Rn,\qquad b_L=C+w_Ln.
$$

The course map exported by `PeriodicTrack.export_map()` plots these sampled borders and centerline. The approximately one-metre export grid is a visualization/inspection grid, rather than the dynamics integration step or a clearance guarantee.

**Projection and vehicle coordinates.** `road.project()` estimates the closest station using a grid seed and bounded local minimization of $\lVert p-C(u(s))\rVert^2$. For the selected projection, the adapter defines $e=(p-C)\cdot n$ and $e_\psi=\operatorname{wrap}(\psi-\psi_{\mathrm{road}})$. With body-frame velocities $v_x,v_y$ and yaw rate $r$, its local geometric rate formulas are

$$
\dot s=\frac{v_x\cos e_\psi-v_y\sin e_\psi}{1-\kappa e},\qquad
\dot e=v_x\sin e_\psi+v_y\cos e_\psi,\qquad
\dot e_\psi=r-\kappa\dot s.
$$

These [adapter formulas](../tracklab/adapter.py#L96) assume a regular local road tube and the arc-length interpretation of $s$; they do not exactly differentiate the approximate inverse map. A positive $1-\kappa e$ does not establish global projection uniqueness. Point margins $w_L-e$ and $w_R+e$ do not guarantee clearance of a finite car footprint.

[FlatSurface](../tracklab/surface.py) supplies the explicit assumptions $\lambda_\mu=1$ and $z_{\mathrm{road}}=0$ by default. The CSV has no measured friction or elevation. The multiplier $\lambda_\mu$ scales a tire model's friction/force parameters; it is not itself a measured tire friction coefficient. `RoadCarAdapter` only maps road observations and wheel environments; the separate driving packages perform vehicle integration and tire-force calculations.
