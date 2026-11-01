# Install and reuse the numerical road model

Install the source checkout into the Python environment used by your simulator:

```bash
python -m pip install -e .
```

The distribution is named `racetrack-components`; the import is `tracklab`.
The core dependency set is NumPy and SciPy. It does not install CasADi, import a
native vehicle binding, compile a car, or start a simulation.

```python
import numpy as np
from tracklab import PeriodicTrack

road = PeriodicTrack("path/to/ordered_track.csv", method="periodic_cubic")
n = max(8, int(np.ceil(road.length / 1.0)))
s = np.linspace(0.0, road.length, n, endpoint=False)
q = road.sample_geometry(s)
centerline_xy = q["center"]
heading = q["heading_rad"]
curvature = q["curvature_1pm"]
right_width, left_width = q["width_right_m"], q["width_left_m"]
projection = road.project(centerline_xy[0], hint_s=0.0)
```

Input columns are x, y, right half-width, left half-width, in metres and cyclic
track order. Headers must start with `#`. Use the original four-column CSV,
not the richer exported geometry CSV. A matching repeated closing endpoint is
removed. Widths must be positive; consecutive duplicates, non-finite values,
and unexpectedly large closing gaps are rejected.

The centerline is an exact periodic cubic or quintic interpolant; widths use
cyclic PCHIP. Arc length and its inverse are numerical approximations. Omitting
the repeated endpoint from controller arrays avoids a zero closing segment.
The road supplies reference geometry and point margins, not a guarantee of
full-vehicle clearance. See [the road API](ROAD.md) and
[interpolation details](INTERPOLATION.md).

An external simulator should retain its own vehicle and tire models, effective
parameter files, integration timestep, friction model and actuator contract.
Sample the road numerically before a controller solve and pass the resulting
arrays as optimizer parameters. SciPy road objects are not CasADi symbolic
expressions. Track widths, friction multipliers and tire parameters represent
different quantities; the CSV supplies no measured friction or elevation.

Optional plotting needs `python -m pip install '.[plots]'`. The demonstration
controllers need `'.[driving]'`, including their separate CasADi pin. Run those
demonstrations and the native builder from a source checkout. The wheel includes
templates and native source/notices but does not bundle a compiled native
library, input datasets, or a source download cache. An installed road API takes
an explicit CSV path.

For reproducible downstream use, build a wheel or pin the Git commit:

```bash
python -m pip install build
python -m build --wheel
python -m pip install dist/racetrack_components-0.1.0-py3-none-any.whl
```

The package retains source notices in its wheel. Distribute input datasets
with their corresponding notices and licenses separately.
