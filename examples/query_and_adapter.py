"""Inspect one constructed car state. This is NOT a driven trajectory or MPC run.

From the project root: python examples/query_and_adapter.py
"""
from pathlib import Path
import sys
import json
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tracklab import PeriodicTrack
from tracklab.adapter import PlanarState, RoadCarAdapter
from tracklab.export import json_clean

road = PeriodicTrack(ROOT / "data" / "Norisring.csv")
s = 100.0
q = road.sample(s)
# Deliberately construct a state 0.25 m left of the reference, pointing along it.
xy = q["center"] + 0.25*q["normal_left"]
state = PlanarState(float(xy[0]), float(xy[1]), float(q["heading_rad"]),
                    vx_mps=10.0, vy_mps=0.0, yaw_rate_radps=0.0)
adapter = RoadCarAdapter(road)
observation = adapter.observe(state, hint_s=s)
result = {"track_length_m": road.length,
          "query_at_100m": q,
          "constructed_state": vars(state),
          "observation": observation,
          "coordinate_rates_not_a_simulation": adapter.frenet_rates(state, observation),
          "plain_python_environment_payload": adapter.wheel_environment(state)}
print(json.dumps(json_clean(result), indent=2))
