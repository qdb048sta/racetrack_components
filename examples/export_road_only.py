"""Export the full-data road without running interpolation comparisons."""
from pathlib import Path
import sys
import json
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tracklab import PeriodicTrack

out = ROOT / "road_only_output"
out.mkdir(exist_ok=True)
road = PeriodicTrack(ROOT / "data" / "Norisring.csv")
print(json.dumps(road.export(out / "norisring_interpolated_1m.csv"), indent=2))
