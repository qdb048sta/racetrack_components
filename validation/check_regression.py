"""Execute original and modular default benchmarks and compare non-timing outputs.

Run from project root: python validation/check_regression.py
This runs NO vehicle model. Saved logs describe only numerical refactoring checks.
"""
from pathlib import Path
import csv
import json
import sys
import importlib.util
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tracklab.evaluation import run_experiment
from tracklab.export import json_clean

spec = importlib.util.spec_from_file_location('regression_original', ROOT/'original'/'racetrack_interpolation_demo.py')
legacy = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = legacy
spec.loader.exec_module(legacy)
left, right = ROOT/'validation'/'original_results', ROOT/'validation'/'refactored_results'
legacy.run_experiment(ROOT/'data'/'Norisring.csv', left)
run_experiment(ROOT/'data'/'Norisring.csv', right)
reports = {}
for filename in ['comparison.csv','phase_details.csv','noise_comparison.csv','noise_details.csv']:
    with (left/filename).open() as f: original = list(csv.DictReader(f))
    with (right/filename).open() as f: refactored = list(csv.DictReader(f))
    assert len(original) == len(refactored)
    max_difference = 0.0
    for x, y in zip(original, refactored):
        for key in x:
            if key.endswith('_ms'): continue  # timings are inherently nondeterministic
            if x[key] == y[key]: continue
            a, b = float(x[key]), float(y[key])
            np.testing.assert_allclose(a, b, atol=1e-10, rtol=1e-10, equal_nan=True)
            max_difference = max(max_difference, abs(a-b))
    reports[filename] = {'rows':len(original), 'max_non_timing_difference':max_difference}
road_filename='norisring_interpolated_1m.csv'
reports[road_filename]={'byte_identical':(left/road_filename).read_bytes()==(right/road_filename).read_bytes()}
assert reports[road_filename]['byte_identical']
for filename in ['norisring_periodic_cubic_model.npz','plot_data.npz']:
    a, b = np.load(left/filename), np.load(right/filename)
    assert a.files == b.files
    for key in a.files:
        np.testing.assert_allclose(a[key], b[key], atol=1e-12, rtol=1e-12, equal_nan=True)
    reports[filename]={'arrays_compared':len(a.files), 'all_match_at_1e_minus_12':True}
original_meta=json.loads((left/'metadata.json').read_text())
new_meta=json.loads((right/'metadata.json').read_text())
assert original_meta==new_meta
reports['metadata.json']={'identical_dictionary':True}
(ROOT/'validation'/'regression_results.json').write_text(json.dumps(json_clean(reports),indent=2))
print(json.dumps(json_clean(reports),indent=2))
