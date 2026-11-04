# Validation: sun2025

Base: `e67066071572d9dbda95604239cc7a43d9615208`.

Environment: Python 3.12.14, NumPy 2.3.5, SciPy 1.17.0, CasADi 3.7.2.
CPU thread limits: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1`.

- 6 paper model/controller tests passed.
- 18 existing road/component tests passed.
- 13 existing Python driving tests passed.
- Native OCD not rebuilt: its code is unchanged.
- Tests are numerical/model checks, not experimental validation or control guarantees.

## Recorded run: sun2025

```json
{
  "experiment": "Sun 2025 kinematic tracking core",
  "steps": 30,
  "solves": 30,
  "max_constraint_residual": 5.830109728321986e-10,
  "max_cross_track_m": 0.006763972282498722,
  "scope": "Supplied track reference; no APF, multi-agent protocol or tire-force model"
}
```

## Recorded run: sun_following

```json
{
  "experiment": "Sun 2025 nominal spatial following",
  "steps": 30,
  "solves": 30,
  "max_constraint_residual": 4.072603365656846e-12,
  "max_relaxation_gap_s_per_m": 9.661688978833405e-05,
  "physical_equality_fallbacks": 24,
  "total_optimization_solves": 54,
  "max_relaxed_candidate_gap_s_per_m": 0.00014349221074870522,
  "scope": "Supplied constant leader; no learned predictor or robust tube guarantee"
}
```


Run commands and equation scope are in the corresponding named paper analysis. Generated plots/CSVs remain ignored outputs, so results can be regenerated locally.
