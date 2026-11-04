# Validation: gao2014

Base: `e67066071572d9dbda95604239cc7a43d9615208`.

Environment: Python 3.12.14, NumPy 2.3.5, SciPy 1.17.0, CasADi 3.7.2.
CPU thread limits: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1`.

- 7 paper model/controller tests passed.
- 18 existing road/component tests passed.
- 13 existing Python driving tests passed.
- Native OCD not rebuilt: its code is unchanged.
- Tests are numerical/model checks, not experimental validation or control guarantees.

## Recorded run: gao2014

```json
{
  "experiment": "Gao 2014 spatial/temporal MPC adaptation",
  "steps": 40,
  "solves": 80,
  "max_constraint_residual": 7.852603716145684e-09,
  "max_cross_track_m": 0.0015236881572613865,
  "max_planned_corridor_slack_m": 9.091681759058193e-11,
  "max_sampled_obstacle_violation_m": 0.0,
  "final_station_m": 19.500005337391407,
  "obstacle_enabled": false,
  "scope": "Fiala substitution; ideal side forces; nonlinear temporal tracker; no robust tube"
}
```

## Recorded run: gao_obstacle

```json
{
  "experiment": "Gao 2014 spatial/temporal MPC adaptation",
  "steps": 100,
  "solves": 200,
  "max_constraint_residual": 7.242504325433163e-09,
  "max_cross_track_m": 1.4959086702379054,
  "max_planned_corridor_slack_m": 0.13323075054165262,
  "max_sampled_obstacle_violation_m": 0.0,
  "final_station_m": 49.23162419666635,
  "obstacle_enabled": true,
  "scope": "Fiala substitution; ideal side forces; nonlinear temporal tracker; no robust tube"
}
```

Commands: `python run_paper_gao.py --steps 40` and `python run_paper_gao.py --steps 100 --obstacle --output outputs/papers/gao_obstacle`. The obstacle run passes the 25-35 m scenario and ends beyond 49 m. Clearance is checked at logged samples, with 0.4 m planned margin and reported soft-constraint slack; this is not a continuous footprint certificate. Earlier hard-corridor attempts exposed tracking mismatch; the final implementation records the slack instead of claiming that issue is solved by a robust tube.

Run commands and equation scope are in the corresponding named paper analysis. Generated plots/CSVs remain ignored outputs, so results can be regenerated locally.
