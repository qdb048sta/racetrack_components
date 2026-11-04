# Validation: beal2011

Base: `e67066071572d9dbda95604239cc7a43d9615208`.

Environment: Python 3.12.14, NumPy 2.3.5, SciPy 1.17.0, CasADi 3.7.2.
CPU thread limits: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1`.

- 6 paper model/controller tests passed.
- 18 existing road/component tests passed.
- 13 existing Python driving tests passed.
- Native OCD not rebuilt: its code is unchanged.
- Tests are numerical/model checks, not experimental validation or control guarantees.

## Recorded run: beal2011

```json
{
  "experiment": "Beal 2011 envelope MPC adaptation",
  "steps": 200,
  "solves": 200,
  "dt_s": 0.01,
  "max_envelope_slack": 9.090909103987799e-11,
  "max_constraint_residual": 2.6020852139652106e-18,
  "max_cross_track_m": 0.046607038170789816,
  "scope": "Chapter 4 lateral envelope; synthetic road-following driver; no roll/friction estimator"
}
```


Run commands and equation scope are in the corresponding named paper analysis. Generated plots/CSVs remain ignored outputs, so results can be regenerated locally.
