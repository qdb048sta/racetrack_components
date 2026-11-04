# Validation: liu2023

Base: `e67066071572d9dbda95604239cc7a43d9615208`.

Environment: Python 3.12.14, NumPy 2.3.5, SciPy 1.17.0, CasADi 3.7.2.
CPU thread limits: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1`.

- 5 paper model/controller tests passed.
- 18 existing road/component tests passed.
- 13 existing Python driving tests passed.
- Native OCD not rebuilt: its code is unchanged.
- Tests are numerical/model checks, not experimental validation or control guarantees.

## Recorded run: liu2023

```json
{
  "experiment": "Liu 2023 tire submodel laboratory",
  "mode_2_frequencies_hz": [
    52.32415177998326,
    753.4897010330345
  ],
  "frequency_mean_hz": 52.32223893202193,
  "frequency_variance_hz2": 0.19919540797854798,
  "pcm_max_validation_error_hz": 1.9733281675371472e-07,
  "mpc_present_in_source": false,
  "scope": "Nonrotating in-plane modes, prescribed-force rigid ring, relaxation, uniform-parameter PCM; no contact solver or vehicle"
}
```


Run commands and equation scope are in the corresponding named paper analysis. Generated plots/CSVs remain ignored outputs, so results can be regenerated locally.
