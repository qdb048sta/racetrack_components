# TUM OCD validation and attribution

[Overview and running](README.md) · [Model](MODEL.md) · [Integration walkthrough](INTEGRATION.md) · [Validation](VALIDATION.md) · [Project guide](../../README.md)

## Recorded comparison

Both models completed one lap of the unchanged `2296.312367 m` reference course,
using their default configuration, an `18 m/s` reference-speed cap, and the same
`2 m/s²` planning budget. The original model was rerun into
`drive_comparison_results/current`; the native run was written to
`drive_OCD_results`. Those generated folders were subsequently cleared from
the working tree; the historical files were moved to the ignored local
`dev/result_archive/`. Rerun the [run commands](README.md#run) to create current reports.

| Recorded quantity | Existing `driving` | Native `driving_OCD` |
| --- | ---: | ---: |
| Lap crossing time | `167.443237 s` | `167.443495 s` |
| Successful MPC solves | `1675 / 1675` | `1675 / 1675` |
| Maximum cross-track error | `4.576444 mm` | `4.642721 mm` |
| RMS cross-track error | `0.454195 mm` | `0.467911 mm` |
| Maximum actual speed | `18.011896 m/s` | `18.012231 m/s` |
| Maximum absolute steering | `18.681193°` | `18.707782°` |
| Minimum sampled body margin | `3.564080 m` | `3.563817 m` |
| Sampled body points outside the road | `0` | `0` |

These numbers quantify tracking under a common conservative reference schedule.
The native equations are checked separately through source identity, force-law,
state-mapping, and symbolic/native derivative tests. The table records an example
run. Generated metrics are local outputs rather than committed files; rerunning
the models creates `drive_comparison_results/current/metrics.json` and
`drive_OCD_results/metrics.json`.

All **49 tests passed** in the recorded test run (`17.053 s`), including the
geometry/interpolation tests, existing driving tests, and all 18 new native
tests. Run the [test command](README.md#smoke-test-full-lap-and-tests) to print a new test report.
Replaying the native lap's fixed controls with a halved integration interval
(`0.0001 s`) produced a maximum logged endpoint position difference of
`0.000642582 m`. A full native run regenerates the timestep-refinement record at
`drive_OCD_results/independent_integration_check.json`.
That is an open-loop numerical sensitivity check with the recorded commands.

## Verification and attribution

`tests/test_driving_OCD.py` checks native state mapping; wheel-torque coupling;
the upstream slip definitions; exact linear tire forces and external scaling;
absence of an added force cap; steering limits; stationary guards; symbolic
derivatives against native derivatives; numerical stepping; the unchanged road
speed planner; and a short closed-loop simulation. Missing native dependencies
cause an explicit failure rather than a substituted model.

The downloaded build inputs retain upstream copyright and license notices.
The pinned TUM source and selected dependencies use the Apache License 2.0;
copies are retained as `native/LICENSE-*.txt`. Local bridge/controller code is
separate. Native component use does not make this course, parameter set,
controller, or output an official TUM validation result. Track widths, friction,
elevation, and numerical footprint checks retain the limitations described in
the [road reference](../../docs/ROAD.md).
