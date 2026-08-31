# Default Quick study results

Run date: 2026-08-31. MATLAB R2026a, Windows 64-bit.
Source: `b20c382725f122ef7cc36b4f0d5448b358b61d0e` (clean before the run).

The actual entry call was `run_control_study("quick", false)`. The `false`
skips the already completed test suite, not training, Monte Carlo, Simulink,
statistics, or figures. No third-argument overrides were used.

## Reproducibility and acceptance

- Five controllers, identical DE population 24 and 240 evaluations per controller;
  tuning seed 0, global seed 2026.
- Training, held-out, and Monte Carlo durations of 10 seconds. Parameters selected
  from training only, then frozen for all evaluation stages.
- Sixty held-out deterministic trials and fifty paired Monte Carlo trials.
- All five representative MATLAB/Simulink comparisons accepted. Maximum
  tilt discrepancy across the five comparisons: about 0.0000493 degrees.
- Full pre-run regression: 376 passed, 0 failed, 0 incomplete. All 159 tracked
  MATLAB files had zero Code Analyzer findings.

## Outcomes under the shared task-success criteria

| Controller | Held-out successes | Monte Carlo successes |
| --- | ---: | ---: |
| ATTITUDE_PID | 3 / 12 | 0 / 10 |
| CASCADE_PID | 12 / 12 | 10 / 10 |
| FUZZY_PID | 12 / 12 | 10 / 10 |
| LQR | 12 / 12 | 10 / 10 |
| LQI | 12 / 12 | 10 / 10 |

The attitude-only PID does not observe position. Its failed rows remain in
the common comparison, with `task_not_settled` recorded; they are not erased
or replaced. Continuous summaries and tests use successful trials only,
whereas success rates use all trials. Thus its Monte Carlo continuous metrics
are unavailable, not zero. These limited Quick samples validate the pipeline
and offer exploratory comparisons, not a definitive controller ranking.

## Saved artifacts

The complete run is under `results/control_study_quick/`:

- `run_manifest.json`: source, effective options, stages, seeds, frozen
  vectors, environment, and the three configuration-file SHA256 hashes.
- `tuning/`: five optimizer results and frozen parameter vectors.
- `deterministic/`, `monte_carlo/`, `simulink_validation/`: all-trial metrics,
  parameter/complexity tables, and accepted equivalence results.
- `raw/`: sixty indexed deterministic MAT traces (about 26.7 MB total).
- `statistics.xlsx`: eleven raw, summary, inference, parameter, and validation
  sheets; matching CSV outputs are also available.
- `figures/`: six figures, each as PNG and vector PDF.

All trial counts, parameter exports, configuration hashes, and raw paths were
independently reconciled. Every workbook sheet was visually inspected and all
six PDFs parsed without diagnostics. Final whole-branch review is still pending
for presentation details and previously ledgered metadata/documentation issues.

This is a software-only experiment. Full-budget study execution and physical
hardware validation have not been performed by this Quick run.
