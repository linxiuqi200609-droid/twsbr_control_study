# Verified default Quick study results

The corrected default Quick run passed software execution and artifact
acceptance. LQR/LQI gains are designed from the nominal plant and stay fixed
while Monte Carlo physics is perturbed. All controllers share the same training
budget, held-out scenarios and paired uncertainty realizations. These small
Quick samples support pipeline verification and exploratory comparisons, not
a definitive superiority claim.

Run date: 2026-09-01, 00:12:06-00:19:10 Asia/Shanghai.
MATLAB R2026a, Windows 64-bit; actual process exit `0`.
Source: `858c6dfeeb6ee98aae77fd41c17e98f4393b438a` (clean before the run).

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
- Corrective-source full regression and final independent release verification:
  **395 passed, 0 failed, 0 incomplete**, all **164 MATLAB files** with zero
  Code Analyzer findings; both full-run process exits were captured as `0`.
- One whole-branch review and one scoped final-fix re-review completed; all ten
  findings addressed within scope, with the fuzzy-bound departure below explicit.

## Outcomes under the shared task-success criteria

| Controller | Held-out successes | Monte Carlo successes |
| --- | ---: | ---: |
| ATTITUDE_PID | 3 / 12 | 0 / 10 |
| CASCADE_PID | 12 / 12 | 10 / 10 |
| FUZZY_PID | 12 / 12 | 10 / 10 |
| LQR | 12 / 12 | 10 / 10 |
| LQI | 12 / 12 | 10 / 10 |

The attitude-only PID does not observe position. These are the NEW run's counts,
not copied historical acceptance: all ten LQR and all ten LQI Monte Carlo trials
changed in position ITAE and/or tilt RMS after nominal-design isolation, even
though the success totals stayed the same. The attitude PID's failed rows remain in
the common comparison, with `task_not_settled` recorded; they are not erased
or replaced. Continuous summaries and tests use successful trials only,
whereas success rates use all trials. Thus its Monte Carlo continuous metrics
are unavailable, not zero. These limited Quick samples validate the pipeline
and offer exploratory comparisons, not a definitive controller ranking.

## Runtime and statistical interpretation

`controller_runtime_seconds` sums controller update and after-actuation calls,
excluding design/reset, plant integration and logging. `mean_step_runtime_us`
divides that time by attempted controller evaluations, not plant samples;
`simulation_runtime_seconds` records the separate whole-simulator walltime.
All 110 ten-second trials completed 1,001 controller updates each. The general
zero-attempt case is unavailable (NaN), not zero-cost. Timing is machine/run
dependent and is not a hardware real-time guarantee.

The four omnibus metrics have five configured groups, four analyzed successful
groups and 40 successful Monte Carlo observations. The baseline remains in
success-rate denominators; it has no successful MC continuous-metric sample.
The control-energy field is an applied-input-squared control-cost proxy, not
measured electrical energy.

## Saved artifacts

The complete run is under `results/control_study_quick/`:

- `run_manifest.json`: source, effective options, stages, seeds, frozen
  vectors, environment, and the three configuration-file SHA256 hashes.
- `tuning/`: five optimizer results and frozen parameter vectors.
- `deterministic/`, `monte_carlo/`, `simulink_validation/`: all-trial metrics,
  parameter/complexity tables, and accepted equivalence results.
- `raw/`: sixty indexed deterministic MAT traces (26,710,749 bytes total;
  largest file 804,084 bytes).
- `statistics.xlsx`: eleven raw, summary, inference, parameter, and validation
  sheets; matching CSV outputs are also available.
- `figures/`: six figures, each as PNG and vector PDF.

All trial counts, parameter exports, configuration hashes, paired perturbations,
timing conversions, statistical group counts and raw paths were independently
reconciled. Every workbook sheet and relevant extended columns were visually
inspected, with zero spreadsheet error cells. All six PDFs parsed without
diagnostics and were viewed. After final tests, the six accepted SLX snapshots
were restored and all 88 Quick artifact hashes remained unchanged.

## Limits and historical evidence

- Fuzzy gains remain nonnegative with maxima of four times each candidate's
  base gains, fixed during online execution. This deliberately preserves the
  detailed implementation plan; it is not the high-level spec's stricter
  candidate-independent positive-bound policy. Adopting that alternative needs
  selected limits, code/model changes, tests and fresh tuning.
- The original 2026-08-31 run from `b20c382`, preserved in `8a14d7b`, remains
  diagnostic only because of MC redesign and mis-scoped timing. The new run
  supersedes it without retroactively validating those old comparisons.
- A historical legacy PNG-export failure did not recur in subsequent focused,
  full or final independent checks. Its cause remains unconfirmed; no masking
  was added. The actual Quick emitted Windows text-scaling/vector-export
  advisories, but all produced PDFs passed independent parsing and visual QA.
- This is a software-only, reduced-budget Quick study. Full-budget multi-seed
  experiments, physical hardware validation and definitive controller rankings
  have not been established by this run.
