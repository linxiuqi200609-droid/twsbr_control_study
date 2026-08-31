# Project status and continuation guide

Updated: 2026-09-01.

## Completed checkpoint: corrected five-controller Quick acceptance

The unified MATLAB/Simulink workflow is implemented and its corrected default
Quick run has passed the local execution and artifact gates. The entry call was
`run_control_study("quick", false)` with no third-argument overrides, from clean
source `858c6dfeeb6ee98aae77fd41c17e98f4393b438a` on 2026-09-01. Its process
started at 00:12:06 and exited normally at 00:19:10 (Asia/Shanghai).

- Five controllers used the same DE population 24 and budget 240 each, tuning
  seed 0 and global seed 2026; training/test/Monte Carlo durations were 10 seconds.
- All 60 deterministic and 50 paired Monte Carlo trials are retained; all five
  representative MATLAB/Simulink comparisons passed and all 12 figures exist.
- Nominal LQR/LQI design gains stay frozen while Monte Carlo physics is perturbed.
  Lifecycle timing is separate from total simulator walltime and uses actual
  controller-update counts. Missing Git metadata is nonfatal and explicitly unknown.
- The whole-branch review and its single correction/re-review wave are complete:
  `311a581` addressed three Important and seven Minor findings, with no new
  breakage identified in the scoped review through `858c6df`.
- Final independent release verification on 2026-09-01 passed **395/395 tests**,
  zero failed/incomplete, and **164 MATLAB files with zero Analyzer findings**.
  Both the Quick process and this full verification have captured exit `0` and
  persistent exit records. The earlier supplemental 104-test run also passed,
  but its process exit was lost after interruption and remains unknown.
- Root reconciled all configuration hashes, frozen vectors, counts, timings,
  paired perturbations and raw paths. All six PDFs parsed without diagnostics
  and were viewed; all eleven workbook sheets and eight extended ranges were
  inspected with zero spreadsheet error cells.
- The 60 indexed raw traces total 26,710,749 bytes (largest 804,084 bytes).
  Six actual Quick SLX files were restored from checked snapshots after tests;
  hashes confirmed all 88 Quick artifacts remained unchanged by the test suite.

Results and limitations are recorded in `docs/QUICK_STUDY_RESULTS.md`. This is
Quick-mode software acceptance, not a Full-budget study, a controller ranking,
hardware validation, or authorization to merge into `main`.

The older run from `b20c382`, preserved in artifact commit `8a14d7b`, remains
diagnostic only because it redesigned gains using perturbed MC plants and used
mis-scoped timing. It has been superseded, not retrospectively validated.

One historical full run encountered a PNG-export failure in an unchanged legacy
entry-point test. Subsequent focused, full, independent and final release runs
passed that path. Its cause remains unconfirmed; no retry or masking behavior
was added and the historical event is not claimed to be fixed.

The detailed fuzzy implementation currently uses nonnegative gains, with an
upper bound of four times each candidate's base gain, held fixed online. This
differs from the high-level spec's strictly positive, candidate-independent
wording. The completed correction wave preserves the existing numerical law and records
the difference explicitly; choosing new global positive limits would require a
separate scientific design decision and retuning.

## Workspace and version history

- Workspace: `D:/Research/srtp`.
- Branch: `codex/five-controller-study-implementation`.
- Authorized remote: `https://github.com/linxiuqi200609-droid/twsbr_control_study.git`.
- Corrected artifact checkpoint: `34791e5fd43faf7de4ad262017f90efcd7ec1b21`.
  Its push completed on 2026-09-01; local, tracking and live remote SHAs were
  verified equal with a clean worktree. This final handoff note follows that
  artifact checkpoint and does not change verified MATLAB source or outputs.
- Commit and push the current branch by default. Do not merge or push to `main` without a new user decision.
- All public MATLAB code uses English, MATLAB-compatible filenames and the explicit `setup_project` path allowlist.

## Completed implementation before this checkpoint

The historical sections below describe evidence at their named commits. Their
then-open review notes were addressed or explicitly dispositioned in the final
wave above; they are not additional pending implementation tasks.

- Nonlinear balancing-robot plant and legacy attitude/cascade PID workflows.
- Five-controller factory, shared numerical simulator, differential evolution, deterministic and paired Monte Carlo experiment components.
- Environment preflight and generated LQR, LQI and fuzzy PID Simulink models.
- Five-controller MATLAB/Simulink equivalence checks, including raw timing and fuzzy gain-log validation. Last reviewed/pushed checkpoint for this part: `880bd67`.

## Completed checkpoint: statistics

- Initial implementation: `b6aee0e`.
- Shared validation/summary refactor and additional error-contract tests: `008d5b2`.
- Final primitive validation coverage: `1a51447` (tests only; production unchanged).
- Full regression log reports **353 passed, 0 failed, 0 incomplete**, followed by a marker printed after `assertSuccess`.
- Independent final focused/compatibility verification: **32/32 passed**, actual process exit `0`; Code Analyzer: **0 findings**. The production refactor and its 353-test full suite were checked before the final test-only additions; the updated test file also passed independent analysis.
- The long full-regression launcher detached; its exit code was not captured. The persisted post-assertion marker and terminated MATLAB process are the evidence, not an inferred exit code.
- Statistics use only successful trials for continuous summaries/tests, while success rates and Wilson intervals count every trial. Failed/nonfinite trial metrics are excluded from continuous calculations, not deleted from the experiment data.
- Statistics review is complete: all findings addressed, no new Critical/Important issues. The seven public statistics functions and two internal helpers are ready for the figure/reporting stages.

## Historical checkpoint: publication-oriented figures

- Figure implementation: `d6221a1`; reviewed PDF compatibility fix: `aff47ce`.
- Six plot families are implemented, with fixed five-controller colors/order and PNG/vector-PDF export: nominal response, saturation response, disturbance recovery, Monte Carlo boxplots, performance comparison, and normalized radar.
- Full regression at `d6221a1`: **363 passed, 0 failed, 0 incomplete**, captured process exit `0`. Code Analyzer covered all 12 added MATLAB files with **0 findings**.
- The final change only removes the radar's zero radial tick while retaining the 0-to-1 score range and all numerical data. Its covering figure/statistics run passed **29/29**; Root's independent final figure run passed **8/8**, captured exit `0`, with both changed files Analyzer-clean.
- Root independently parsed all six normal PDFs and two no-success radar boundary cases: **8/8**, each exit `0` and no parser diagnostics. The initial zero-tick PDF syntax warning is fixed. Normal figure layouts and the final radar renders were inspected visually.
- Task review and the scoped fix review are complete, with no open Critical/Important findings. A non-blocking final-review candidate remains: make unavailable-controller information more explicit in radar legends/annotations, especially when all controllers have zero successful trials.
- At this historical figure checkpoint, previews used synthetic test fixtures and the real Quick study had not yet run. The later diagnostic Quick run is recorded below; neither synthetic previews nor the superseded diagnostic robustness results are accepted final comparison evidence.

## Historical checkpoint: unified workflow and diagnostic Quick run

- Task8 source committed locally as `3f18496` after resume on 2026-08-31; not yet pushed. The unified entry point, export helpers, mapping documentation, and output tests are present.
- Root independently verified this source: **375/375 tests passed**, no failed/incomplete tests; Code Analyzer covered **159 tracked MATLAB files with 0 findings**, captured process exit `0`. Persistent post-assertion marker: `TASK8_ROOT_VERIFICATION_OK tests=375 files=159 findings=0`. Test-generated legacy binaries were restored by exact path.
- The focused output suite passed **12/12**. Root imported, rendered, and viewed all eleven sheets of both synthetic reporter and frozen/disabled workflow previews. Populated text widths, numeric parameter vectors, and header-only empty data schemas were verified; no spreadsheet error cells were found. These are export/integration checks, not the formal Quick experiment.
- Initial task review found one Important gap: rejected Simulink comparisons still allowed successful return. Fix `3d886b1` now persists the failed evidence and accurate stage metadata, then raises `twsbr:study:simulink_validation_failed` unless the five comparisons are accepted. Its 13-test output suite and root-entry compatibility test passed. Scoped re-review is complete with no open Important findings or new breakage.
- Root's latest independent verification on `3d886b1` passed **376/376 tests**, no failed/incomplete tests; **159 tracked MATLAB files, 0 Analyzer findings**, captured process exit `0`. Evidence: `task-8-root-r1-full-verification.log`, final marker `TASK8_ROOT_VERIFICATION_OK tests=376 files=159 findings=0`. A Minor display issue remains for final review: long headers clip in empty XLSX sheets that have no native column-width records.
- The actual default Quick run completed from clean source `b20c382`, with captured exit `0`: five controllers each used population24/evaluation budget240/seed0; training/test/MC durations stayed10seconds;60 deterministic and50 Monte Carlo rows were retained;5/5 Simulink comparisons passed;12 figures were generated. No frozen-vector/reduced/skipped-stage overrides were used. All configuration hashes, parameter exports, raw-index paths and stage metadata passed independent reconciliation. Manifest pre-run `source_dirty=false`.
- Root imported and visually inspected all11 actual workbook sheets and relevant extra columns; zero spreadsheet error cells. All6 actual PDFs parsed with exit0 and empty diagnostics and were viewed. Remaining final-review candidates concern unavailable-controller annotations, edge-label margins, the omnibus group-count definition, and earlier documented minor items.
- Historical diagnostic Quick success counts: attitude PID3/12 held-out and0/10 Monte Carlo; the other four controllers each12/12 and10/10. All19 failed baseline rows remain recorded as `task_not_settled`. Because of the nominal-design defect identified above, these robustness counts must not support fair-comparison conclusions. See `docs/QUICK_STUDY_RESULTS.md`; no Full-study or hardware result is claimed.
- Sixty compact Quick raw traces (26.7MB total, maximum0.804MB per file) are selected for versioning by validated index paths so the saved raw index stays usable after clone. A separate1.42GB debugging snapshot and large Full artifacts remain local/ignored. The earlier usage interruption preserved all files, and no usage-reset credits were consumed.
- Local continuation records include `task-8-report.md`, `task-8-root-qa.md`, and persistent `task-8-*.log` files under the active plan's ignored SDD directory. The latest log/report and Git state take precedence over these checkpoint counts as work progresses.

## Next research stage

The eight-task implementation plan, final correction/review, corrected default
Quick, artifact QA and artifact synchronization are complete. Do not restart
them on continuation. The manifest deliberately identifies clean source
`858c6df`; generated outputs are versioned later in artifact checkpoint `34791e5`.

1. The next research stage can be a Full-budget multi-seed study using the
   existing entry point, after selecting its compute/time budget. No Full run
   is claimed here and no completed controller/model task needs to be restarted.
2. Before paper-level conclusions, retain the failed baseline trials, distinguish
   task success from upright stabilization, and decide whether the stricter fuzzy
   gain-bound alternative is actually required. Hardware validation is separate.

## Continuation sources

- Binding design: `docs/superpowers/specs/2026-08-26-five-controller-control-study-design.md`.
- Active plan: `docs/superpowers/plans/2026-08-26-simulink-reporting-implementation-plan.md`.
- Detailed local ledger, task briefs, review packages and verification logs: `.superpowers/sdd/2026-08-26-simulink-reporting-implementation-plan/` (Git-ignored; available in this workspace, not in a fresh clone).
- The local ledger is authoritative for in-flight work. In a fresh clone, inspect this document, commit history and current files; do not restart completed controller/model tasks merely because the original plan checkboxes remain unchecked.

## Execution precautions

- This Windows environment requires authorized MATLAB execution; sandbox startup can fail before running project code.
- Persist long MATLAB verification output with a logfile/diary and print a unique success marker after assertions. Keep tool session handles until exit; do not automatically repeat a long run merely because temporary output detached.
- Run only one MATLAB verification/simulation process at a time.
- Full tests regenerate tracked result/SLX artifacts. Restore only exact known test-generated paths after MATLAB exits, preserving unrelated user changes.
- Keep large raw Full-study data and temporary logs local. Do not substitute reduced test options for the actual default Quick completion gate.
