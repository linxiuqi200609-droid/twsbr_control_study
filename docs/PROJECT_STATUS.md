# Project status and continuation guide

Updated: 2026-08-30.

## Workspace and version history

- Workspace: `D:/Research/srtp`.
- Branch: `codex/five-controller-study-implementation`.
- Authorized remote: `https://github.com/linxiuqi200609-droid/twsbr_control_study.git`.
- Commit and push the current branch by default. Do not merge or push to `main` without a new user decision.
- All public MATLAB code uses English, MATLAB-compatible filenames and the explicit `setup_project` path allowlist.

## Completed implementation before this checkpoint

- Nonlinear balancing-robot plant and legacy attitude/cascade PID workflows.
- Five-controller factory, shared numerical simulator, differential evolution, deterministic and paired Monte Carlo experiment components.
- Environment preflight and generated LQR, LQI and fuzzy PID Simulink models.
- Five-controller MATLAB/Simulink equivalence checks, including raw timing and fuzzy gain-log validation. Last reviewed/pushed checkpoint for this part: `880bd67`.

## Current checkpoint: statistics

- Initial implementation: `b6aee0e`.
- Shared validation/summary refactor and additional error-contract tests: `008d5b2`.
- Final primitive validation coverage: `1a51447` (tests only; production unchanged).
- Full regression log reports **353 passed, 0 failed, 0 incomplete**, followed by a marker printed after `assertSuccess`.
- Independent final focused/compatibility verification: **32/32 passed**, actual process exit `0`; Code Analyzer: **0 findings**. The production refactor and its 353-test full suite were checked before the final test-only additions; the updated test file also passed independent analysis.
- The long full-regression launcher detached; its exit code was not captured. The persisted post-assertion marker and terminated MATLAB process are the evidence, not an inferred exit code.
- Statistics use only successful trials for continuous summaries/tests, while success rates and Wilson intervals count every trial. Failed/nonfinite trial metrics are excluded from continuous calculations, not deleted from the experiment data.
- Statistics review is complete: all findings addressed, no new Critical/Important issues. The seven public statistics functions and two internal helpers are ready for the figure/reporting stages.

## Next work

1. Implement Task 7: six paper-oriented figure families, each exported as PNG and PDF, with fixed five-controller order and explicit handling of no-success groups.
2. Implement Task 8: raw/result exports, reproducibility manifest, `run_control_study` entry point, documentation, and actual default Quick end-to-end validation.
3. Run final whole-branch review and completion checks. Do not claim the whole study is complete before actual Quick training/evaluation, five accepted equivalence results, expected artifacts, clean analysis/tests, and remote synchronization are verified.

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
