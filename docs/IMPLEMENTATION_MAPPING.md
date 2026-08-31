# Five-controller study implementation mapping

This map connects the approved control-study requirements to their MATLAB
implementation and its primary verification entry points.

| Research requirement | Implementation | Verification entry point |
| --- | --- | --- |
| Shared nonlinear plant, sampling, saturation, and controller adapters | `models/`, `simulation/simulate_control_system.m`, `controllers/` | `tests/test_unified_*`, `tests/test_legacy_controller_adapters.m` |
| Five named controller parameter spaces and fair DE budgets | `controllers/controller_parameter_space.m`, `optimization/differential_evolution.m`, `optimization/tune_all_controllers.m` | `tests/test_controller_parameterization.m`, `tests/test_tuning_pipeline.m` |
| Training-only selection and held-out deterministic evaluation | `scenarios/training_scenarios.m`, `scenarios/heldout_scenarios.m`, `optimization/select_frozen_parameters.m`, `experiments/run_deterministic_batch.m` | `tests/test_training_scenarios.m`, `tests/test_heldout_scenarios.m`, `tests/test_deterministic_batch.m` |
| Paired uncertain-plant robustness cases | `config/monte_carlo_config.m`, `scenarios/generate_monte_carlo_scenario.m`, `experiments/run_monte_carlo.m` | `tests/test_monte_carlo_pairing.m` |
| Representative MATLAB/Simulink comparison for all controllers | `experiments/run_simulink_validation_batch.m`, `simulation/run_controller_simulink.m`, `simulation/compare_matlab_simulink.m` | `tests/test_simulink_equivalence.m`, `tests/test_unified_*equivalence.m` |
| Complexity and runtime reporting | `experiments/benchmark_controllers.m` | `tests/test_tuning_pipeline.m` |
| Descriptive and nonparametric statistics over frozen outcomes | `evaluation/summarize_*_results.m`, `evaluation/run_nonparametric_tests.m` | `tests/test_statistics.m` |
| Raw-trace preservation and path-safe index | `reporting/save_raw_simulation.m` | `tests/test_control_study_outputs.m` |
| Scenario metadata without executable handles | `reporting/scenario_to_record.m`, `reporting/write_json_file.m` | `tests/test_control_study_outputs.m` |
| Flat CSV/XLSX reporting and vector preservation | `reporting/write_results_tables.m` | `tests/test_control_study_outputs.m` |
| Reproducibility manifest and configuration hashes | `reporting/build_run_manifest.m` | `tests/test_control_study_outputs.m` |
| One-command workflow, explicit skipped stages, and stale-artifact control | `run_control_study.m`, `workflows/run_control_study_workflow.m` | `tests/test_control_study_outputs.m`, `tests/test_root_entry_points.m` |
| Paper figures from fixed held-out raw keys and paired Monte Carlo metrics | `visualization/generate_paper_figures.m` | `tests/test_paper_figures.m` |

The public root wrapper supports the documented calls
`run_control_study("quick", true)` and `run_control_study("full", true)`.
The optional third argument exists solely for controlled test/integration
execution; its effective values and every skipped stage are persisted in the
run manifest.
