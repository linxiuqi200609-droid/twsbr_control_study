# Simulink and Reporting Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and validate five reproducible Simulink controller models, then deliver statistics, figures, manifests, artifacts, documentation, and a one-command Quick or Full study workflow.

**Architecture:** Reuse the existing generated nonlinear plant subsystem in three new programmatic closed-loop builders, run one representative MATLAB/Simulink equivalence scenario per controller, and keep reporting downstream of frozen deterministic and Monte Carlo tables. The root workflow orchestrates existing tested units without embedding controller-specific mathematics.

**Tech Stack:** MATLAB, Simulink, Control System Toolbox, Statistics and Machine Learning Toolbox, function-based unit tests, tables, JSON, Excel, `exportgraphics`, Git.

**Spec:** `docs/superpowers/specs/2026-08-26-five-controller-control-study-design.md`

## Global Constraints

- Complete the legacy core, new controller, and optimization plans first.
- Keep the existing plant, attitude PID, and cascade PID Simulink models operational.
- Generate all SLX files through public builders and private creator functions.
- Use English block, signal, model, file, and logged variable names.
- Every model uses the same plant parameters, actuator limit, state order, reference and disturbance semantics, controller sample time, and maximum plant solver step.
- Differential-evolution training and Monte Carlo batches remain MATLAB numerical simulations; Simulink is for representative equivalence only.
- Formal statistics stop with a clear identifier if Statistics and Machine Learning Toolbox is absent.
- Version source, tests, SLX files, summaries, representative figures, and manifests; keep large Full raw results local.

---

## File map

- `config/validate_environment.m`: dependency, output, and Git preflight.
- `builders/build_fuzzy_pid_simulink.m`, `build_lqr_simulink.m`, `build_lqi_simulink.m`: public reproducible builders.
- `builders/private/create_fuzzy_pid_simulink_model.m`, `create_lqr_simulink_model.m`, `create_lqi_simulink_model.m`: model composition.
- `simulation/run_controller_simulink.m`: name-based Simulink execution adapter.
- `simulation/compare_matlab_simulink.m`: common-time numerical comparison.
- `experiments/run_simulink_validation_batch.m`: build and validate all five models.
- `evaluation/bootstrap_mean_ci.m`, `wilson_interval.m`, `cliffs_delta.m`, `holm_adjust.m`: deterministic statistical primitives.
- `evaluation/summarize_deterministic_results.m`, `summarize_monte_carlo_results.m`, `run_nonparametric_tests.m`: result tables.
- `visualization/plot_nominal_response.m`, `plot_saturation_response.m`, `plot_disturbance_recovery.m`, `plot_monte_carlo_boxplots.m`, `plot_performance_pareto.m`, `plot_normalized_radar.m`, `generate_paper_figures.m`: publication-oriented outputs.
- `reporting/save_raw_simulation.m`, `write_results_tables.m`, `build_run_manifest.m`: artifact export.
- `workflows/run_control_study_workflow.m`: orchestration behind the root wrapper.
- `run_control_study.m`: stable public entry point.
- `README.md`, `docs/IMPLEMENTATION_MAPPING.md`, `.gitignore`: usage, research mapping, and artifact policy.

### Task 1: Environment preflight

**Files:**
- Create: `config/validate_environment.m`
- Create: `tests/test_validate_environment.m`

**Interfaces:**
- Consumes: project root and logical `require_statistics`.
- Produces: a scalar report with MATLAB release, platform, toolbox availability, output writeability, Git commit, and `accepted`; throws stable errors for required missing dependencies.

- [ ] **Step 1: Write failing environment-report tests**

```matlab
function test_environment_report_has_required_fields(test_case)
root = string(fileparts(fileparts(mfilename("fullpath"))));
report = validate_environment(root, false);
verifyEqual(test_case, fieldnames(report), { ...
    'matlab_release'; 'platform'; 'simulink_available'; ...
    'control_available'; 'statistics_available'; ...
    'output_writable'; 'git_commit'; 'accepted'});
verifyTrue(test_case, report.simulink_available);
verifyTrue(test_case, report.control_available);
verifyTrue(test_case, report.output_writable);
verifyTrue(test_case, report.accepted);
end

function test_invalid_root_is_rejected(test_case)
verifyError(test_case, @() validate_environment("Z:/missing-srtp", false), ...
    "twsbr:environment:invalid_project_root");
end
```

- [ ] **Step 2: Run preflight tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_validate_environment.m'); assertSuccess(r)"
```

Expected: failure because `validate_environment` is undefined.

- [ ] **Step 3: Implement dependency and output checks**

Use `ver("simulink")`, `ver("control")`, and `ver("stats")`. Check output writeability by creating and closing a temporary file inside `results`, then deleting that exact file with cleanup. Read the Git commit with:

```matlab
[status, commit] = system(sprintf('git -C "%s" rev-parse HEAD', project_root));
if status ~= 0
    commit = "unavailable";
else
    commit = strtrim(string(commit));
end
```

Throw `twsbr:environment:simulink_required`, `twsbr:environment:control_required`, or `twsbr:environment:statistics_required` when a required component is absent. Set `accepted` only when root, Simulink, Control System Toolbox, and output checks pass, plus statistics when requested.

- [ ] **Step 4: Run the preflight tests**

Run the command from Step 2. Expected: all tests pass on the target MATLAB installation.

- [ ] **Step 5: Commit environment preflight**

```powershell
git add config/validate_environment.m tests/test_validate_environment.m
git commit -m "feat: add study environment preflight"
```

### Task 2: Programmatic LQR Simulink model

**Files:**
- Create: `builders/build_lqr_simulink.m`
- Create: `builders/private/create_lqr_simulink_model.m`
- Create: `tests/test_lqr_simulink_model.m`

**Interfaces:**
- Consumes: validated plant and decoded LQR parameters.
- Produces: absolute path `simulink_models/twsbr_lqr.slx` with logged state, reference, `u_raw`, applied input, and disturbances.

- [ ] **Step 1: Write a failing builder-structure test**

```matlab
function test_lqr_builder_creates_expected_model(test_case)
plant = twsbr_params();
config = experiment_config("quick");
params = decode_controller_vector("LQR", ...
    log10([10,1,200,10,0.1]), plant, config);
path_out = build_lqr_simulink(plant, params);
verifyTrue(test_case, isfile(path_out));
load_system(path_out);
cleanup = onCleanup(@() close_system("twsbr_lqr", 0));
required = ["position_reference"; "reference_sampling"; ...
    "state_sampling"; "state_error"; "lqr_gain"; ...
    "actuator_saturation"; "nonlinear_plant"; "state_log"; ...
    "u_raw_log"; "u_log"];
blocks = string(get_param(find_system("twsbr_lqr", ...
    "SearchDepth", 1, "Type", "Block"), "Name"));
verifyTrue(test_case, all(ismember(required, blocks)));
end
```

- [ ] **Step 2: Run the builder test and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_lqr_simulink_model.m'); assertSuccess(r)"
```

Expected: failure because the LQR builder is undefined.

- [ ] **Step 3: Implement public builder and private creator**

Follow the existing cascade builder lifecycle: validate inputs, require Simulink, ensure `twsbr_plant.slx`, close loaded target, delete only the exact target SLX, call the private creator, update, save, and close.

The private creator adds:

```matlab
add_block("simulink/Sources/From Workspace", model + "/position_reference", ...);
add_block("simulink/Discrete/Zero-Order Hold", model + "/reference_sampling", ...);
add_block("simulink/Discrete/Zero-Order Hold", model + "/state_sampling", ...);
add_block("simulink/Math Operations/Sum", model + "/state_error", ...
    "Inputs", "+-");
add_block("simulink/Math Operations/Gain", model + "/lqr_gain", ...
    "Gain", mat2str(-params.gain, 17), "Multiplication", "Matrix(K*u)");
add_block("simulink/Discontinuities/Saturation", ...
    model + "/actuator_saturation", ...);
add_block("twsbr_plant/nonlinear_plant", model + "/nonlinear_plant", ...);
```

Construct reference state `[x_reference;0;0;0]` with a Mux, name all connected lines, add force and torque From Workspace blocks, and add To Workspace logs in `Structure With Time` format. Match the existing variable-step `ode45`, relative tolerance, absolute tolerance, and maximum step settings.

- [ ] **Step 4: Run LQR and existing Simulink model tests**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_lqr_simulink_model.m','tests/test_cascade_pid_simulink_model.m','tests/test_twsbr_simulink.m'}); assertSuccess(r)"
```

Expected: all models build without warnings and tests pass.

- [ ] **Step 5: Commit the LQR model**

```powershell
git add builders/build_lqr_simulink.m builders/private/create_lqr_simulink_model.m simulink_models/twsbr_lqr.slx tests/test_lqr_simulink_model.m
git commit -m "feat: add generated LQR Simulink model"
```

### Task 3: Programmatic LQI Simulink model

**Files:**
- Create: `builders/build_lqi_simulink.m`
- Create: `builders/private/create_lqi_simulink_model.m`
- Create: `tests/test_lqi_simulink_model.m`

**Interfaces:**
- Consumes: plant and decoded LQI parameters.
- Produces: `simulink_models/twsbr_lqi.slx` with explicit position-error integrator and logs.

- [ ] **Step 1: Write a failing LQI block and parameter test**

```matlab
function test_lqi_model_contains_explicit_integral_loop(test_case)
plant = twsbr_params();
config = experiment_config("quick");
params = decode_controller_vector("LQI", ...
    log10([10,1,200,10,100,0.1]), plant, config);
path_out = build_lqi_simulink(plant, params);
verifyTrue(test_case, isfile(path_out));
load_system(path_out);
cleanup = onCleanup(@() close_system("twsbr_lqi", 0));
verifyEqual(test_case, get_param( ...
    "twsbr_lqi/position_error_integrator", "SampleTime"), ...
    sprintf("%.17g", config.sample_time));
verifyTrue(test_case, is_simulink_block_present( ...
    "twsbr_lqi", "integral_gain"));
verifyTrue(test_case, is_simulink_block_present( ...
    "twsbr_lqi", "controller_sum"));
end
```

- [ ] **Step 2: Run the LQI model test and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_lqi_simulink_model.m'); assertSuccess(r)"
```

Expected: failure because the LQI builder is undefined.

- [ ] **Step 3: Implement LQI builder with visible integral state**

Reuse the LQR model's source, sampling, plant, saturation, and logging blocks. Add:

```matlab
add_block("simulink/Math Operations/Sum", model + "/position_error", ...
    "Inputs", "+-");
add_block("simulink/Discrete/Discrete-Time Integrator", ...
    model + "/position_error_integrator", ...
    "gainval", sprintf("%.17g", params.sample_time), ...
    "SampleTime", sprintf("%.17g", params.sample_time), ...
    "LimitOutput", "on", ...
    "UpperSaturationLimit", sprintf("%.17g", params.position_integral_limit), ...
    "LowerSaturationLimit", sprintf("%.17g", -params.position_integral_limit));
add_block("simulink/Math Operations/Gain", model + "/state_gain", ...
    "Gain", mat2str(-params.state_gain,17), "Multiplication", "Matrix(K*u)");
add_block("simulink/Math Operations/Gain", model + "/integral_gain", ...
    "Gain", sprintf("%.17g", -params.integral_gain));
add_block("simulink/Math Operations/Sum", model + "/controller_sum", ...
    "Inputs", "++");
```

Log position error and position integral. Include a sampled reset input so repeated simulations clear the discrete integrator deterministically.

- [ ] **Step 4: Run LQI, LQR, and legacy builder tests**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_lqi_simulink_model.m','tests/test_lqr_simulink_model.m','tests/test_cascade_pid_simulink_model.m'}); assertSuccess(r)"
```

Expected: all tests pass.

- [ ] **Step 5: Commit the LQI model**

```powershell
git add builders/build_lqi_simulink.m builders/private/create_lqi_simulink_model.m simulink_models/twsbr_lqi.slx tests/test_lqi_simulink_model.m
git commit -m "feat: add generated LQI Simulink model"
```

### Task 4: Programmatic fuzzy PID Simulink model

**Files:**
- Create: `builders/build_fuzzy_pid_simulink.m`
- Create: `builders/private/create_fuzzy_pid_simulink_model.m`
- Create: `tests/test_fuzzy_pid_simulink_model.m`

**Interfaces:**
- Consumes: plant and decoded fuzzy PID parameters.
- Produces: `simulink_models/twsbr_fuzzy_pid.slx` with shared fixed-rule inference and logs for effective gains.

- [ ] **Step 1: Write a failing fuzzy model structure test**

```matlab
function test_fuzzy_model_logs_online_gains(test_case)
plant = twsbr_params();
config = experiment_config("quick");
vector = [log10([0.241,0.000396,0.193,9.255,0.05,1.011]), ...
    0.2,0.2,0.2];
params = decode_controller_vector("FUZZY_PID", vector, plant, config);
path_out = build_fuzzy_pid_simulink(plant, params);
verifyTrue(test_case, isfile(path_out));
load_system(path_out);
cleanup = onCleanup(@() close_system("twsbr_fuzzy_pid", 0));
required = ["fuzzy_controller"; "kp_theta_log"; ...
    "ki_theta_log"; "kd_theta_log"; "u_raw_log"; ...
    "actuator_saturation"; "nonlinear_plant"];
blocks = string(get_param(find_system("twsbr_fuzzy_pid", ...
    "SearchDepth",1,"Type","Block"), "Name"));
verifyTrue(test_case, all(ismember(required, blocks)));
end
```

- [ ] **Step 2: Run fuzzy model test and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_fuzzy_pid_simulink_model.m'); assertSuccess(r)"
```

Expected: failure because the fuzzy builder is undefined.

- [ ] **Step 3: Implement the sampled fuzzy controller subsystem**

Follow the existing cascade model layout and replace its controller MATLAB Function script with a code-generation-compatible function that maintains position and attitude integrals, computes the cascade outer loop, normalizes attitude error and rate, calls `fuzzy_pid_inference`, applies the decoded gain bounds, and returns:

```text
u_raw
position_error
theta_reference
theta_error
position_integral
theta_integral
kp_theta
ki_theta
kd_theta
```

The model includes sampled state, reference, and reset inputs; one common Saturation block; the existing nonlinear plant subsystem; force and torque From Workspace inputs; and named To Workspace logs for every listed output.

- [ ] **Step 4: Run fuzzy MATLAB and Simulink tests**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_fuzzy_pid_simulink_model.m','tests/test_fuzzy_pid_inference.m','tests/test_fuzzy_pid_step.m'}); assertSuccess(r)"
```

Expected: all tests pass and the model updates without warnings.

- [ ] **Step 5: Commit the fuzzy PID model**

```powershell
git add builders/build_fuzzy_pid_simulink.m builders/private/create_fuzzy_pid_simulink_model.m simulink_models/twsbr_fuzzy_pid.slx tests/test_fuzzy_pid_simulink_model.m
git commit -m "feat: add generated fuzzy PID Simulink model"
```

### Task 5: Five-controller Simulink execution and equivalence batch

**Files:**
- Create: `simulation/run_controller_simulink.m`
- Create: `simulation/compare_matlab_simulink.m`
- Create: `experiments/run_simulink_validation_batch.m`
- Create: `tests/test_simulink_equivalence.m`

**Interfaces:**
- Consumes: controller name, frozen vector, plant, study config, and representative generic scenario.
- Produces: common Simulink result schema and a five-row equivalence table.

- [ ] **Step 1: Write failing comparison and batch tests**

```matlab
function test_comparison_uses_frozen_thresholds(test_case)
matlab_result = make_small_common_result(0.0);
simulink_result = make_small_common_result(0.001);
comparison = compare_matlab_simulink(matlab_result, simulink_result, 0.001);
verifyEqual(test_case, comparison.tilt_tolerance_deg, 0.2);
verifyEqual(test_case, comparison.position_tolerance_m, 0.01);
verifyEqual(test_case, comparison.input_tolerance, 0.02);
verifyTrue(test_case, comparison.accepted);
end

function test_validation_batch_returns_all_controller_names(test_case)
[vectors, config] = starter_vectors_for_test();
table_out = run_simulink_validation_batch(vectors, twsbr_params(), config);
verifyEqual(test_case, height(table_out), 5);
verifyEqual(test_case, sort(table_out.controller), ...
    sort(config.controller_names));
verifyTrue(test_case, all(table_out.accepted));
end
```

- [ ] **Step 2: Run equivalence tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_simulink_equivalence.m'); assertSuccess(r)"
```

Expected: failure because the generic runner and comparison functions are undefined.

- [ ] **Step 3: Implement name-based runner, interpolation, and validation batch**

`run_controller_simulink` dispatches to existing legacy runners for `ATTITUDE_PID` and `CASCADE_PID`, and to the new builders plus `Simulink.SimulationInput` for the other names. Normalize logs into the same fields used by `simulate_control_system`.

`compare_matlab_simulink` creates common time from zero to the shorter final time at `plant_step`, interpolates state and applied input with `pchip`, and calculates maximum tilt degrees, position meters, and applied-input difference. It sets `accepted` only when all three strict inequalities pass.

`run_simulink_validation_batch` uses the `5 deg` initial-tilt scenario for attitude PID and the `0.5 m` position step for the other four controllers, runs MATLAB first, then Simulink, and appends one comparison record per controller. For fuzzy PID, also interpolate and compare effective gain diagnostics with relative tolerance `1e-6`.

- [ ] **Step 4: Run all Simulink and legacy tests**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_simulink_equivalence.m','tests/test_attitude_pid_simulink.m','tests/test_cascade_pid_simulink_equivalence.m','tests/test_twsbr_simulink_warnings.m'}); assertSuccess(r)"
```

Expected: every model and equivalence test passes.

- [ ] **Step 5: Commit common Simulink validation**

```powershell
git add simulation/run_controller_simulink.m simulation/compare_matlab_simulink.m experiments/run_simulink_validation_batch.m tests/test_simulink_equivalence.m
git commit -m "feat: validate five MATLAB and Simulink controllers"
```

### Task 6: Statistical summaries and nonparametric tests

**Files:**
- Create: `evaluation/bootstrap_mean_ci.m`
- Create: `evaluation/wilson_interval.m`
- Create: `evaluation/cliffs_delta.m`
- Create: `evaluation/holm_adjust.m`
- Create: `evaluation/summarize_deterministic_results.m`
- Create: `evaluation/summarize_monte_carlo_results.m`
- Create: `evaluation/run_nonparametric_tests.m`
- Create: `tests/test_statistics.m`

**Interfaces:**
- Consumes: deterministic, Monte Carlo, and complexity tables.
- Produces: deterministic summary, descriptive confidence-interval table, omnibus test table, and pairwise test table.

- [ ] **Step 1: Write failing primitive and successful-subset tests**

```matlab
function test_wilson_and_cliffs_delta_known_values(test_case)
[low, high] = wilson_interval(5, 10, 1.959963984540054);
verifyEqual(test_case, low, 0.2365930905, "AbsTol", 1e-9);
verifyEqual(test_case, high, 0.7634069095, "AbsTol", 1e-9);
verifyEqual(test_case, cliffs_delta([3,4], [1,2]), 1.0);
verifyEqual(test_case, cliffs_delta([1,2], [3,4]), -1.0);
end

function test_descriptive_statistics_condition_on_success(test_case)
data = table(["A";"A";"B";"B"], [true;false;true;true], ...
    [1;1000;2;4], 'VariableNames', ...
    {'controller','success','theta_rms_deg'});
summary = summarize_monte_carlo_results(data, "theta_rms_deg", 100, 7);
row_a = summary(summary.controller == "A", :);
verifyEqual(test_case, row_a.mean, 1);
verifyEqual(test_case, row_a.success_rate, 0.5);
verifyEqual(test_case, row_a.n, 1);
end
```

- [ ] **Step 2: Run statistics tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_statistics.m'); assertSuccess(r)"
```

Expected: failure because statistical functions are undefined.

- [ ] **Step 3: Implement statistical primitives and table builders**

Use local seeded resampling in `bootstrap_mean_ci`. Implement Wilson directly from the tested formula. Implement Cliff's delta from all pairwise comparisons. Implement Holm by sorting p-values ascending, multiplying by remaining hypotheses, enforcing a running maximum, clipping to one, and restoring original order.

For each controller and metric, summarize only rows where `success` is true, while computing success count and Wilson interval from every row. `run_nonparametric_tests` uses `kruskalwallis(values,groups,'off')` for omnibus tests and `ranksum(x,y)` for every controller pair, then appends Holm-adjusted p-values and Cliff's delta.

- [ ] **Step 4: Run statistics and metric tests**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_statistics.m','tests/test_control_metrics.m'}); assertSuccess(r)"
```

Expected: all tests pass.

- [ ] **Step 5: Commit statistical evaluation**

```powershell
git add evaluation/bootstrap_mean_ci.m evaluation/wilson_interval.m evaluation/cliffs_delta.m evaluation/holm_adjust.m evaluation/summarize_deterministic_results.m evaluation/summarize_monte_carlo_results.m evaluation/run_nonparametric_tests.m tests/test_statistics.m
git commit -m "feat: add robust controller statistics"
```

### Task 7: Publication-oriented figures

**Files:**
- Create: `visualization/plot_nominal_response.m`
- Create: `visualization/plot_saturation_response.m`
- Create: `visualization/plot_disturbance_recovery.m`
- Create: `visualization/plot_monte_carlo_boxplots.m`
- Create: `visualization/plot_performance_pareto.m`
- Create: `visualization/plot_normalized_radar.m`
- Create: `visualization/generate_paper_figures.m`
- Create: `tests/test_paper_figures.m`

**Interfaces:**
- Consumes: raw deterministic result lookup, Monte Carlo table, and figure directory.
- Produces: `F1` through `F6` in both PNG and PDF with fixed English labels and controller order.

- [ ] **Step 1: Write a failing artifact test**

```matlab
function test_generate_paper_figures_creates_png_and_pdf(test_case)
output = string(tempname);
mkdir(output);
cleanup = onCleanup(@() rmdir(output, "s"));
[raw, monte_carlo] = synthetic_figure_inputs();
paths = generate_paper_figures(raw, monte_carlo, output);
verifyEqual(test_case, numel(paths), 12);
verifyTrue(test_case, all(isfile(paths)));
verifyTrue(test_case, any(endsWith(paths, "F1_nominal_response.png")));
verifyTrue(test_case, any(endsWith(paths, "F6_normalized_radar.pdf")));
end
```

- [ ] **Step 2: Run figure tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_paper_figures.m'); assertSuccess(r)"
```

Expected: failure because figure generators are undefined.

- [ ] **Step 3: Implement six deterministic figure contracts**

Use fixed controller order `ATTITUDE_PID`, `CASCADE_PID`, `FUZZY_PID`, `LQR`, `LQI` and `Visible="off"` figures.

- `F1`: three stacked axes for position/reference, tilt degrees, and applied input in the `0.75 m` step.
- `F2`: one input axis per controller showing `u_raw`, `u`, and saturation periods in the saturation-stress scenario.
- `F3`: the same three stacked axes for the positive impulse scenario.
- `F4`: a `2 x 2` tiled layout of successful-trial boxcharts for position ITAE, tilt RMS, control energy, and saturation ratio.
- `F5`: median successful position ITAE versus control energy, with marker size `60 + 160*success_rate`.
- `F6`: min-max normalized median successful tilt RMS, position ITAE, control energy, and saturation ratio on polar axes, where one means best.

Each plot function returns its figure handle. `generate_paper_figures` calls `exportgraphics` once for 300-DPI PNG and once for vector PDF, closes each figure with cleanup, and returns all absolute paths.

- [ ] **Step 4: Run figure tests and check file sizes**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_paper_figures.m'); assertSuccess(r)"
```

Expected: all 12 files exist and each has nonzero size.

- [ ] **Step 5: Commit figure generation**

```powershell
git add visualization/plot_nominal_response.m visualization/plot_saturation_response.m visualization/plot_disturbance_recovery.m visualization/plot_monte_carlo_boxplots.m visualization/plot_performance_pareto.m visualization/plot_normalized_radar.m visualization/generate_paper_figures.m tests/test_paper_figures.m
git commit -m "feat: add five-controller paper figures"
```

### Task 8: Artifact reporting, manifest, and one-command workflow

**Files:**
- Create: `reporting/save_raw_simulation.m`
- Create: `reporting/write_results_tables.m`
- Create: `reporting/build_run_manifest.m`
- Create: `workflows/run_control_study_workflow.m`
- Create: `run_control_study.m`
- Create: `tests/test_control_study_outputs.m`
- Modify: `tests/test_root_entry_points.m`
- Modify: `README.md`
- Create: `docs/IMPLEMENTATION_MAPPING.md`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: mode, run-tests flag, and optional scalar options structure used by tests to provide frozen vectors and disable expensive formal stages.
- Produces: `summary = run_control_study(mode,run_tests_flag)` and the complete mode-specific result directory.

- [ ] **Step 1: Write failing reporting and root-entry tests**

```matlab
function test_reporters_create_expected_artifacts(test_case)
root = string(tempname);
mkdir(root); cleanup = onCleanup(@() rmdir(root,"s"));
[data, raw, vectors, validation] = synthetic_reporting_inputs();
paths = write_results_tables(root, data, data, data, data, data, validation);
manifest_path = build_run_manifest(root, experiment_config("quick"), ...
    objective_config(), vectors, "test-commit");
verifyTrue(test_case, isfile(paths.statistics_xlsx));
verifyTrue(test_case, isfile(paths.deterministic_csv));
verifyTrue(test_case, isfile(paths.monte_carlo_csv));
verifyTrue(test_case, isfile(manifest_path));
manifest = jsondecode(fileread(manifest_path));
verifyEqual(test_case, manifest.git_commit, "test-commit");
end

function test_root_wrapper_delegates_to_workflow(test_case)
summary = run_control_study("quick", false, ...
    struct("frozen_vectors", starter_vectors_for_test(), ...
    "run_monte_carlo", false, "run_simulink", false, ...
    "output_root", string(tempname)));
verifyEqual(test_case, summary.mode, "quick");
verifyTrue(test_case, isfolder(summary.output_root));
end
```

- [ ] **Step 2: Run output tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_control_study_outputs.m'); assertSuccess(r)"
```

Expected: failure because reporters and the root entry point are undefined.

- [ ] **Step 3: Implement raw, table, manifest, workflow, and wrapper functions**

`save_raw_simulation` saves one result to a sanitized `CONTROLLER__SCENARIO.mat` path and returns its relative index row. `write_results_tables` uses `writetable` for deterministic, Monte Carlo, descriptive, omnibus, pairwise, complexity, and equivalence CSV files, and writes each table to a named sheet in `statistics.xlsx`.

`build_run_manifest` records UTC creation time, `version`, `computer`, toolbox versions, Git commit, mode, seed, controller names, frozen vectors, and SHA-256 hashes of the three configuration files. Compute a hash with Java `MessageDigest` over `uint8(fileread(path))` and encode two-digit lowercase hexadecimal bytes.

`run_control_study_workflow` executes this fixed order:

```matlab
environment = validate_environment(project_root, options.require_statistics);
plant = twsbr_params();
config = experiment_config(mode);
train = training_scenarios(options.training_duration);
if isfield(options, "frozen_vectors")
    frozen = options.frozen_vectors;
    tuning_runs = table();
else
    tuning_runs = tune_all_controllers(plant, config, ...
        objective_config(), train, options.starter_vectors);
    frozen = select_frozen_parameters(tuning_runs);
end
[deterministic, raw] = run_deterministic_batch( ...
    frozen, plant, config, heldout_scenarios(options.test_duration), ...
    config.global_seed);
```

Then conditionally run Monte Carlo and Simulink based on options, always calculate complexity, generate available summaries, write artifacts, generate figures when required raw scenarios exist, build the manifest, and return every path and table in `summary`. The public wrapper calls `setup_project()` and delegates. It accepts the third options argument for deterministic tests while the documented user calls remain `run_control_study("quick",true)` and `run_control_study("full",true)`.

Add Full raw result directories and transient caches to `.gitignore`, but do not ignore source, tests, SLX, summary CSV, figures, or manifests.

- [ ] **Step 4: Run all tests, code analysis, and Quick workflow**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests'); assertSuccess(r); files=dir('**/*.m'); issues=[]; for k=1:numel(files), issues=[issues; checkcode(fullfile(files(k).folder,files(k).name),'-struct')]; end; assert(isempty(issues)); summary=run_control_study('quick',false); assert(isfile(summary.manifest_path))"
```

Expected: all tests pass, code analysis has no issues, Quick training and evaluation finish, all five models validate, and the expected output files exist.

- [ ] **Step 5: Document, commit, push, and verify remote state**

Update README with environment requirements, Quick and Full commands, five controller definitions, training/test isolation, output tree, control-energy wording, and software-simulation limitation. Map every research requirement to exact code files in `docs/IMPLEMENTATION_MAPPING.md`.

```powershell
git add .gitignore README.md docs/IMPLEMENTATION_MAPPING.md reporting workflows run_control_study.m tests/test_control_study_outputs.m tests/test_root_entry_points.m results/control_study_quick simulink_models
git commit -m "feat: deliver unified five-controller study"
git push origin main
git status --short
git rev-parse HEAD
git rev-parse origin/main
```

Expected: status is empty and local `HEAD` exactly equals `origin/main`.

## Completion gate

Do not claim completion until the full legacy and new test suite passes, Quick mode produces all expected artifacts, five MATLAB/Simulink comparisons are accepted, MATLAB code analysis is clean, and the pushed remote commit equals local `HEAD`.
