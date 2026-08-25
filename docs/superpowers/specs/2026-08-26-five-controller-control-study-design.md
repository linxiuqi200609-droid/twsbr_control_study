# Five Controller Unified Control Study Design

## 1. Purpose

This document defines the approved architecture for extending the existing two-wheeled self-balancing robot MATLAB and Simulink project into a reproducible five-controller comparative study.

The five controllers are:

1. `ATTITUDE_PID`: the existing basic attitude PID, used as a limited-capability control group.
2. `CASCADE_PID`: the existing position PID and attitude PD cascade.
3. `FUZZY_PID`: a position PID outer loop and fuzzy self-tuning attitude PID inner loop.
4. `LQR`: discrete full-state linear quadratic regulation.
5. `LQI`: discrete linear quadratic integral control with position-error integration.

The study follows the reference project's reproducible pipeline pattern: isolated training and test scenarios, equal differential-evolution evaluation budgets, deterministic and Monte Carlo evaluation, statistical summaries, publication-oriented figures, run manifests, and representative MATLAB/Simulink equivalence checks.

All results are nonlinear software simulations. They are not physical-vehicle experiments or evidence of hardware deployment safety.

## 2. Approved scope

### 2.1 Required outcomes

- Preserve the existing nonlinear plant, basic attitude PID, cascade PID, entry points, results, and Simulink models.
- Connect the existing plant and two existing controllers to a unified numerical simulation pipeline without changing their mathematical behavior.
- Implement MATLAB numerical and Simulink versions of fuzzy PID, LQR, and LQI.
- Train all five controllers with the same differential-evolution algorithm, training scenarios, random-seed set, objective definition, failure policy, and exact function-evaluation budget.
- Freeze parameters using training results only.
- Evaluate frozen parameters on held-out deterministic and paired Monte Carlo scenarios.
- Export raw data, summary tables, statistics, figures, Simulink equivalence results, and a reproducibility manifest.
- Use English file and directory names that follow MATLAB naming rules.
- Record implementation phases in Git and push them to the configured GitHub remote.

### 2.2 Explicit non-goals

- No steering, yaw, or lateral dynamics.
- No claim of real-vehicle validation, measured electrical energy, or safe hardware deployment.
- No training of fuzzy membership-function shapes or rule-table entries in the first implementation.
- No assumption that fuzzy PID, LQR, or LQI must outperform cascade PID.
- No Simulink-in-the-loop differential-evolution training; repeated training and batch tests use the MATLAB numerical simulator.
- No unrelated restructuring of the working legacy entry points.

## 3. Architecture

The selected architecture is a unified MATLAB numerical simulation core with five controller adapters and five Simulink validation models.

```text
Quick or Full configuration
    -> environment validation
    -> training scenarios
    -> exact-budget differential evolution
    -> frozen controller parameters
    -> held-out deterministic scenarios
    -> paired Monte Carlo scenarios
    -> metrics and statistical analysis
    -> paper figures and result tables
    -> representative MATLAB/Simulink equivalence checks
    -> run manifest
```

The numerical simulator owns the nonlinear plant integration, scenario signals, measurement noise, actuator saturation, failure detection, timing, and logging. Controllers return an unsaturated command and diagnostics. This prevents controller-specific implementations from changing shared experimental conditions.

## 4. Plant, state, input, and timing contract

The existing plant implementation remains the single source of truth:

```text
models/twsbr_params.m
models/twsbr_dynamics.m
models/twsbr_rk4_step.m
models/twsbr_linear_model.m
```

No duplicate nonlinear dynamics equations will be introduced.

The state order remains:

```text
[x; x_dot; theta; theta_dot]
```

The upright equilibrium is `theta = 0`. The normalized controller command is converted to equivalent horizontal force through the existing `motor_force_gain`. The shared applied command is:

```matlab
u = min(max(u_raw, -plant_params.u_max), plant_params.u_max);
```

The timing contract is:

```text
controller sample time = 0.01 s
plant integration step = 0.001 s
plant integrator        = existing fixed-step RK4
control hold            = 10 plant steps per controller update
```

Scenario signals, controller updates, logging, failure checks, and plant steps use one documented order that reproduces the existing simulator order.

## 5. Controller definitions

### 5.1 Basic attitude PID

The existing `attitude_pid_step.m` remains unchanged:

```text
theta_error = theta - theta_reference
u_raw = Kp * theta_error + Ki * integral_error + Kd * theta_dot
```

It uses only body tilt and angular rate. In the unified study, `theta_reference` is zero. Position reference is intentionally ignored by the controller, but position error, drift, and task failure are still recorded. This controller is the limited-capability control group.

The trained vector has three log-parameterized positive gains:

```text
log10(Kp) in [-1.0, 2.0]
log10(Ki) in [-4.0, 1.0]
log10(Kd) in [-2.0, 1.5]
```

The actuator limit, integral limit, sample time, and plant step are fixed configuration values rather than trained variables.

### 5.2 Cascade PID

The existing `cascade_pid_step.m` remains unchanged. Its position PID outer loop produces a bounded attitude reference, and its attitude PD inner loop produces `u_raw`.

The trained vector has five log-parameterized gains:

```text
log10(Kp_x)     in [-2.0, 0.5]
log10(Ki_x)     in [-4.0, 0.0]
log10(Kd_x)     in [-2.0, 1.0]
log10(Kp_theta) in [ 0.0, 2.0]
log10(Kd_theta) in [-1.5, 1.5]
```

The attitude-reference limit remains fixed at `12 deg`.

### 5.3 Fuzzy cascade PID

The outer loop is a conventional position PID. The inner loop is a fuzzy self-tuning attitude PID.

The fuzzy inference inputs are normalized attitude error and normalized error-change rate. The outputs are normalized adjustments `delta_kp`, `delta_ki`, and `delta_kd`. Online gains are:

```text
Kp = Kp0 * (1 + alpha_p * delta_kp)
Ki = Ki0 * (1 + alpha_i * delta_ki)
Kd = Kd0 * (1 + alpha_d * delta_kd)
```

The input normalization ranges are fixed physical configuration values. Online gains are clipped to fixed positive safety ranges. The fuzzy system uses seven symmetric linguistic variables:

```text
NB, NM, NS, ZO, PS, PM, PB
```

It uses fixed symmetric triangular membership functions, fixed `7 x 7` rule matrices, product or minimum implication selected once in configuration, weighted-average defuzzification, and no Fuzzy Logic Toolbox dependency. MATLAB numerical simulation and Simulink call the same code-generation-compatible fuzzy inference function.

The trained vector has nine variables:

```text
outer position Kp, Ki, Kd       3 variables
inner base Kp0, Ki0, Kd0        3 variables
alpha_p, alpha_i, alpha_d       3 variables
```

The six positive controller gains use log parameterization. Each adjustment ratio is bounded linearly in `[0, 1.5]`. Membership functions, normalization ranges, rule matrices, and gain safety limits are frozen before training.

### 5.4 Discrete LQR

The existing continuous analytical `A` and `B` matrices are discretized with zero-order hold at `0.01 s`. The discrete Riccati equation gives the full-state feedback gain `K`.

```matlab
error_state = state - [x_reference; 0; 0; 0];
u_raw = -K * error_state;
```

The trained vector has five log-parameterized variables:

```text
log10(q_x)         in [-2.0, 4.0]
log10(q_x_dot)     in [-2.0, 4.0]
log10(q_theta)     in [-2.0, 5.0]
log10(q_theta_dot) in [-2.0, 4.0]
log10(r)           in [-3.0, 2.0]
```

Every candidate must yield a finite stabilizing Riccati solution and closed-loop discrete eigenvalues strictly inside the unit circle. Invalid candidates receive the invalid penalty.

### 5.5 Discrete LQI

LQI augments the four-state discrete plant with the position-error integral:

```text
z(k+1) = z(k) + sample_time * (x - x_reference)
u_raw  = -Kx * state - Ki * z
```

The trained vector has six log-parameterized variables:

```text
log10(q_x)          in [-2.0, 4.0]
log10(q_x_dot)      in [-2.0, 4.0]
log10(q_theta)      in [-2.0, 5.0]
log10(q_theta_dot)  in [-2.0, 4.0]
log10(q_integral)   in [-2.0, 5.0]
log10(r)            in [-3.0, 2.0]
```

Plain LQI has no additional anti-windup law in this comparison. The integral state receives a large finite numerical-safety limit. The shared actuator block applies saturation. Invalid or non-stabilizing Riccati solutions receive the invalid penalty.

## 6. Unified controller interface

The numerical simulator interacts with all controllers through:

```matlab
controller = create_controller(controller_name, vector, ...
    plant_params, experiment_config);
controller = reset_controller(controller);
[control, controller] = controller_step( ...
    controller, time, measured_state, x_reference);
controller = controller_after_actuation( ...
    controller, control.u_raw, applied_u);
```

`control` always contains finite scalar `u_raw` and a diagnostics structure. Controller-specific states remain inside `controller`.

The supported names are fixed:

```matlab
["ATTITUDE_PID", "CASCADE_PID", "FUZZY_PID", "LQR", "LQI"]
```

## 7. Legacy compatibility gate

Existing behavior is protected before new controllers or optimization are accepted.

The following implementations are not rewritten:

```text
models/twsbr_params.m
models/twsbr_dynamics.m
models/twsbr_rk4_step.m
controllers/attitude_pid_params.m
controllers/attitude_pid_step.m
controllers/cascade_pid_params.m
controllers/cascade_pid_step.m
```

The unified layer adds:

```text
controllers/adapt_attitude_pid_controller.m
controllers/adapt_cascade_pid_controller.m
```

The adapters call the existing step functions, expose their `u_raw`, preserve their controller-state updates, and forward diagnostics. The numerical simulator applies the common saturation. For both existing controllers, the common saturated command must equal the legacy step function's saturated command because both use the same `u_max`.

Optimized vectors pass through `decode_controller_vector.m` and become structures compatible with the existing step functions. Existing hand-tuned parameters remain available as legacy defaults, Quick-mode starter candidates, and regression baselines.

The compatibility gate compares the legacy and unified simulations under identical parameters and scenarios. Required agreement is:

```text
maximum time difference        <= 1e-14 s
maximum state difference       <= 1e-12
maximum raw control difference <= 1e-12
maximum applied input error    <= 1e-12
saturation flags               exactly equal
success and failure reason     exactly equal
```

If proven floating-point operation ordering prevents `1e-12`, the tolerance may be relaxed only to a documented machine-precision-level value. The wider MATLAB/Simulink tolerances may not replace this regression gate.

The following public interfaces and artifacts remain valid:

```text
run_project.m
run_attitude_pid.m
run_cascade_pid.m
simulate_attitude_pid.m
simulate_cascade_pid.m
twsbr_plant.slx
twsbr_attitude_pid.slx
twsbr_cascade_pid.slx
```

## 8. Training and test scenarios

### 8.1 Training-only scenarios

Only these scenarios participate in differential-evolution objectives:

| Name | Definition |
|---|---|
| `T1_initial_tilt_5deg` | `5 deg` initial tilt, zero position reference |
| `T2_position_step_0p5m` | position reference steps to `0.5 m` at `1 s` |
| `T3_impulse_disturbance` | `2 deg` initial tilt and `5 N` force for `0.2 s` starting at `3 s` |

The attitude PID runs all three scenarios and receives the same position-related objective terms even though it cannot use position reference.

### 8.2 Held-out deterministic scenarios

The held-out set contains:

- initial tilts of `3, 6, 9, 12 deg`;
- position steps of `0.25, 0.75, 1.0 m`;
- an `8 deg` initial tilt, `1.0 m` position command, and `10 N` saturation-stress impulse;
- positive and negative force impulses;
- a body-torque impulse;
- a constant force bias.

Test scenarios never enter tuning, parameter selection, or objective scaling. Parameters remain frozen throughout final evaluation.

### 8.3 Paired Monte Carlo scenario

For each Monte Carlo index, one random scenario and one perturbed plant are generated and reused by all five controllers. Paired data include:

- body-mass, center-of-mass-length, and body-inertia perturbations;
- initial tilt;
- position-reference amplitude;
- impulse sign, amplitude, start time, and duration;
- constant force bias;
- the complete measurement-noise sequence.

Controller design always uses nominal parameters. Only the simulated plant receives physical-parameter perturbations.

## 9. Objective and failure policy

For successful simulations, the normalized training objective is:

```text
J = 0.25 * theta_itae          / 0.8
  + 0.25 * position_itae       / 8.0
  + 0.15 * control_energy      / 4.0
  + 0.15 * saturation_time     / 1.0
  + 0.10 * max_abs_theta_rad   / deg2rad(10)
  + 0.10 * recovery_time       / 3.0
```

The recovery term is zero for scenarios without a defined disturbance end. The objective is averaged across the three training scenarios.

The failure penalty is:

```text
500 * (2 - survived_time / scenario_duration)
```

Completely invalid vectors return `1e6`.

A simulation fails when any state or control becomes nonfinite, body tilt exceeds `30 deg`, position exceeds `5 m`, controller evaluation fails, plant integration fails, or the run terminates early. Every failure remains in raw and summary outputs.

Task success additionally checks the final `0.5 s` window: mean absolute tilt no greater than `3 deg`, mean speed no greater than `0.25 m/s`, and mean absolute position error within `max(0.10 m, 20% of the nonzero target)`. For zero position reference, the position tolerance is `0.30 m` so attitude stabilization can be distinguished from uncontrolled drift.

## 10. Differential-evolution fairness

The project implements an exact-budget differential-evolution routine without requiring Global Optimization Toolbox.

All controllers use the same:

- training scenarios;
- population size;
- exact number of objective evaluations;
- seed set;
- mutation and crossover policy;
- failure penalties;
- nonlinear simulator;
- actuator limit;
- stopping rule.

The initial population includes the available starter vector and bounded randomized candidates. Search variables use the parameterizations in Section 5.

Quick mode uses:

```text
population size       24
evaluation budget     240
tuning seeds          [0]
Monte Carlo runs      10
```

Full mode uses:

```text
population size       40
evaluation budget     3200
tuning seeds          [0,1,2,3,4,5,6,7,8,9]
Monte Carlo runs      200
```

The selected frozen vector is the lowest training objective among independent seeds. All runs remain in the tuning table. The final report includes parameter dimension, actual evaluations, training wall time, control-step runtime, and final training objective so equal computational budgets are auditable.

## 11. Metrics and statistics

Common metrics include:

```text
success
failure_reason
survived_time
theta_rms_deg
max_abs_theta_deg
theta_itae
attitude_settling_time
position_itae
final_abs_position_error
position_settling_time
position_overshoot
position_drift
control_energy
saturation_time
saturation_ratio
disturbance_recovery_time
controller_runtime_seconds
mean_step_runtime_us
```

Position metrics remain defined for the attitude PID and expose its inability to track commanded position.

Continuous Monte Carlo metrics are summarized only over task-successful trials. Overall success rate and its Wilson interval use all trials. Statistical outputs include means, medians, standard deviations, quartiles, bootstrap mean confidence intervals, Kruskal-Wallis tests, Mann-Whitney pairwise tests, Holm correction, and Cliff's delta.

`control_energy` means the integral of squared normalized applied input and is a control-cost proxy, not measured battery energy.

## 12. MATLAB and Simulink strategy

Five closed-loop Simulink models are required:

```text
simulink_models/twsbr_attitude_pid.slx
simulink_models/twsbr_cascade_pid.slx
simulink_models/twsbr_fuzzy_pid.slx
simulink_models/twsbr_lqr.slx
simulink_models/twsbr_lqi.slx
```

The first two remain compatible with their current builders. New builders create the last three programmatically with English block and signal names.

Every model shares plant parameters, state ordering, actuator limits, controller sample time, plant step, disturbance inputs, state logging, and failure limits.

The fuzzy model uses a code-generation-compatible MATLAB Function subsystem calling the shared fixed-rule fuzzy inference. The LQR model uses the frozen state-feedback gain. The LQI model adds a discrete position-error integrator and the frozen augmented gain.

Representative equivalence scenarios are:

- `5 deg` initial tilt for attitude PID;
- `0.5 m` position step for cascade PID, fuzzy PID, LQR, and LQI.

Equivalence acceptance is:

```text
maximum tilt difference       < 0.2 deg
maximum position difference   < 0.01 m
maximum applied input error   < 0.02
```

Fuzzy PID validation also compares online gain trajectories.

## 13. Project structure

The existing directories remain. The following public files and directories are added or extended.

```text
run_control_study.m
config/
    validate_environment.m
    experiment_config.m
    objective_config.m
    monte_carlo_config.m
models/
    twsbr_discrete_model.m
    twsbr_augmented_lqi_model.m
controllers/
    adapt_attitude_pid_controller.m
    adapt_cascade_pid_controller.m
    fuzzy_pid_rule_base.m
    fuzzy_pid_inference.m
    fuzzy_pid_step.m
    lqr_step.m
    lqi_step.m
    create_controller.m
    reset_controller.m
    controller_step.m
    controller_after_actuation.m
    controller_parameter_space.m
    decode_controller_vector.m
scenarios/
    training_scenarios.m
    heldout_scenarios.m
    generate_monte_carlo_scenario.m
simulation/
    simulate_control_system.m
    run_controller_simulink.m
    compare_matlab_simulink.m
optimization/
    differential_evolution.m
    control_objective.m
    tune_controller.m
    tune_all_controllers.m
    select_frozen_parameters.m
experiments/
    run_deterministic_batch.m
    run_monte_carlo.m
    benchmark_controllers.m
    run_simulink_validation_batch.m
evaluation/
    calculate_control_metrics.m
    summarize_deterministic_results.m
    summarize_monte_carlo_results.m
    bootstrap_mean_ci.m
    wilson_interval.m
    cliffs_delta.m
    holm_adjust.m
    run_nonparametric_tests.m
visualization/
    generate_paper_figures.m
    plot_nominal_response.m
    plot_saturation_response.m
    plot_disturbance_recovery.m
    plot_monte_carlo_boxplots.m
    plot_performance_pareto.m
    plot_normalized_radar.m
reporting/
    save_raw_simulation.m
    write_results_tables.m
    build_run_manifest.m
workflows/
    run_control_study_workflow.m
builders/
    build_fuzzy_pid_simulink.m
    build_lqr_simulink.m
    build_lqi_simulink.m
builders/private/
    create_fuzzy_pid_simulink_model.m
    create_lqr_simulink_model.m
    create_lqi_simulink_model.m
```

`setup_project.m` adds only the public code directories through its explicit allowlist. It does not add tests, results, documentation, generated models, caches, or builder-private code.

## 14. Output structure

Quick and Full modes use separate output roots. A Full output has:

```text
results/control_study_full/
    tuning/
        tuning_runs.mat
        tuning_runs.csv
        frozen_controller_parameters.json
    deterministic/
        raw/
        summary_metrics.csv
        controller_parameters.csv
        controller_complexity.csv
    monte_carlo/
        monte_carlo_metrics.csv
    simulink_validation/
        equivalence_summary.csv
    figures/
        F1_nominal_response.png
        F2_saturation_response.png
        F3_disturbance_recovery.png
        F4_monte_carlo_boxplots.png
        F5_performance_pareto.png
        F6_normalized_radar.png
    statistics.xlsx
    statistics_descriptive.csv
    statistics_omnibus.csv
    statistics_pairwise.csv
    training_scenarios.json
    heldout_scenarios.json
    run_manifest.json
```

Figures are exported in PNG and PDF. Full raw MAT and Monte Carlo intermediate data remain local by default. Source, tests, documentation, generated SLX files, Quick summaries, statistical summaries, representative figures, and the manifest are suitable for Git versioning.

## 15. Error handling and environment validation

`validate_environment.m` checks MATLAB, Simulink, Control System Toolbox, Statistics and Machine Learning Toolbox, output-directory write access, and Git metadata availability.

The fuzzy system and differential evolution do not require Fuzzy Logic Toolbox or Global Optimization Toolbox. LQR and LQI require Control System Toolbox. The formal nonparametric statistics phase requires Statistics and Machine Learning Toolbox; its absence stops the formal pipeline with a clear diagnostic instead of silently omitting results.

Each public function validates types, shapes, finiteness, bounds, state order, scenario metadata, timing ratios, and output paths. Expected numerical failures become structured failed trials. Programming or configuration errors retain stable identifiers and stop the workflow.

## 16. Verification and acceptance

Unit tests cover discrete models, controller factory behavior, fuzzy membership and symmetry, fuzzy gain bounds, LQR stability, LQI augmented controllability, exact-budget deterministic differential evolution, objective penalties, metrics, and scenario isolation.

Integration tests cover Quick tuning, deterministic batches, paired Monte Carlo data, five programmatic Simulink builders, MATLAB/Simulink equivalence, output artifacts, legacy equivalence, and legacy entry points.

Required functional acceptance is:

- all existing and new tests pass;
- MATLAB code analysis reports no errors;
- the legacy compatibility gate passes before new-controller integration;
- attitude PID stabilizes the nominal initial-tilt scenario;
- cascade PID, fuzzy PID, LQR, and LQI complete the nominal `0.5 m` position scenario;
- all applied commands remain inside the shared limit;
- fuzzy online gains remain finite and bounded;
- LQR and LQI produce finite stabilizing Riccati solutions;
- all five Simulink models rebuild without warnings and pass equivalence thresholds;
- Quick mode is reproducible for a fixed seed;
- every expected MAT, CSV, JSON, XLSX, PNG, PDF, and SLX artifact is created.

Controller rankings and claims of superiority are not acceptance criteria.

## 17. Implementation sequence

1. Freeze current plant, attitude PID, cascade PID, entry-point, and artifact baselines.
2. Add environment validation, configuration, unified scenarios, and result schema.
3. Connect the existing plant to the unified simulator.
4. Add legacy controller adapters and pass plant, attitude PID, and cascade PID equivalence gates.
5. Re-run all legacy tests and entry points.
6. Add discrete plant and augmented LQI models with tests.
7. Add LQR and LQI controllers with tests.
8. Add fixed-rule fuzzy inference and fuzzy cascade PID with tests.
9. Add common metrics and failure policy with tests.
10. Add exact-budget differential evolution and objective evaluation with tests.
11. Complete Quick training for all five controllers and freeze starter results.
12. Add deterministic and paired Monte Carlo batches.
13. Build fuzzy PID, LQR, and LQI Simulink models.
14. Validate all five MATLAB/Simulink implementations.
15. Add statistics, tables, figures, and run manifest.
16. Run the complete legacy and unified verification suite.
17. Update README and research mapping documentation.
18. Commit each coherent phase and push it to the configured GitHub remote.

Implementation must not proceed past the legacy compatibility gate if the existing plant or either existing controller changes behavior.
