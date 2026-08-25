# Unified Legacy Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Connect the existing nonlinear plant, basic attitude PID, and cascade PID to one common numerical simulation interface while preserving legacy behavior to machine-precision tolerances.

**Architecture:** Add configuration, generic scenarios, parameter decoding, controller adapters, and a fixed-step simulator around the existing plant and controller functions. Existing public entry points and mathematical implementations remain unchanged; regression tests form a hard gate for later controllers.

**Tech Stack:** MATLAB function-based unit tests, fixed-step RK4, existing nonlinear plant and PID functions, Git.

**Spec:** `docs/superpowers/specs/2026-08-26-five-controller-control-study-design.md`

## Global Constraints

- State order is exactly `[x; x_dot; theta; theta_dot]`.
- Controller sample time is `0.01 s`; plant step is `0.001 s`.
- The applied command is clipped once to `[-plant_params.u_max, plant_params.u_max]` by the common simulator.
- Do not modify the mathematics in `twsbr_dynamics.m`, `twsbr_rk4_step.m`, `attitude_pid_step.m`, or `cascade_pid_step.m`.
- Keep `run_project.m`, `run_attitude_pid.m`, and `run_cascade_pid.m` working.
- Add only public source directories to the explicit `setup_project.m` allowlist; never use `genpath`.
- Use English MATLAB-valid file, function, variable, block, and signal names.
- Write the failing test before each implementation and commit only after relevant tests pass.

---

## File map

- `config/experiment_config.m`: return immutable Quick or Full study configuration.
- `scenarios/training_scenarios.m`: return the three generic training scenarios.
- `controllers/controller_parameter_space.m`: describe names and bounds for controller vectors.
- `controllers/decode_controller_vector.m`: convert vectors into validated legacy parameter structures.
- `controllers/adapt_attitude_pid_controller.m`: call the existing attitude PID step through the common interface.
- `controllers/adapt_cascade_pid_controller.m`: call the existing cascade PID step through the common interface.
- `controllers/create_controller.m`: construct a controller structure from a name and vector.
- `controllers/reset_controller.m`: reset controller state deterministically.
- `controllers/controller_step.m`: dispatch one sampled controller update.
- `controllers/controller_after_actuation.m`: validate common saturation and commit pending state.
- `simulation/simulate_control_system.m`: run the common nonlinear fixed-step simulation and return the common result schema.
- `setup_project.m`: add `config`, `optimization`, `experiments`, `evaluation`, and `reporting` to the public allowlist as those directories appear.
- `tests/test_experiment_config.m`: verify configuration and path behavior.
- `tests/test_controller_parameterization.m`: verify bounds and legacy vector decoding.
- `tests/test_legacy_controller_adapters.m`: verify adapter outputs and state updates.
- `tests/test_unified_plant_equivalence.m`: verify the common simulator uses the existing RK4 plant exactly.
- `tests/test_unified_attitude_pid_equivalence.m`: compare legacy and common attitude PID trajectories.
- `tests/test_unified_cascade_pid_equivalence.m`: compare legacy and common cascade PID trajectories.
- `tests/test_legacy_entry_points_after_integration.m`: verify the three original root workflows remain usable.

### Task 1: Public directories and immutable experiment configuration

**Files:**
- Create: `config/experiment_config.m`
- Create: `tests/test_experiment_config.m`
- Modify: `setup_project.m`
- Modify: `tests/test_project_paths.m`

**Interfaces:**
- Consumes: existing `setup_project()` and project root layout.
- Produces: `config = experiment_config(mode)` where `mode` is `"quick"` or `"full"`; `config` contains timing, controller names, training budgets, Monte Carlo count, seed list, and limits.

- [ ] **Step 1: Write failing configuration and path tests**

```matlab
function tests = test_experiment_config
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function teardownOnce(test_case)
path(test_case.TestData.original_path);
end

function test_quick_configuration_is_frozen(test_case)
config = experiment_config("quick");
verifyEqual(test_case, config.mode, "quick");
verifyEqual(test_case, config.controller_names, ...
    ["ATTITUDE_PID"; "CASCADE_PID"; "FUZZY_PID"; "LQR"; "LQI"]);
verifyEqual(test_case, config.sample_time, 0.01, "AbsTol", 1e-15);
verifyEqual(test_case, config.plant_step, 0.001, "AbsTol", 1e-15);
verifyEqual(test_case, config.population_size, 24);
verifyEqual(test_case, config.evaluation_budget, 240);
verifyEqual(test_case, config.tuning_seeds, 0);
verifyEqual(test_case, config.monte_carlo_runs, 10);
end

function test_full_configuration_is_frozen(test_case)
config = experiment_config("full");
verifyEqual(test_case, config.population_size, 40);
verifyEqual(test_case, config.evaluation_budget, 3200);
verifyEqual(test_case, config.tuning_seeds, 0:9);
verifyEqual(test_case, config.monte_carlo_runs, 200);
end

function test_unknown_mode_is_rejected(test_case)
verifyError(test_case, @() experiment_config("draft"), ...
    "twsbr:study:invalid_mode");
end
```

Extend `test_project_paths.m` so the expected public directory allowlist is:

```matlab
directory_names = ["models"; "controllers"; "simulation"; "scenarios"; ...
    "builders"; "visualization"; "workflows"; "config"; ...
    "optimization"; "experiments"; "evaluation"; "reporting"];
```

- [ ] **Step 2: Run the focused tests and verify failure**

Run:

```powershell
matlab -batch "cd('D:/Research/srtp'); r=runtests({'tests/test_experiment_config.m','tests/test_project_paths.m'}); assertSuccess(r)"
```

Expected: failure because `experiment_config` does not exist and the new public directories are not in the allowlist.

- [ ] **Step 3: Implement the configuration and path allowlist**

Create `experiment_config.m` with this public structure:

```matlab
function config = experiment_config(mode)
mode = lower(string(mode));
if ~isscalar(mode) || ~any(mode == ["quick", "full"])
    error("twsbr:study:invalid_mode", ...
        "mode must be either quick or full.");
end
config = struct();
config.mode = mode;
config.controller_names = ["ATTITUDE_PID"; "CASCADE_PID"; ...
    "FUZZY_PID"; "LQR"; "LQI"];
config.sample_time = 0.01;
config.plant_step = 0.001;
config.theta_reference_limit = deg2rad(12);
config.position_integral_limit = 1e6;
config.fuzzy_gain_adjustment_limit = 1.5;
config.global_seed = 2026;
if mode == "quick"
    config.population_size = 24;
    config.evaluation_budget = 240;
    config.tuning_seeds = 0;
    config.monte_carlo_runs = 10;
    config.bootstrap_resamples = 100;
    config.benchmark_repeats = 1000;
else
    config.population_size = 40;
    config.evaluation_budget = 3200;
    config.tuning_seeds = 0:9;
    config.monte_carlo_runs = 200;
    config.bootstrap_resamples = 2000;
    config.benchmark_repeats = 50000;
end
end
```

Modify the `directory_names` array in `setup_project.m` to match the tested allowlist. Do not add nonexistent directories to `paths.code_directories`; retain the existing `isfolder` filter.

- [ ] **Step 4: Run focused and existing path tests**

Run the command from Step 2. Expected: all tests pass and `paths.missing_code_directories` lists only public directories not created yet.

- [ ] **Step 5: Commit the configuration contract**

```powershell
git add setup_project.m config/experiment_config.m tests/test_experiment_config.m tests/test_project_paths.m
git commit -m "feat: add unified study configuration"
```

### Task 2: Generic training scenarios and legacy parameter decoding

**Files:**
- Create: `scenarios/training_scenarios.m`
- Create: `controllers/controller_parameter_space.m`
- Create: `controllers/decode_controller_vector.m`
- Create: `tests/test_controller_parameterization.m`
- Create: `tests/test_training_scenarios.m`

**Interfaces:**
- Consumes: `experiment_config(mode)`, `twsbr_params()`, `attitude_pid_params()`, and `cascade_pid_params()`.
- Produces: `scenarios = training_scenarios(duration)` as a scalar structure with fields `T1_initial_tilt_5deg`, `T2_position_step_0p5m`, and `T3_impulse_disturbance`; `space = controller_parameter_space(name)`; `params = decode_controller_vector(name, vector, plant_params, config)`.

- [ ] **Step 1: Write failing scenario and parameter tests**

```matlab
function test_training_scenarios_are_exact_and_marked_train(test_case)
scenarios = training_scenarios(8.0);
verifyEqual(test_case, fieldnames(scenarios), ...
    {'T1_initial_tilt_5deg'; 'T2_position_step_0p5m'; ...
    'T3_impulse_disturbance'});
verifyEqual(test_case, scenarios.T1_initial_tilt_5deg.initial_state, ...
    [0; 0; deg2rad(5); 0], "AbsTol", 1e-15);
verifyEqual(test_case, scenarios.T2_position_step_0p5m.x_reference(0.9), 0);
verifyEqual(test_case, scenarios.T2_position_step_0p5m.x_reference(1.0), 0.5);
verifyEqual(test_case, scenarios.T3_impulse_disturbance.force_disturbance(3.1), 5);
verifyEqual(test_case, scenarios.T3_impulse_disturbance.force_disturbance(3.2), 0);
verifyTrue(test_case, all(structfun(@(s) s.split == "train", scenarios)));
end

function test_legacy_parameter_spaces_and_decoding(test_case)
plant = twsbr_params();
config = experiment_config("quick");
attitude_space = controller_parameter_space("ATTITUDE_PID");
cascade_space = controller_parameter_space("CASCADE_PID");
verifyEqual(test_case, attitude_space.dimension, 3);
verifyEqual(test_case, cascade_space.dimension, 5);
attitude = decode_controller_vector("ATTITUDE_PID", ...
    log10([1.9, 0.2, 0.18]), plant, config);
cascade = decode_controller_vector("CASCADE_PID", log10([0.24, ...
    0.0004, 0.19, 9.25, 1.01]), plant, config);
verifyEqual(test_case, attitude.kp, 1.9, "RelTol", 1e-12);
verifyEqual(test_case, attitude.u_max, plant.u_max);
verifyEqual(test_case, cascade.theta_reference_limit, deg2rad(12));
verifyEqual(test_case, cascade.sample_time, 0.01);
end
```

- [ ] **Step 2: Run tests and verify missing-function failures**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_training_scenarios.m','tests/test_controller_parameterization.m'}); assertSuccess(r)"
```

Expected: failure because the three production functions do not exist.

- [ ] **Step 3: Implement exact generic scenarios and legacy decoding**

Each generic scenario must contain exactly these public fields:

```matlab
scenario = struct( ...
    "name", name, ...
    "split", "train", ...
    "duration", duration, ...
    "initial_state", initial_state(:), ...
    "x_reference", x_reference, ...
    "force_disturbance", force_disturbance, ...
    "torque_disturbance", torque_disturbance, ...
    "reference_start", reference_start, ...
    "disturbance_end", disturbance_end, ...
    "measurement_noise_std", zeros(4, 1));
```

`controller_parameter_space.m` returns `name`, `parameter_names`, `lower_bounds`, `upper_bounds`, and `dimension`. Use the exact bounds from Section 5 of the spec. `decode_controller_vector.m` checks dimension, bounds, real finiteness, applies `10.^vector`, and calls the existing parameter validators:

```matlab
switch upper(string(controller_name))
    case "ATTITUDE_PID"
        gains = 10 .^ vector(:).';
        params = attitude_pid_params(struct( ...
            "kp", gains(1), "ki", gains(2), "kd", gains(3), ...
            "sample_time", config.sample_time, ...
            "plant_step", config.plant_step), plant_params);
    case "CASCADE_PID"
        gains = 10 .^ vector(:).';
        params = cascade_pid_params(struct( ...
            "kp_x", gains(1), "ki_x", gains(2), "kd_x", gains(3), ...
            "kp_theta", gains(4), "kd_theta", gains(5), ...
            "sample_time", config.sample_time, ...
            "plant_step", config.plant_step, ...
            "theta_reference_limit", config.theta_reference_limit), ...
            plant_params);
    otherwise
        error("twsbr:controller:unsupported_name", ...
            "Controller is not implemented in this phase: %s", controller_name);
end
```

- [ ] **Step 4: Run focused tests**

Run the command from Step 2. Expected: all tests pass.

- [ ] **Step 5: Commit generic scenarios and legacy decoding**

```powershell
git add scenarios/training_scenarios.m controllers/controller_parameter_space.m controllers/decode_controller_vector.m tests/test_training_scenarios.m tests/test_controller_parameterization.m
git commit -m "feat: define study scenarios and parameter spaces"
```

### Task 3: Legacy controller adapters and common controller lifecycle

**Files:**
- Create: `controllers/adapt_attitude_pid_controller.m`
- Create: `controllers/adapt_cascade_pid_controller.m`
- Create: `controllers/create_controller.m`
- Create: `controllers/reset_controller.m`
- Create: `controllers/controller_step.m`
- Create: `controllers/controller_after_actuation.m`
- Create: `tests/test_legacy_controller_adapters.m`

**Interfaces:**
- Consumes: decoded legacy parameter structures and state `[x; x_dot; theta; theta_dot]`.
- Produces: controller structures with fields `name`, `params`, `state`, `pending_state`, and `pending_legacy_u`; step results with fields `u_raw`, `theta_reference`, and `diagnostics`.

- [ ] **Step 1: Write failing lifecycle and adapter tests**

```matlab
function test_attitude_adapter_uses_existing_step_and_ignores_position(test_case)
plant = twsbr_params();
config = experiment_config("quick");
vector = log10([1.9, 0.2, 0.18]);
controller = create_controller("ATTITUDE_PID", vector, plant, config);
controller = reset_controller(controller);
[control, controller] = controller_step(controller, 0.0, ...
    [0; 0; 0.1; 0.2], 5.0);
[legacy, legacy_state] = attitude_pid_step( ...
    struct("integral_error", 0.0), 0.1, 0.2, 0.0, controller.params);
verifyEqual(test_case, control.u_raw, legacy.u_raw, "AbsTol", 1e-15);
verifyEqual(test_case, control.theta_reference, 0.0);
verifyEqual(test_case, controller.pending_state, legacy_state);
controller = controller_after_actuation(controller, control.u_raw, legacy.u);
verifyEqual(test_case, controller.state, legacy_state);
end

function test_cascade_adapter_uses_existing_step(test_case)
plant = twsbr_params();
config = experiment_config("quick");
vector = log10([0.24100028146267993, 0.0003962067755988572, ...
    0.1930824033173246, 9.254929149177556, 1.0113335430173094]);
controller = reset_controller(create_controller( ...
    "CASCADE_PID", vector, plant, config));
[control, controller] = controller_step(controller, 0.0, zeros(4, 1), 0.5);
[legacy, legacy_state] = cascade_pid_step( ...
    struct("position_integral", 0.0), zeros(4, 1), 0.5, controller.params);
verifyEqual(test_case, control.u_raw, legacy.u_raw, "AbsTol", 1e-15);
verifyEqual(test_case, control.theta_reference, legacy.theta_reference);
controller = controller_after_actuation(controller, control.u_raw, legacy.u);
verifyEqual(test_case, controller.state, legacy_state);
end
```

- [ ] **Step 2: Run adapter tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_legacy_controller_adapters.m'); assertSuccess(r)"
```

Expected: failure because the controller lifecycle functions do not exist.

- [ ] **Step 3: Implement adapters and lifecycle dispatch**

The attitude adapter calls:

```matlab
[legacy, next_state] = attitude_pid_step(controller.state, ...
    measured_state(3), measured_state(4), 0.0, controller.params);
control = struct("u_raw", legacy.u_raw, "theta_reference", 0.0, ...
    "diagnostics", legacy);
controller.pending_state = next_state;
controller.pending_legacy_u = legacy.u;
```

The cascade adapter calls:

```matlab
[legacy, next_state] = cascade_pid_step(controller.state, ...
    measured_state, x_reference, controller.params);
control = struct("u_raw", legacy.u_raw, ...
    "theta_reference", legacy.theta_reference, ...
    "diagnostics", legacy);
controller.pending_state = next_state;
controller.pending_legacy_u = legacy.u;
```

`controller_after_actuation.m` verifies:

```matlab
if abs(applied_u - controller.pending_legacy_u) > 1e-12
    error("twsbr:controller:legacy_saturation_mismatch", ...
        "Common and legacy actuator saturation differ.");
end
controller.state = controller.pending_state;
```

`create_controller.m` decodes the vector and initializes empty lifecycle fields. `reset_controller.m` selects exactly `struct("integral_error", 0.0)` or `struct("position_integral", 0.0)`. `controller_step.m` validates a finite four-state column vector and dispatches by exact controller name.

- [ ] **Step 4: Run adapter and existing controller tests**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_legacy_controller_adapters.m','tests/test_attitude_pid_controller.m','tests/test_cascade_pid_step.m'}); assertSuccess(r)"
```

Expected: all tests pass.

- [ ] **Step 5: Commit the legacy adapter layer**

```powershell
git add controllers/adapt_attitude_pid_controller.m controllers/adapt_cascade_pid_controller.m controllers/create_controller.m controllers/reset_controller.m controllers/controller_step.m controllers/controller_after_actuation.m tests/test_legacy_controller_adapters.m
git commit -m "feat: adapt legacy PID controllers to common interface"
```

### Task 4: Common fixed-step nonlinear simulator

**Files:**
- Create: `simulation/simulate_control_system.m`
- Create: `tests/test_unified_plant_equivalence.m`

**Interfaces:**
- Consumes: controller lifecycle functions, a generic scenario, nominal or perturbed plant parameters, and a scalar seed.
- Produces: a simulation structure with fields `controller_name`, `scenario_name`, `seed`, `time`, `state`, `position_reference`, `theta_reference`, `u_raw`, `u`, `saturated`, `force_disturbance`, `torque_disturbance`, `diagnostics`, `success`, `failure_reason`, `survived_time`, and `runtime_seconds`.

- [ ] **Step 1: Write a failing one-step plant and schema test**

```matlab
function test_common_simulator_matches_existing_rk4_step(test_case)
plant = twsbr_params();
config = experiment_config("quick");
scenario = training_scenarios(0.01).T1_initial_tilt_5deg;
vector = log10([0.0 + 1e-4, 1e-4, 1e-4]);
result = simulate_control_system("ATTITUDE_PID", vector, plant, ...
    config, scenario, 7);
expected = twsbr_rk4_step(scenario.initial_state, result.u(1), ...
    config.plant_step, plant, result.force_disturbance(1), ...
    result.torque_disturbance(1));
verifyEqual(test_case, result.state(2, :).', expected, "AbsTol", 1e-14);
verifyEqual(test_case, fieldnames(result), { ...
    'controller_name'; 'scenario_name'; 'seed'; 'time'; 'state'; ...
    'position_reference'; 'theta_reference'; 'u_raw'; 'u'; ...
    'saturated'; 'force_disturbance'; 'torque_disturbance'; ...
    'diagnostics'; 'success'; 'failure_reason'; 'survived_time'; ...
    'runtime_seconds'});
end
```

- [ ] **Step 2: Run the simulator test and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_unified_plant_equivalence.m'); assertSuccess(r)"
```

Expected: failure because `simulate_control_system` does not exist.

- [ ] **Step 3: Implement the simulator with the frozen update order**

Use this loop order for every plant step:

```matlab
if mod(index - 1, controller_step_ratio) == 0
    measured_state = state(index, :).';
    noise = measurement_noise(index, :).';
    measured_state = measured_state + noise;
    [held_control, controller] = controller_step(controller, ...
        current_time, measured_state, reference);
    held_u = min(max(held_control.u_raw, -plant_params.u_max), ...
        plant_params.u_max);
    controller = controller_after_actuation( ...
        controller, held_control.u_raw, held_u);
end
state(index + 1, :) = twsbr_rk4_step(state(index, :).', held_u, ...
    config.plant_step, plant_params, force, torque).';
```

Generate the complete measurement-noise matrix once with a local `RandStream('mt19937ar','Seed',seed)` and never change the global random stream. Stop and truncate logs on nonfinite state, `abs(theta) > deg2rad(30)`, or `abs(x) > 5`. Store controller-specific diagnostics as a structure array with one element per logged sample.

- [ ] **Step 4: Run the simulator, plant, and model tests**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_unified_plant_equivalence.m','tests/test_twsbr_model.m','tests/test_twsbr_simulation.m'}); assertSuccess(r)"
```

Expected: all tests pass.

- [ ] **Step 5: Commit the common simulator**

```powershell
git add simulation/simulate_control_system.m tests/test_unified_plant_equivalence.m
git commit -m "feat: add common nonlinear control simulator"
```

### Task 5: Machine-precision legacy equivalence gate

**Files:**
- Create: `tests/test_unified_attitude_pid_equivalence.m`
- Create: `tests/test_unified_cascade_pid_equivalence.m`
- Create: `tests/test_legacy_entry_points_after_integration.m`

**Interfaces:**
- Consumes: legacy simulators and the common simulator.
- Produces: an executable regression gate required before starting the new-controller plan.

- [ ] **Step 1: Write trajectory equivalence tests**

For attitude PID, use `attitude_pid_scenarios().positive_tilt` and construct the equivalent generic scenario with zero position reference. Compare:

```matlab
legacy = simulate_attitude_pid(plant, pid_params, legacy_scenario);
common = simulate_control_system("ATTITUDE_PID", ...
    log10([pid_params.kp, pid_params.ki, pid_params.kd]), ...
    plant, config, generic_scenario, 0);
verifyEqual(test_case, common.time, legacy.time, "AbsTol", 1e-14);
verifyEqual(test_case, common.state, legacy.state, "AbsTol", 1e-12);
verifyEqual(test_case, common.u_raw, legacy.u_raw, "AbsTol", 1e-12);
verifyEqual(test_case, common.u, legacy.u, "AbsTol", 1e-12);
verifyEqual(test_case, common.saturated, legacy.saturated);
verifyEqual(test_case, common.success, legacy.success);
verifyEqual(test_case, common.failure_reason, legacy.failure_reason);
```

For cascade PID, use `cascade_pid_scenarios().positive_position_step` and the same assertions. The generic scenario must reproduce the exact reference, disturbance functions, event times, duration, and initial state.

The entry-point regression test calls `run_project(false)`, `run_attitude_pid(false)`, and `run_cascade_pid(false)` from a temporary external directory and asserts their existing model, result, and figure paths.

- [ ] **Step 2: Run the new gate and inspect any numerical difference**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_unified_attitude_pid_equivalence.m','tests/test_unified_cascade_pid_equivalence.m','tests/test_legacy_entry_points_after_integration.m'}); assertSuccess(r)"
```

Expected on the first run: either all pass or a focused failure that identifies update-order differences. Do not widen tolerances to Simulink-scale values.

- [ ] **Step 3: Align only adapter or common-simulator ordering**

If a comparison fails, preserve legacy source functions and change only generic signal sampling, logging, state truncation, or adapter commit order until the tested tolerances pass. The required plant call remains:

```matlab
next_state = twsbr_rk4_step(current_state, applied_u, ...
    config.plant_step, plant_params, force_disturbance, ...
    torque_disturbance);
```

- [ ] **Step 4: Run the entire existing and new test suite**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests'); assertSuccess(r)"
```

Expected: all legacy and new tests pass with no changed legacy acceptance thresholds.

- [ ] **Step 5: Commit the compatibility gate**

```powershell
git add simulation/simulate_control_system.m controllers tests/test_unified_attitude_pid_equivalence.m tests/test_unified_cascade_pid_equivalence.m tests/test_legacy_entry_points_after_integration.m
git commit -m "test: enforce unified legacy controller equivalence"
```

## Completion gate

Do not begin the new-controller implementation plan until Task 5 passes the complete test suite and `git status --short` is empty.
