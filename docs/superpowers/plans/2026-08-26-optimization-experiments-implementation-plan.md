# Optimization and Experiments Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Train all five controllers with one exact-budget differential-evolution implementation, freeze parameters from training only, and run held-out deterministic and paired Monte Carlo experiments through the common nonlinear simulator.

**Architecture:** Separate metric calculation, objective evaluation, optimizer mechanics, parameter selection, and experiment runners. Every stochastic component uses a local seeded stream, and Monte Carlo conditions are generated once per run before evaluating all controllers.

**Tech Stack:** MATLAB tables and structures, fixed-step common simulator, custom differential evolution, function-based unit tests.

**Spec:** `docs/superpowers/specs/2026-08-26-five-controller-control-study-design.md`

## Global Constraints

- Complete the unified legacy core and new controller plans first.
- Training uses only `T1_initial_tilt_5deg`, `T2_position_step_0p5m`, and `T3_impulse_disturbance`.
- Held-out and Monte Carlo scenarios never affect tuning, objective scaling, or parameter selection.
- All five controllers use identical population size, exact evaluation budget, seed set, objective weights, failure penalties, plant, and actuator limit.
- Invalid vectors return `1e6`; failed simulations remain in all result tables.
- Use local `RandStream` objects and do not mutate MATLAB's global random stream.
- Continuous Monte Carlo metrics are later summarized over successful trials only; success rate always uses every trial.
- Preserve the legacy equivalence gate after every task.

---

## File map

- `config/objective_config.m`: fixed weights, scales, and penalties.
- `config/monte_carlo_config.m`: fixed parameter, noise, reference, and disturbance ranges.
- `evaluation/calculate_control_metrics.m`: one simulation-to-record conversion.
- `optimization/differential_evolution.m`: deterministic exact-budget optimizer.
- `optimization/control_objective.m`: common three-scenario objective.
- `optimization/tune_controller.m`: run all training seeds for one controller.
- `optimization/tune_all_controllers.m`: apply identical options to all controllers.
- `optimization/select_frozen_parameters.m`: select lowest training cost only.
- `scenarios/heldout_scenarios.m`: deterministic test-only scenarios.
- `scenarios/generate_monte_carlo_scenario.m`: paired random condition generator.
- `experiments/run_deterministic_batch.m`: controller-by-scenario held-out runner.
- `experiments/run_monte_carlo.m`: run-index-first paired robustness runner.
- `experiments/benchmark_controllers.m`: controller-step runtime and parameter dimensions.
- `tests/test_control_metrics.m`: exact metrics and task success.
- `tests/test_differential_evolution.m`: determinism, bounds, and exact budget.
- `tests/test_control_objective.m`: weights and penalties.
- `tests/test_tuning_pipeline.m`: equal options and training-only selection.
- `tests/test_heldout_scenarios.m`: exact isolated test set.
- `tests/test_monte_carlo_pairing.m`: identical random conditions for all controllers.
- `tests/test_deterministic_batch.m`: row counts, failures, and raw result indexing.

### Task 1: Fixed objective and Monte Carlo configuration

**Files:**
- Create: `config/objective_config.m`
- Create: `config/monte_carlo_config.m`
- Create: `tests/test_study_configs.m`

**Interfaces:**
- Consumes: no mutable external state.
- Produces: `config = objective_config()` and `config = monte_carlo_config(mode)`.

- [ ] **Step 1: Write failing frozen-value tests**

```matlab
function test_objective_weights_scales_and_penalties_are_frozen(test_case)
config = objective_config();
verifyEqual(test_case, config.metric_names, ["theta_itae"; ...
    "position_itae"; "control_energy"; "saturation_time"; ...
    "max_abs_theta_rad"; "disturbance_recovery_time"]);
verifyEqual(test_case, config.weights, [0.25; 0.25; 0.15; 0.15; 0.10; 0.10]);
verifyEqual(test_case, config.scales, ...
    [0.8; 8.0; 4.0; 1.0; deg2rad(10); 3.0], "AbsTol", 1e-15);
verifyEqual(test_case, sum(config.weights), 1.0, "AbsTol", 1e-15);
verifyEqual(test_case, config.failure_penalty, 500);
verifyEqual(test_case, config.invalid_penalty, 1e6);
end

function test_monte_carlo_ranges_are_mode_independent(test_case)
quick = monte_carlo_config("quick");
full = monte_carlo_config("full");
verifyEqual(test_case, quick.parameter_rho, 0.10);
verifyEqual(test_case, full.parameter_rho, 0.20);
verifyEqual(test_case, quick.initial_tilt_deg_range, [2, 8]);
verifyEqual(test_case, quick.reference_range, [0.25, 1.0]);
verifyEqual(test_case, quick.force_range, [4, 9]);
verifyEqual(test_case, quick.noise_std, ...
    [0.002; 0.01; deg2rad(0.15); deg2rad(0.5)], "AbsTol", 1e-15);
end
```

- [ ] **Step 2: Run tests and verify missing functions**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_study_configs.m'); assertSuccess(r)"
```

Expected: failure because both configuration functions are undefined.

- [ ] **Step 3: Implement scalar validated configuration structures**

`objective_config.m` returns the exact arrays tested above. `monte_carlo_config.m` validates `quick` or `full`, sets `parameter_rho`, and fixes:

```matlab
config.duration = 10.0;
config.initial_tilt_deg_range = [2.0, 8.0];
config.reference_range = [0.25, 1.0];
config.reference_start = 1.0;
config.force_range = [4.0, 9.0];
config.force_start_range = [3.0, 5.0];
config.force_duration_range = [0.10, 0.30];
config.constant_force_range = [-0.5, 0.5];
config.noise_std = [0.002; 0.01; deg2rad(0.15); deg2rad(0.5)];
```

- [ ] **Step 4: Run focused configuration tests**

Run the command from Step 2. Expected: all tests pass.

- [ ] **Step 5: Commit frozen experiment configuration**

```powershell
git add config/objective_config.m config/monte_carlo_config.m tests/test_study_configs.m
git commit -m "feat: freeze study objective and uncertainty config"
```

### Task 2: Common metrics and task-success policy

**Files:**
- Create: `evaluation/calculate_control_metrics.m`
- Create: `tests/test_control_metrics.m`

**Interfaces:**
- Consumes: one common simulation structure, its generic scenario, and plant parameters.
- Produces: a scalar metrics structure with all fields listed in Section 11 of the spec.

- [ ] **Step 1: Write failing analytical-metric tests**

```matlab
function test_metrics_match_simple_trajectory(test_case)
simulation = struct();
simulation.controller_name = "TEST";
simulation.scenario_name = "linear";
simulation.seed = 3;
simulation.time = [0; 1; 2];
simulation.state = [0,0,0,0; 0.5,0,0.1,0; 1,0,0,0];
simulation.position_reference = [1; 1; 1];
simulation.u_raw = [0; 0.5; 1];
simulation.u = [0; 0.5; 1];
simulation.saturated = [false; false; true];
simulation.success = true;
simulation.failure_reason = "";
simulation.survived_time = 2;
simulation.runtime_seconds = 0.003;
scenario = struct("split", "test", "reference_start", 0, ...
    "disturbance_end", 0, "duration", 2);
metrics = calculate_control_metrics(simulation, scenario, twsbr_params());
verifyEqual(test_case, metrics.position_itae, 0.5, "AbsTol", 1e-12);
verifyEqual(test_case, metrics.control_energy, 0.75, "AbsTol", 1e-12);
verifyEqual(test_case, metrics.max_abs_theta_rad, 0.1, "AbsTol", 1e-12);
verifyEqual(test_case, metrics.saturation_time, 0.5, "AbsTol", 1e-12);
verifyEqual(test_case, metrics.saturation_ratio, 1/3, "AbsTol", 1e-12);
end

function test_task_failure_is_reported_without_overwriting_simulation_failure(test_case)
simulation = make_unsettled_successful_simulation();
scenario = struct("split", "test", "reference_start", 0, ...
    "disturbance_end", 0, "duration", simulation.time(end));
metrics = calculate_control_metrics(simulation, scenario, twsbr_params());
verifyFalse(test_case, metrics.success);
verifyTrue(test_case, metrics.simulation_success);
verifyEqual(test_case, metrics.failure_reason, "task_not_settled");
end
```

- [ ] **Step 2: Run metric tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_control_metrics.m'); assertSuccess(r)"
```

Expected: failure because `calculate_control_metrics` is undefined.

- [ ] **Step 3: Implement exact integrations and final-window success**

Use `trapz(time, signal)` for ITAE, energy, and saturation time. Compute:

```matlab
theta = simulation.state(:, 3);
position_error = simulation.position_reference - simulation.state(:, 1);
theta_itae = trapz(time, time .* abs(theta));
position_itae = trapz(time, time .* abs(position_error));
control_energy = trapz(time, simulation.u .^ 2);
saturation_time = trapz(time, double(simulation.saturated));
```

Use the last `0.5 s` of samples for task success. Require mean absolute tilt at most `3 deg`, mean absolute speed at most `0.25 m/s`, and mean position error within `max(0.10,0.20*abs(final_reference))`; use `0.30 m` when the final reference is zero. Preserve an existing simulation failure reason; otherwise use `task_not_settled`. Implement reverse-scan settling time with a fixed tolerance and return the scenario duration when no permanent settling occurs.

- [ ] **Step 4: Run metric and simulator tests**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_control_metrics.m','tests/test_unified_plant_equivalence.m'}); assertSuccess(r)"
```

Expected: all tests pass.

- [ ] **Step 5: Commit common metrics**

```powershell
git add evaluation/calculate_control_metrics.m tests/test_control_metrics.m
git commit -m "feat: add unified control performance metrics"
```

### Task 3: Deterministic exact-budget differential evolution

**Files:**
- Create: `optimization/differential_evolution.m`
- Create: `tests/test_differential_evolution.m`

**Interfaces:**
- Consumes: scalar objective function handle, parameter-space structure, and options with `population_size`, `evaluation_budget`, `seed`, `mutation_factor`, `crossover_rate`, and optional `starter_vector`.
- Produces: `[best_vector,best_value,history]` where `history` records every evaluation number, candidate, value, and current best.

- [ ] **Step 1: Write failing exact-budget and determinism tests**

```matlab
function test_de_uses_exact_budget_and_respects_bounds(test_case)
space = struct("dimension", 3, "lower_bounds", -ones(1,3), ...
    "upper_bounds", ones(1,3));
options = struct("population_size", 8, "evaluation_budget", 31, ...
    "seed", 9, "mutation_factor", 0.8, "crossover_rate", 0.9, ...
    "starter_vector", [0.5, -0.5, 0.25]);
[best, value, history] = differential_evolution( ...
    @(x) sum(x.^2), space, options);
verifyEqual(test_case, height(history), 31);
verifyEqual(test_case, history.evaluation, (1:31).');
verifyGreaterThanOrEqual(test_case, best, space.lower_bounds);
verifyLessThanOrEqual(test_case, best, space.upper_bounds);
verifyLessThanOrEqual(test_case, value, sum(options.starter_vector.^2));
end

function test_de_is_deterministic_without_global_rng_changes(test_case)
rng(77);
before = rng;
[a, va, ha] = run_small_de();
middle = rng;
[b, vb, hb] = run_small_de();
after = rng;
verifyEqual(test_case, a, b);
verifyEqual(test_case, va, vb);
verifyEqual(test_case, ha, hb);
verifyEqual(test_case, middle, before);
verifyEqual(test_case, after, before);
end
```

- [ ] **Step 2: Run optimizer tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_differential_evolution.m'); assertSuccess(r)"
```

Expected: failure because `differential_evolution` is undefined.

- [ ] **Step 3: Implement current-to-best-free classic DE/rand/1/bin**

Create `RandStream("mt19937ar","Seed",options.seed)`. Initialize a bounded uniform population, replace its first row with the validated starter vector when supplied, and evaluate every initial member. For each target, select three distinct other indices with the local stream, calculate:

```matlab
mutant = population(a,:) + options.mutation_factor * ...
    (population(b,:) - population(c,:));
mutant = min(max(mutant, lower_bounds), upper_bounds);
mask = rand(stream, 1, dimension) < options.crossover_rate;
mask(randi(stream, dimension)) = true;
trial = population(target, :);
trial(mask) = mutant(mask);
```

Evaluate and apply greedy replacement. Stop immediately when the exact budget is reached, even in the middle of a generation. Append one table row per objective call. Reject population sizes below four, budgets below population size, invalid bounds, and nonfinite objective returns with stable `twsbr:de:*` identifiers.

- [ ] **Step 4: Run optimizer tests and code analysis**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_differential_evolution.m'); assertSuccess(r); assert(isempty(checkcode('optimization/differential_evolution.m','-id')))"
```

Expected: all tests pass and no code-analysis messages.

- [ ] **Step 5: Commit differential evolution**

```powershell
git add optimization/differential_evolution.m tests/test_differential_evolution.m
git commit -m "feat: add exact-budget differential evolution"
```

### Task 4: Shared three-scenario objective

**Files:**
- Create: `optimization/control_objective.m`
- Create: `tests/test_control_objective.m`

**Interfaces:**
- Consumes: controller name, vector, nominal plant, study config, objective config, training scenarios, and base seed.
- Produces: `[cost,details]`; cost is finite scalar, details contains per-scenario metrics and penalties.

- [ ] **Step 1: Write failing objective and penalty tests**

```matlab
function test_invalid_vector_returns_invalid_penalty(test_case)
config = experiment_config("quick");
[cost, details] = control_objective("ATTITUDE_PID", [NaN, 0, 0], ...
    twsbr_params(), config, objective_config(), training_scenarios(4), 0);
verifyEqual(test_case, cost, 1e6);
verifyEqual(test_case, details.failure_reason, "invalid_controller_vector");
end

function test_objective_is_deterministic_and_uses_three_scenarios(test_case)
config = experiment_config("quick");
vector = log10([1.9, 0.2, 0.18]);
scenarios = training_scenarios(4);
[first, first_details] = control_objective("ATTITUDE_PID", vector, ...
    twsbr_params(), config, objective_config(), scenarios, 12);
[second, second_details] = control_objective("ATTITUDE_PID", vector, ...
    twsbr_params(), config, objective_config(), scenarios, 12);
verifyEqual(test_case, second, first);
verifyEqual(test_case, second_details, first_details);
verifyEqual(test_case, numel(first_details.scenarios), 3);
end
```

- [ ] **Step 2: Run objective tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_control_objective.m'); assertSuccess(r)"
```

Expected: failure because `control_objective` is undefined.

- [ ] **Step 3: Implement normalized cost and survival-weighted failure penalty**

For each training scenario, run `simulate_control_system`, then `calculate_control_metrics`. Build the successful cost in the exact metric order from `objective_config`. Set recovery contribution to zero when `scenario.disturbance_end == 0`. If task success is false, add:

```matlab
scenario_cost = scenario_cost + objective.failure_penalty * ...
    (2 - simulation.survived_time / scenario.duration);
```

Return the mean of the three scenario costs. Catch only known candidate errors from decoding, Riccati solving, fuzzy validation, simulation nonfinite output, and metrics; convert those to `invalid_penalty` and a structured reason. Rethrow configuration and programming errors.

- [ ] **Step 4: Run objective, metric, and controller tests**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_control_objective.m','tests/test_control_metrics.m','tests/test_all_controller_factory.m'}); assertSuccess(r)"
```

Expected: all tests pass.

- [ ] **Step 5: Commit the unified objective**

```powershell
git add optimization/control_objective.m tests/test_control_objective.m
git commit -m "feat: add shared controller training objective"
```

### Task 5: Multi-seed tuning and frozen parameter selection

**Files:**
- Create: `optimization/tune_controller.m`
- Create: `optimization/tune_all_controllers.m`
- Create: `optimization/select_frozen_parameters.m`
- Create: `tests/test_tuning_pipeline.m`

**Interfaces:**
- Consumes: controller names, study configuration, nominal plant, training scenarios, and optional starter vectors.
- Produces: a tuning-run table and `frozen_vectors` structure selected only by minimum training cost.

- [ ] **Step 1: Write failing equal-budget and selection tests**

```matlab
function test_selection_uses_only_training_cost(test_case)
runs = table(["LQR";"LQR";"LQI";"LQI"], [0;1;0;1], ...
    [2.0;1.0;4.0;3.0], {[1,2]};{[3,4]};{[5,6]};{[7,8]}, ...
    'VariableNames', {'controller','seed','training_cost','vector'});
frozen = select_frozen_parameters(runs);
verifyEqual(test_case, frozen.LQR, [3,4]);
verifyEqual(test_case, frozen.LQI, [7,8]);
end

function test_tune_controller_uses_same_exact_budget_per_seed(test_case)
config = experiment_config("quick");
config.population_size = 4;
config.evaluation_budget = 8;
config.tuning_seeds = [2, 3];
runs = tune_controller("ATTITUDE_PID", twsbr_params(), config, ...
    objective_config(), training_scenarios(4), ...
    log10([1.9,0.2,0.18]));
verifyEqual(test_case, runs.evaluation_count, [8;8]);
verifyEqual(test_case, runs.seed, [2;3]);
end
```

- [ ] **Step 2: Run tuning tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_tuning_pipeline.m'); assertSuccess(r)"
```

Expected: failure because the tuning functions are undefined.

- [ ] **Step 3: Implement single-controller, all-controller, and selection functions**

`tune_controller` builds one objective closure per seed, calls `differential_evolution`, and appends controller, seed, training cost, vector, evaluation count, and elapsed seconds. `tune_all_controllers` loops `config.controller_names` in fixed order and passes identical options. It may not alter budget based on parameter dimension. `select_frozen_parameters` groups by controller and selects the first minimum training cost after sorting by `training_cost` then `seed`.

- [ ] **Step 4: Run tuning and optimizer tests**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_tuning_pipeline.m','tests/test_differential_evolution.m','tests/test_control_objective.m'}); assertSuccess(r)"
```

Expected: all tests pass.

- [ ] **Step 5: Commit the tuning pipeline**

```powershell
git add optimization/tune_controller.m optimization/tune_all_controllers.m optimization/select_frozen_parameters.m tests/test_tuning_pipeline.m
git commit -m "feat: add fair multi-controller tuning pipeline"
```

### Task 6: Held-out and paired Monte Carlo scenarios

**Files:**
- Create: `scenarios/heldout_scenarios.m`
- Create: `scenarios/generate_monte_carlo_scenario.m`
- Create: `tests/test_heldout_scenarios.m`
- Create: `tests/test_monte_carlo_pairing.m`

**Interfaces:**
- Consumes: duration, run index, base seed, Monte Carlo config, and nominal plant.
- Produces: held-out scenario structure; `[scenario,perturbed_plant,metadata] = generate_monte_carlo_scenario(...)`.

- [ ] **Step 1: Write failing isolation and pairing tests**

```matlab
function test_training_and_heldout_names_are_disjoint(test_case)
train = training_scenarios(8);
test = heldout_scenarios(10);
verifyEmpty(test_case, intersect(fieldnames(train), fieldnames(test)));
verifyTrue(test_case, all(structfun(@(s) s.split == "test", test)));
verifyEqual(test_case, numel(fieldnames(test)), 12);
end

function test_monte_carlo_generation_is_repeatable_and_local(test_case)
nominal = twsbr_params();
config = monte_carlo_config("quick");
rng(91); before = rng;
[a, plant_a, meta_a] = generate_monte_carlo_scenario(4, 20260, config, nominal);
[b, plant_b, meta_b] = generate_monte_carlo_scenario(4, 20260, config, nominal);
verifyEqual(test_case, a, b);
verifyEqual(test_case, plant_a, plant_b);
verifyEqual(test_case, meta_a, meta_b);
verifyEqual(test_case, rng, before);
verifyNotEqual(test_case, plant_a.body_mass, nominal.body_mass);
end
```

- [ ] **Step 2: Run scenario tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_heldout_scenarios.m','tests/test_monte_carlo_pairing.m'}); assertSuccess(r)"
```

Expected: failure because held-out and Monte Carlo generators are absent.

- [ ] **Step 3: Implement the exact held-out set and paired generator**

Create four initial-tilt scenarios, three position steps, one saturation-stress scenario, positive and negative force impulses, one torque impulse, and one constant-bias scenario with the names from the spec. Use function handles for references and disturbances.

For Monte Carlo, seed a local stream with `base_seed + run_index`, sample uniform deltas for body mass, center-of-mass length, and body inertia in `[-rho,rho]`, and create a validated perturbed plant through `twsbr_params(overrides)`. Alternate force sign by run-index parity. Store every sampled quantity and a `measurement_noise_seed` in metadata.

- [ ] **Step 4: Run all scenario tests**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_training_scenarios.m','tests/test_heldout_scenarios.m','tests/test_monte_carlo_pairing.m'}); assertSuccess(r)"
```

Expected: all tests pass.

- [ ] **Step 5: Commit held-out and Monte Carlo scenarios**

```powershell
git add scenarios/heldout_scenarios.m scenarios/generate_monte_carlo_scenario.m tests/test_heldout_scenarios.m tests/test_monte_carlo_pairing.m
git commit -m "feat: add isolated test and Monte Carlo scenarios"
```

### Task 7: Deterministic, Monte Carlo, and complexity batches

**Files:**
- Create: `experiments/run_deterministic_batch.m`
- Create: `experiments/run_monte_carlo.m`
- Create: `experiments/benchmark_controllers.m`
- Create: `tests/test_deterministic_batch.m`
- Extend: `tests/test_monte_carlo_pairing.m`

**Interfaces:**
- Consumes: frozen vector structure, nominal plant, configuration, scenarios, and seed.
- Produces: metric tables, raw result lookup structures, paired Monte Carlo table, and controller complexity table.

- [ ] **Step 1: Write failing batch-shape and pairing tests**

```matlab
function test_deterministic_batch_keeps_every_controller_scenario_pair(test_case)
[vectors, config] = starter_vectors_for_test();
scenario_struct = heldout_scenarios(2);
names = fieldnames(scenario_struct);
scenario_struct = rmfield(scenario_struct, names(3:end));
[metrics, raw] = run_deterministic_batch(vectors, twsbr_params(), ...
    config, scenario_struct, 5);
verifyEqual(test_case, height(metrics), 5 * 2);
verifyEqual(test_case, numel(fieldnames(raw)), 5 * 2);
verifyEqual(test_case, sort(unique(metrics.controller)), ...
    sort(config.controller_names));
end

function test_monte_carlo_rows_share_conditions_by_run_index(test_case)
[vectors, config] = starter_vectors_for_test();
config.monte_carlo_runs = 2;
table_out = run_monte_carlo(vectors, twsbr_params(), config, ...
    monte_carlo_config("quick"), 20260);
verifyEqual(test_case, height(table_out), 10);
for run_index = 0:1
    rows = table_out(table_out.run_index == run_index, :);
    verifyEqual(test_case, numel(unique(rows.delta_body_mass)), 1);
    verifyEqual(test_case, numel(unique(rows.force_amplitude)), 1);
    verifyEqual(test_case, numel(unique(rows.measurement_noise_seed)), 1);
end
end
```

- [ ] **Step 2: Run batch tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_deterministic_batch.m','tests/test_monte_carlo_pairing.m'}); assertSuccess(r)"
```

Expected: failure because experiment runners are undefined.

- [ ] **Step 3: Implement controller-complete batch loops**

`run_deterministic_batch` loops controller names outermost and scenario names innermost, saves every raw result under a MATLAB-valid key such as `ATTITUDE_PID__S1_initial_tilt_3deg`, and converts each metrics structure with `struct2table(...,'AsArray',true)`.

`run_monte_carlo` loops run index outermost, calls `generate_monte_carlo_scenario` once, then loops all controller names using the same scenario, perturbed plant, seed, and noise metadata. Append metadata columns to each metric record.

`benchmark_controllers` creates each controller once, performs reset plus step plus common saturation plus after-actuation for `config.benchmark_repeats`, and records calls, total seconds, mean step microseconds, and parameter dimension.

- [ ] **Step 4: Run batch tests and the complete suite**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests'); assertSuccess(r)"
```

Expected: all tests pass and legacy trajectory equivalence remains within frozen tolerances.

- [ ] **Step 5: Commit batch experiments**

```powershell
git add experiments/run_deterministic_batch.m experiments/run_monte_carlo.m experiments/benchmark_controllers.m tests/test_deterministic_batch.m tests/test_monte_carlo_pairing.m
git commit -m "feat: add deterministic and paired robustness batches"
```

## Completion gate

Before starting Simulink and reporting work, run a reduced five-controller tuning job with population size four and budget eight, freeze vectors from training only, run two held-out scenarios plus two Monte Carlo indices, and verify that all 20 expected test rows are retained even when a controller fails.
