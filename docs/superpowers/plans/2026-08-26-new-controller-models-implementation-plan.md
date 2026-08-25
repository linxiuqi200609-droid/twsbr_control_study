# New Controller Models Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add discrete LQR, discrete LQI, and fixed-rule fuzzy cascade PID to the common controller interface with deterministic, finite, and independently tested MATLAB implementations.

**Architecture:** Derive discrete linear models from the existing analytical plant, solve LQR/LQI gains through Control System Toolbox, and implement toolbox-independent fuzzy inference shared with later Simulink models. Extend the common factory only after each controller passes focused unit tests.

**Tech Stack:** MATLAB, Control System Toolbox `dlqr`, matrix exponential zero-order hold, function-based unit tests, existing common simulator.

**Spec:** `docs/superpowers/specs/2026-08-26-five-controller-control-study-design.md`

## Global Constraints

- Complete `2026-08-26-unified-legacy-core-implementation-plan.md` first.
- Keep state order `[x; x_dot; theta; theta_dot]`, sample time `0.01 s`, and plant step `0.001 s`.
- Controllers return unsaturated `u_raw`; only the common simulator applies the actuator limit.
- Fuzzy inference must not require Fuzzy Logic Toolbox.
- LQR and LQI candidates must have finite stabilizing discrete Riccati solutions.
- Fuzzy rules, membership centers, normalization ranges, and gain limits are fixed before optimization.
- Do not change existing attitude PID or cascade PID mathematics.
- Use TDD and commit after every independently passing controller.

---

## File map

- `models/twsbr_discrete_model.m`: zero-order-hold discretization of the existing analytical model.
- `models/twsbr_augmented_lqi_model.m`: five-state position-integral model.
- `controllers/lqr_step.m`: stateless LQR command calculation.
- `controllers/lqi_step.m`: LQI command and pending integral update.
- `controllers/fuzzy_pid_rule_base.m`: fixed symmetric `7 x 7` singleton rule matrices.
- `controllers/fuzzy_pid_inference.m`: normalized membership, firing, and weighted-average inference.
- `controllers/fuzzy_pid_step.m`: cascade outer loop and fuzzy self-tuning attitude PID inner loop.
- `controllers/controller_parameter_space.m`: add exact fuzzy PID, LQR, and LQI spaces.
- `controllers/decode_controller_vector.m`: build validated new-controller parameters and Riccati gains.
- `controllers/create_controller.m`, `reset_controller.m`, `controller_step.m`, `controller_after_actuation.m`: add three lifecycle branches.
- `tests/test_discrete_linear_model.m`: verify ZOH and augmented controllability.
- `tests/test_lqr_controller.m`: verify stable gain and command sign.
- `tests/test_lqi_controller.m`: verify augmented gain and integral behavior.
- `tests/test_fuzzy_pid_inference.m`: verify membership, symmetry, bounds, and determinism.
- `tests/test_fuzzy_pid_step.m`: verify online gain and state limits.
- `tests/test_all_controller_factory.m`: verify all five controllers share the common contract.

### Task 1: Discrete plant and LQI augmentation

**Files:**
- Create: `models/twsbr_discrete_model.m`
- Create: `models/twsbr_augmented_lqi_model.m`
- Create: `tests/test_discrete_linear_model.m`

**Interfaces:**
- Consumes: `[A,B,C,D] = twsbr_linear_model(plant_params)` and positive scalar sample time.
- Produces: `[Ad,Bd,Cd,Dd] = twsbr_discrete_model(plant_params, sample_time)` and `[Aaug,Baug] = twsbr_augmented_lqi_model(Ad,Bd,sample_time)`.

- [ ] **Step 1: Write failing ZOH and controllability tests**

```matlab
function tests = test_discrete_linear_model
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(root, "-begin");
setup_project();
end

function teardownOnce(test_case)
path(test_case.TestData.original_path);
end

function test_zero_order_hold_matches_matrix_exponential(test_case)
plant = twsbr_params();
sample_time = 0.01;
[a, b] = twsbr_linear_model(plant);
[ad, bd, cd, dd] = twsbr_discrete_model(plant, sample_time);
augmented = expm([a, b; zeros(1, 5)] * sample_time);
verifyEqual(test_case, ad, augmented(1:4, 1:4), "AbsTol", 1e-14);
verifyEqual(test_case, bd, augmented(1:4, 5), "AbsTol", 1e-14);
verifyEqual(test_case, cd, eye(4));
verifyEqual(test_case, dd, zeros(4, 1));
end

function test_lqi_augmented_model_is_controllable(test_case)
[ad, bd] = twsbr_discrete_model(twsbr_params(), 0.01);
[a_aug, b_aug] = twsbr_augmented_lqi_model(ad, bd, 0.01);
controllability = [b_aug, a_aug*b_aug, a_aug^2*b_aug, ...
    a_aug^3*b_aug, a_aug^4*b_aug];
verifyEqual(test_case, size(a_aug), [5, 5]);
verifyEqual(test_case, size(b_aug), [5, 1]);
verifyEqual(test_case, rank(controllability), 5);
verifyEqual(test_case, a_aug(5, :), [0.01, 0, 0, 0, 1], ...
    "AbsTol", 1e-15);
end
```

- [ ] **Step 2: Run the tests and verify missing functions**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_discrete_linear_model.m'); assertSuccess(r)"
```

Expected: failure because both model functions are undefined.

- [ ] **Step 3: Implement exact ZOH and the five-state augmentation**

```matlab
function [ad, bd, cd, dd] = twsbr_discrete_model(plant_params, sample_time)
[a, b, cd, dd] = twsbr_linear_model(plant_params);
if ~isnumeric(sample_time) || ~isscalar(sample_time) || ...
        ~isfinite(sample_time) || sample_time <= 0
    error("twsbr:linear:invalid_sample_time", ...
        "sample_time must be a positive finite scalar.");
end
zoh = expm([a, b; zeros(1, 5)] * sample_time);
ad = zoh(1:4, 1:4);
bd = zoh(1:4, 5);
end
```

```matlab
function [a_aug, b_aug] = twsbr_augmented_lqi_model(ad, bd, sample_time)
if ~isequal(size(ad), [4, 4]) || ~isequal(size(bd), [4, 1]) || ...
        any(~isfinite(ad), "all") || any(~isfinite(bd), "all")
    error("twsbr:lqi:invalid_discrete_model", ...
        "ad and bd must be finite 4-state matrices.");
end
a_aug = [ad, zeros(4, 1); sample_time*[1, 0, 0, 0], 1];
b_aug = [bd; 0];
end
```

- [ ] **Step 4: Run focused and analytical-model tests**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_discrete_linear_model.m','tests/test_twsbr_linear_model.m'}); assertSuccess(r)"
```

Expected: all tests pass.

- [ ] **Step 5: Commit discrete models**

```powershell
git add models/twsbr_discrete_model.m models/twsbr_augmented_lqi_model.m tests/test_discrete_linear_model.m
git commit -m "feat: add discrete plant and LQI models"
```

### Task 2: Discrete LQR controller

**Files:**
- Create: `controllers/lqr_step.m`
- Create: `tests/test_lqr_controller.m`
- Modify: `controllers/controller_parameter_space.m`
- Modify: `controllers/decode_controller_vector.m`
- Modify: `controllers/create_controller.m`
- Modify: `controllers/reset_controller.m`
- Modify: `controllers/controller_step.m`
- Modify: `controllers/controller_after_actuation.m`

**Interfaces:**
- Consumes: a five-value log vector, nominal plant parameters, sample time, finite state, and position reference.
- Produces: decoded fields `q_diag`, `r_value`, `gain`, and `closed_loop_eigenvalues`; `control = lqr_step(plant_state, x_reference, params)`.

- [ ] **Step 1: Write failing LQR tests**

```matlab
function test_decoded_lqr_is_discrete_and_stable(test_case)
plant = twsbr_params();
config = experiment_config("quick");
vector = log10([10, 1, 200, 10, 0.1]);
params = decode_controller_vector("LQR", vector, plant, config);
verifyEqual(test_case, size(params.gain), [1, 4]);
verifyTrue(test_case, all(abs(params.closed_loop_eigenvalues) < 1));
end

function test_lqr_uses_position_error_state(test_case)
plant = twsbr_params();
config = experiment_config("quick");
params = decode_controller_vector("LQR", ...
    log10([10, 1, 200, 10, 0.1]), plant, config);
control = lqr_step([0.2; 0; 0; 0], 0.2, params);
verifyEqual(test_case, control.u_raw, 0.0, "AbsTol", 1e-12);
control = lqr_step([0; 0; 0.05; 0], 0.0, params);
verifyTrue(test_case, isfinite(control.u_raw));
verifyEqual(test_case, control.state_error, [0; 0; 0.05; 0]);
end
```

- [ ] **Step 2: Run the LQR tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_lqr_controller.m'); assertSuccess(r)"
```

Expected: failure because LQR decoding and `lqr_step` are absent.

- [ ] **Step 3: Implement LQR decoding, stability validation, and lifecycle branch**

Add the exact LQR bounds from the spec. Decode with:

```matlab
weights = 10 .^ vector(:).';
[ad, bd] = twsbr_discrete_model(plant_params, config.sample_time);
q_diag = weights(1:4);
r_value = weights(5);
[gain, ~, poles] = dlqr(ad, bd, diag(q_diag), r_value);
if any(~isfinite(gain), "all") || any(abs(poles) >= 1)
    error("twsbr:lqr:nonstabilizing_solution", ...
        "The LQR candidate does not stabilize the discrete plant.");
end
params = struct("q_diag", q_diag, "r_value", r_value, ...
    "gain", gain, "closed_loop_eigenvalues", poles, ...
    "sample_time", config.sample_time);
```

Implement the stateless step:

```matlab
function control = lqr_step(plant_state, x_reference, params)
state_error = plant_state(:) - [x_reference; 0; 0; 0];
u_raw = -params.gain * state_error;
control = struct("u_raw", u_raw, "theta_reference", 0.0, ...
    "state_error", state_error, ...
    "diagnostics", struct("state_error_norm", norm(state_error)));
end
```

Add `LQR` branches to the lifecycle. Its state and pending state are empty scalar structures, and `controller_after_actuation` performs no state update.

- [ ] **Step 4: Run LQR, factory, and discrete-model tests**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_lqr_controller.m','tests/test_discrete_linear_model.m','tests/test_legacy_controller_adapters.m'}); assertSuccess(r)"
```

Expected: all tests pass and legacy adapter tests remain unchanged.

- [ ] **Step 5: Commit LQR**

```powershell
git add controllers/lqr_step.m controllers/controller_parameter_space.m controllers/decode_controller_vector.m controllers/create_controller.m controllers/reset_controller.m controllers/controller_step.m controllers/controller_after_actuation.m tests/test_lqr_controller.m
git commit -m "feat: add discrete LQR controller"
```

### Task 3: Discrete LQI controller

**Files:**
- Create: `controllers/lqi_step.m`
- Create: `tests/test_lqi_controller.m`
- Modify: `controllers/controller_parameter_space.m`
- Modify: `controllers/decode_controller_vector.m`
- Modify: `controllers/create_controller.m`
- Modify: `controllers/reset_controller.m`
- Modify: `controllers/controller_step.m`
- Modify: `controllers/controller_after_actuation.m`

**Interfaces:**
- Consumes: six-value log vector and controller state `struct("position_integral", value)`.
- Produces: decoded `state_gain`, `integral_gain`, and stable augmented poles; `lqi_step` returns `u_raw`, position error, current integral, and pending next integral.

- [ ] **Step 1: Write failing LQI tests**

```matlab
function test_lqi_gain_is_finite_and_stable(test_case)
plant = twsbr_params();
config = experiment_config("quick");
params = decode_controller_vector("LQI", ...
    log10([10, 1, 200, 10, 100, 0.1]), plant, config);
verifyEqual(test_case, size(params.state_gain), [1, 4]);
verifyTrue(test_case, isscalar(params.integral_gain));
verifyTrue(test_case, all(abs(params.closed_loop_eigenvalues) < 1));
end

function test_lqi_integrates_position_error(test_case)
plant = twsbr_params();
config = experiment_config("quick");
params = decode_controller_vector("LQI", ...
    log10([10, 1, 200, 10, 100, 0.1]), plant, config);
[control, next_state] = lqi_step(struct("position_integral", 0.2), ...
    [0.1; 0; 0; 0], 0.4, params);
verifyEqual(test_case, control.position_error, -0.3, "AbsTol", 1e-15);
verifyEqual(test_case, next_state.position_integral, 0.197, "AbsTol", 1e-15);
verifyTrue(test_case, isfinite(control.u_raw));
end
```

- [ ] **Step 2: Run LQI tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_lqi_controller.m'); assertSuccess(r)"
```

Expected: failure because LQI support is absent.

- [ ] **Step 3: Implement LQI gain decoding and state update**

Use `dlqr(a_aug,b_aug,diag(q_aug_diag),r_value)`, split `gain_aug(1:4)` and `gain_aug(5)`, and reject poles with magnitude at least one. Implement:

```matlab
function [control, next_state] = lqi_step( ...
        controller_state, plant_state, x_reference, params)
position_error = plant_state(1) - x_reference;
integral = controller_state.position_integral;
u_raw = -params.state_gain * plant_state(:) ...
    - params.integral_gain * integral;
next_integral = integral + params.sample_time * position_error;
next_integral = min(max(next_integral, ...
    -params.position_integral_limit), params.position_integral_limit);
control = struct("u_raw", u_raw, "theta_reference", 0.0, ...
    "position_error", position_error, "position_integral", integral, ...
    "diagnostics", struct("position_integral", integral));
next_state = struct("position_integral", next_integral);
end
```

Add `LQI` lifecycle branches. Commit the pending integral after common saturation without back-calculation.

- [ ] **Step 4: Run LQI, LQR, and legacy tests**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_lqi_controller.m','tests/test_lqr_controller.m','tests/test_legacy_controller_adapters.m'}); assertSuccess(r)"
```

Expected: all tests pass.

- [ ] **Step 5: Commit LQI**

```powershell
git add controllers/lqi_step.m controllers/controller_parameter_space.m controllers/decode_controller_vector.m controllers/create_controller.m controllers/reset_controller.m controllers/controller_step.m controllers/controller_after_actuation.m tests/test_lqi_controller.m
git commit -m "feat: add discrete LQI controller"
```

### Task 4: Fixed fuzzy rule base and inference

**Files:**
- Create: `controllers/fuzzy_pid_rule_base.m`
- Create: `controllers/fuzzy_pid_inference.m`
- Create: `tests/test_fuzzy_pid_inference.m`

**Interfaces:**
- Consumes: normalized finite scalars `error_normalized` and `rate_normalized`.
- Produces: three finite normalized adjustments in `[-1,1]` and diagnostics containing seven memberships per input and total firing strength.

- [ ] **Step 1: Write failing fuzzy inference tests**

```matlab
function test_rule_matrices_are_fixed_symmetric_and_bounded(test_case)
rules = fuzzy_pid_rule_base();
verifyEqual(test_case, size(rules.delta_kp), [7, 7]);
verifyEqual(test_case, size(rules.delta_ki), [7, 7]);
verifyEqual(test_case, size(rules.delta_kd), [7, 7]);
verifyLessThanOrEqual(test_case, max(abs(rules.delta_kp), [], "all"), 3);
verifyEqual(test_case, rules.delta_kp, rot90(rules.delta_kp, 2));
verifyEqual(test_case, rules.delta_ki, rot90(rules.delta_ki, 2));
verifyEqual(test_case, rules.delta_kd, rot90(rules.delta_kd, 2));
end

function test_inference_is_bounded_symmetric_and_deterministic(test_case)
[first, diagnostics] = fuzzy_pid_inference(0.35, -0.20);
second = fuzzy_pid_inference(0.35, -0.20);
opposite = fuzzy_pid_inference(-0.35, 0.20);
verifyEqual(test_case, second, first);
verifyEqual(test_case, opposite, first, "AbsTol", 1e-14);
verifyLessThanOrEqual(test_case, max(abs(struct2array(first))), 1);
verifyEqual(test_case, sum(diagnostics.error_membership), 1, ...
    "AbsTol", 1e-14);
verifyGreaterThan(test_case, diagnostics.total_firing_strength, 0);
end
```

- [ ] **Step 2: Run fuzzy inference tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_fuzzy_pid_inference.m'); assertSuccess(r)"
```

Expected: failure because the fuzzy functions do not exist.

- [ ] **Step 3: Implement the fixed rule matrices and inference**

Generate the immutable singleton-index matrices from `indices = -3:3`:

```matlab
[error_index, rate_index] = ndgrid(-3:3, -3:3);
delta_kp = min(3, round((abs(error_index) + abs(rate_index)) / 2));
delta_ki = max(-3, min(3, ...
    3 - abs(error_index) - abs(rate_index)));
delta_kd = max(-3, min(3, ...
    abs(rate_index) - abs(error_index)));
```

Implement seven piecewise-linear memberships centered at `linspace(-1,1,7)`. Clip inputs to `[-1,1]`, use triangular shoulder functions at both ends, and normalize each seven-value membership vector to sum to one. For every pair, use `min(mu_error(i),mu_rate(j))` as firing strength. Defuzzify each singleton rule matrix with:

```matlab
normalized_output = sum(firing .* rule_matrix, "all") ...
    / (3 * sum(firing, "all"));
```

Return `struct("delta_kp",...,"delta_ki",...,"delta_kd",...)`. Reject nonfinite inputs with `twsbr:fuzzy:invalid_input`.

- [ ] **Step 4: Run fuzzy inference tests and code analysis**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_fuzzy_pid_inference.m'); assertSuccess(r); assert(isempty(checkcode('controllers/fuzzy_pid_inference.m','-id')))"
```

Expected: tests pass and `checkcode` returns no messages.

- [ ] **Step 5: Commit fuzzy inference**

```powershell
git add controllers/fuzzy_pid_rule_base.m controllers/fuzzy_pid_inference.m tests/test_fuzzy_pid_inference.m
git commit -m "feat: add fixed fuzzy PID inference"
```

### Task 5: Fuzzy cascade PID controller

**Files:**
- Create: `controllers/fuzzy_pid_step.m`
- Create: `tests/test_fuzzy_pid_step.m`
- Modify: `controllers/controller_parameter_space.m`
- Modify: `controllers/decode_controller_vector.m`
- Modify: `controllers/create_controller.m`
- Modify: `controllers/reset_controller.m`
- Modify: `controllers/controller_step.m`
- Modify: `controllers/controller_after_actuation.m`

**Interfaces:**
- Consumes: nine-value vector, state with `position_integral` and `theta_integral`, plant state, and position reference.
- Produces: `u_raw`, attitude reference, effective inner gains, fuzzy adjustments, errors, and pending integrals.

- [ ] **Step 1: Write failing fuzzy controller tests**

```matlab
function test_fuzzy_pid_online_gains_are_finite_and_bounded(test_case)
plant = twsbr_params();
config = experiment_config("quick");
vector = [log10([0.24, 0.0004, 0.19, 9.25, 0.05, 1.01]), ...
    0.2, 0.2, 0.2];
params = decode_controller_vector("FUZZY_PID", vector, plant, config);
state = struct("position_integral", 0.0, "theta_integral", 0.0);
[control, next_state] = fuzzy_pid_step(state, ...
    [0; 0; deg2rad(4); 0], 0.5, params);
verifyTrue(test_case, isfinite(control.u_raw));
verifyGreaterThan(test_case, control.kp_theta, 0);
verifyGreaterThanOrEqual(test_case, control.ki_theta, 0);
verifyGreaterThanOrEqual(test_case, control.kd_theta, 0);
verifyLessThanOrEqual(test_case, control.kp_theta, params.kp_theta_max);
verifyTrue(test_case, all(isfinite(struct2array(next_state))));
end

function test_fuzzy_pid_zero_state_has_zero_command(test_case)
plant = twsbr_params();
config = experiment_config("quick");
vector = [log10([0.24, 0.0004, 0.19, 9.25, 0.05, 1.01]), ...
    0.2, 0.2, 0.2];
controller = reset_controller(create_controller( ...
    "FUZZY_PID", vector, plant, config));
[control, ~] = controller_step(controller, 0, zeros(4, 1), 0);
verifyEqual(test_case, control.u_raw, 0, "AbsTol", 1e-14);
end
```

- [ ] **Step 2: Run fuzzy controller tests and verify failure**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_fuzzy_pid_step.m'); assertSuccess(r)"
```

Expected: failure because fuzzy decoding and controller stepping are absent.

- [ ] **Step 3: Implement fuzzy controller and lifecycle branch**

Decode six gains through `10.^vector(1:6)` and use linear `alpha_p`, `alpha_i`, `alpha_d` from `vector(7:9)`. Fixed parameters are:

```matlab
params.theta_error_normalizer = deg2rad(12);
params.theta_rate_normalizer = deg2rad(120);
params.kp_theta_max = 4 * params.kp_theta_base;
params.ki_theta_max = 4 * params.ki_theta_base;
params.kd_theta_max = 4 * params.kd_theta_base;
```

The step computes the existing cascade outer-loop equations, normalizes `theta_error` and `theta_dot`, calls `fuzzy_pid_inference`, applies bounded online gains, and calculates:

```matlab
u_raw = kp_theta * theta_error ...
    + ki_theta * controller_state.theta_integral ...
    + kd_theta * theta_dot;
```

Propose `position_integral + sample_time*position_error`. Propose the theta integral only when predicted common saturation is absent or `theta_error*u_raw < 0`, then clamp both integrals. Add the `FUZZY_PID` branches to factory, reset, step, and after-actuation lifecycle functions.

- [ ] **Step 4: Run all new-controller unit tests**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests({'tests/test_fuzzy_pid_step.m','tests/test_fuzzy_pid_inference.m','tests/test_lqr_controller.m','tests/test_lqi_controller.m','tests/test_legacy_controller_adapters.m'}); assertSuccess(r)"
```

Expected: all tests pass.

- [ ] **Step 5: Commit fuzzy cascade PID**

```powershell
git add controllers/fuzzy_pid_step.m controllers/controller_parameter_space.m controllers/decode_controller_vector.m controllers/create_controller.m controllers/reset_controller.m controllers/controller_step.m controllers/controller_after_actuation.m tests/test_fuzzy_pid_step.m
git commit -m "feat: add fuzzy self-tuning cascade PID"
```

### Task 6: Five-controller factory and common simulator contract

**Files:**
- Create: `tests/test_all_controller_factory.m`
- Modify: `simulation/simulate_control_system.m`

**Interfaces:**
- Consumes: all five controller names and in-bound starter vectors.
- Produces: the same common result schema for every controller without controller-specific branches in the simulator.

- [ ] **Step 1: Write a failing all-controller contract test**

```matlab
function test_all_controllers_produce_common_finite_schema(test_case)
plant = twsbr_params();
config = experiment_config("quick");
vectors = struct( ...
    "ATTITUDE_PID", log10([1.9, 0.2, 0.18]), ...
    "CASCADE_PID", log10([0.241, 0.000396, 0.193, 9.255, 1.011]), ...
    "FUZZY_PID", [log10([0.241, 0.000396, 0.193, 9.255, 0.05, 1.011]), 0.2, 0.2, 0.2], ...
    "LQR", log10([10, 1, 200, 10, 0.1]), ...
    "LQI", log10([10, 1, 200, 10, 100, 0.1]));
scenario = training_scenarios(0.1).T1_initial_tilt_5deg;
for name = config.controller_names.'
    result = simulate_control_system(name, vectors.(name), ...
        plant, config, scenario, 11);
    verifyEqual(test_case, result.controller_name, name);
    verifyTrue(test_case, all(isfinite(result.u_raw)));
    verifyTrue(test_case, all(abs(result.u) <= plant.u_max + 1e-12));
    verifyEqual(test_case, size(result.state, 2), 4);
end
end
```

- [ ] **Step 2: Run the contract test**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests/test_all_controller_factory.m'); assertSuccess(r)"
```

Expected on the first run: focused failures where the simulator assumes legacy-only diagnostics or after-actuation behavior.

- [ ] **Step 3: Remove legacy-only assumptions from the common simulator**

Keep controller-specific logic inside lifecycle functions. The simulator must only read:

```matlab
held_control.u_raw
held_control.theta_reference
held_control.diagnostics
```

Apply the same saturation and RK4 call for all names. Preallocate diagnostics as a cell array when controller-specific structures have different fields.

- [ ] **Step 4: Run the complete test suite**

```powershell
matlab -batch "cd('D:/Research/srtp'); setup_project; r=runtests('tests'); assertSuccess(r)"
```

Expected: all tests pass, including machine-precision legacy equivalence.

- [ ] **Step 5: Commit the five-controller contract**

```powershell
git add simulation/simulate_control_system.m tests/test_all_controller_factory.m
git commit -m "test: verify five-controller simulation contract"
```

## Completion gate

Do not start optimization or batch experiments until all five controllers produce the common result schema, the LQR/LQI stability checks pass, fuzzy gains remain bounded, and every legacy equivalence test still passes.
