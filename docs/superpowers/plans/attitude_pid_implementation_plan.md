# Basic Attitude PID Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build, verify, document, and publish matching MATLAB and Simulink implementations of a discrete basic attitude PID that keeps the robot upright without position control.

**Architecture:** A validated PID parameter structure feeds a pure discrete controller step function. A fixed step RK4 simulator closes this controller around the existing nonlinear plant, while a generated Simulink model uses the same gains, sample time, sign convention, saturation, and conditional integration rule.

**Tech Stack:** MATLAB, Simulink, Stateflow MATLAB Function blocks, MATLAB Unit Test Framework, Git.

## Global Constraints

- Create all deliverables under `D:\Research\srtp`.
- Use English names beginning with a letter and containing only letters, digits, and underscores.
- Use `lower_snake_case` for MATLAB functions, variables, model blocks, signals, tests, and result names.
- Control only `theta` and `theta_dot`; do not close position or velocity feedback.
- Use `theta_error = theta - theta_reference` and `theta_reference = 0` by default.
- Produce `u_raw` before one shared saturation and `u` after saturation.
- Use `sample_time = 0.01 s`, `plant_step = 0.001 s`, and plant `u_max = 1.0`.
- Use conditional integration and clamp `integral_error` to `+/-integral_limit`.
- Keep all plant calculations in SI units.
- Do not change plant parameters or acceptance scenarios when tuning PID gains.
- Complete the full red, green, refactor cycle for each production function.
- After full verification, commit locally and push the current branch to `origin`.

---

### Task 1: PID parameters and one step controller

**Files:**
- Create: `attitude_pid_params.m`
- Create: `attitude_pid_step.m`
- Create: `tests/test_attitude_pid_controller.m`

**Interfaces:**
- Produces: `pid_params = attitude_pid_params(overrides, plant_params)`.
- Produces: `[control, next_state] = attitude_pid_step(controller_state, theta, theta_dot, theta_reference, pid_params)`.
- `controller_state` is a scalar structure with field `integral_error`.
- `control` contains `theta_error`, `integral_error`, `u_raw`, `u`, and `saturated`.

- [ ] **Step 1: Write failing parameter and controller behavior tests**

Tests use literal expected values for proportional, integral, and derivative signs, verify `u` stays inside `+/-u_max`, verify integration stops when error increases saturation, verify integration resumes when error reduces saturation, and verify invalid or unknown parameters raise stable error identifiers.

- [ ] **Step 2: Run the focused tests and verify RED**

```powershell
matlab -batch "cd('D:/Research/srtp'); addpath(pwd); results=runtests('tests/test_attitude_pid_controller.m'); assertSuccess(results)"
```

Expected: failure because both PID functions are undefined.

- [ ] **Step 3: Implement parameter defaults and validation**

Defaults are `kp=1.90`, `ki=0.20`, `kd=0.18`, `sample_time=0.01`, `plant_step=0.001`, `integral_limit=0.50`, and `u_max=plant_params.u_max`. Gains must be finite and nonnegative; time steps, integral limit, and actuator limit must be positive; `sample_time / plant_step` must be an integer within `1e-12`.

- [ ] **Step 4: Implement one deterministic PID step**

Compute raw and applied command from the current integral state, use saturation tolerance `1e-12`, update the integral only when unsaturated or `theta_error * u_raw < 0`, and clamp the next integral state.

- [ ] **Step 5: Run the focused tests and verify GREEN**

Expected: all controller tests pass without warnings.

### Task 2: Fixed step nonlinear simulation

**Files:**
- Create: `twsbr_rk4_step.m`
- Create: `simulate_attitude_pid.m`
- Create: `tests/test_attitude_pid_simulation.m`

**Interfaces:**
- Produces: `next_state = twsbr_rk4_step(state, u, step_size, plant_params, force_disturbance, torque_disturbance)`.
- Produces: `simulation = simulate_attitude_pid(plant_params, pid_params, scenario)`.
- Scenario fields are `name`, `initial_state`, `duration`, `theta_reference`, `force_disturbance`, and `torque_disturbance` function handles.
- Simulation fields include `time`, `state`, `theta_reference`, `u_raw`, `u`, `saturated`, `integral_error`, `force_disturbance`, `torque_disturbance`, `success`, `failure_reason`, and `metrics`.

- [ ] **Step 1: Write failing RK4 and closed loop tests**

Verify RK4 preserves the zero equilibrium, agrees with `ode45` for one short uncontrolled interval, keeps every logged array aligned, leaves zero state at zero, stabilizes positive and negative five degree initial tilt, respects actuator and integral limits, and records finite values.

- [ ] **Step 2: Run the focused tests and verify RED**

Expected: failure because RK4 and closed loop simulation functions are undefined.

- [ ] **Step 3: Implement RK4 using `twsbr_dynamics` for all four stages**

Hold control and disturbances constant within a plant step and validate positive finite step size.

- [ ] **Step 4: Implement the multirate simulator and metrics**

Update the PID every ten plant steps, hold `u` between controller samples, apply scenario disturbances at every plant step, stop on tilt beyond `theta_fail_deg`, position beyond `x_limit`, or nonfinite state, and compute all metrics defined by the design specification.

- [ ] **Step 5: Run simulation tests and verify GREEN**

If the initial gains miss a behavioral criterion, record the failing metric, adjust only `kp`, `ki`, or `kd`, and rerun the same unchanged test.

### Task 3: Acceptance scenarios and result figure

**Files:**
- Create: `attitude_pid_scenarios.m`
- Create: `plot_attitude_pid_results.m`
- Create: `tests/test_attitude_pid_acceptance.m`

**Interfaces:**
- Produces: `scenarios = attitude_pid_scenarios()` as a scalar structure of named scenario structures.
- Produces: `figure_handle = plot_attitude_pid_results(simulation)`.

- [ ] **Step 1: Write failing scenario and acceptance tests**

Assert exact scenario durations, initial angles, and torque impulse timing and magnitude. Verify zero state, positive tilt, negative tilt, large tilt, and torque impulse criteria from the design specification.

- [ ] **Step 2: Run focused tests and verify RED**

Expected: failure because scenario and plot functions are undefined.

- [ ] **Step 3: Implement immutable scenario definitions**

Use zero force disturbance in every scenario and a `0.05 N m` torque impulse only when `1.00 <= time && time <= 1.05`.

- [ ] **Step 4: Implement the four panel result figure**

Plot tilt, angular velocity, raw and applied control, and wheel position with English labels, invisible batch figure creation, and local suppression of only `MATLAB:uicontainer:ScrollableOnWithTextScaling`.

- [ ] **Step 5: Run acceptance tests and verify GREEN**

Expected: every deterministic MATLAB scenario meets the frozen criteria.

### Task 4: Generated Simulink closed loop model

**Files:**
- Create: `build_attitude_pid_simulink.m`
- Create: `run_attitude_pid_simulink.m`
- Create: `private/create_attitude_pid_simulink_model.m`
- Create through builder: `twsbr_attitude_pid.slx`
- Create: `tests/test_attitude_pid_simulink.m`

**Interfaces:**
- Produces: `model_path = build_attitude_pid_simulink(plant_params, pid_params)`.
- Produces: `simulation = run_attitude_pid_simulink(plant_params, pid_params, scenario)` using the same aligned output field names as the MATLAB simulation where applicable.

- [ ] **Step 1: Write failing build, warning, saturation, and trajectory tests**

Verify the SLX exists, updates, runs zero and positive tilt cases, rebuilds warning free, all block names satisfy `^[A-Za-z][A-Za-z0-9_]*$`, applied control respects limits, and positive tilt differs from MATLAB by less than `0.2 deg` after interpolation.

- [ ] **Step 2: Run focused tests and verify RED**

Expected: failure because Simulink builder and runner are undefined.

- [ ] **Step 3: Implement the model builder**

Copy the existing `nonlinear_plant` subsystem from `twsbr_plant.slx`, add a sampled state path, a discrete MATLAB Function PID block with persistent integral state, an external saturation block, disturbance sources, English logging blocks, and variable step `ode45` plant solver with `MaxStep=0.001`.

- [ ] **Step 4: Implement the Simulink runner and normalize logged shapes**

Set initial state, duration, reference, disturbances, PID constants, and integral initial value without undeclared workspace variables. Return every time series as `N by width` arrays.

- [ ] **Step 5: Run Simulink tests and verify GREEN**

Expected: build, run, warning, naming, bounds, and trajectory tests pass.

### Task 5: One command workflow, documentation, and artifacts

**Files:**
- Create: `run_attitude_pid.m`
- Modify: `README.md`
- Create through execution: `results/attitude_pid_results.mat`
- Create through execution: `results/attitude_pid_response.png`
- Create: `tests/test_attitude_pid_project.m`

**Interfaces:**
- Produces: `summary = run_attitude_pid(run_tests_flag)` containing paths, parameters, scenario metrics, and optional test results.

- [ ] **Step 1: Write a failing end to end artifact test**

Call `run_attitude_pid(false)` and assert the MAT, PNG, and SLX paths exist, have nonzero size, include the final PID parameters, and report every acceptance scenario as successful.

- [ ] **Step 2: Run the focused test and verify RED**

Expected: failure because the workflow function is undefined.

- [ ] **Step 3: Implement workflow and README updates**

Run all five scenarios, run the Simulink positive tilt case, export the positive tilt figure, save all parameters, trajectories, diagnostics, metrics, and comparison values, and document the sign convention and position control limitation.

- [ ] **Step 4: Run the end to end test and verify GREEN**

Expected: all expected artifacts are generated with accepted metrics.

### Task 6: Full verification and Git delivery

**Files:**
- Verify all project `.m`, `.slx`, `.mat`, `.png`, `.md`, and test files.

- [ ] **Step 1: Run the complete fresh MATLAB verification**

```powershell
matlab -batch "cd('D:/Research/srtp'); addpath(pwd); summary=run_attitude_pid(true); assert(isfile(summary.model_path)); assert(isfile(summary.results_path)); assert(isfile(summary.figure_path));"
```

- [ ] **Step 2: Run code, naming, and artifact audits**

Require zero `checkcode` issues, English file and block names, readable nonblank PNG output, finite MAT data, clean Simulink rebuild, and no generated `slprj` cache in the commit.

- [ ] **Step 3: Review the complete Git diff**

Confirm every changed file belongs to the attitude PID scope and no unrelated user change is staged.

- [ ] **Step 4: Commit and push**

```powershell
git add -A
git commit -m "feat: add basic attitude PID control"
git push origin main
```

- [ ] **Step 5: Verify remote synchronization**

Require clean `git status --short`, matching `HEAD` and `origin/main`, and report the final commit hash and validation counts.
