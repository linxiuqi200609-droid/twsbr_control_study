# Two Wheeled Self Balancing Robot Plant Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and verify matching MATLAB and Simulink longitudinal plant models for a two wheeled self balancing robot.

**Architecture:** A shared parameter function feeds a nonlinear four state cart pole plant and an analytical linearization. MATLAB scripts provide open loop simulation and verification, while a builder script creates a Simulink model with the same equations and compares its trajectory against the MATLAB solver.

**Tech Stack:** MATLAB, Simulink, MATLAB Unit Test Framework.

## Global Constraints

- Create all deliverables under `D:\Research\srtp`.
- All new file, folder, model, function, variable, test, signal, and result names use English.
- Names start with an English letter and contain only English letters, digits, and underscores.
- MATLAB code uses `lower_snake_case`; primary function names exactly match their `.m` filenames.
- Do not use spaces, hyphens, parentheses, Chinese characters, or other special characters in names.
- Use the state order `[x; x_dot; theta; theta_dot]`, with `theta = 0` at the upright equilibrium.
- The plant accepts normalized control `u`, horizontal disturbance force, and body disturbance torque.
- The plant itself does not clip `u`; actuator saturation remains external.
- Internal calculations use SI units.

---

### Task 1: Plant parameters and nonlinear dynamics

**Files:**
- Create: `twsbr_params.m`
- Create: `twsbr_dynamics.m`
- Create: `tests/test_twsbr_model.m`

**Interfaces:**
- Produces: `params = twsbr_params(overrides)` returning a validated scalar structure.
- Produces: `state_dot = twsbr_dynamics(~, state, u, params, force_disturbance, torque_disturbance)` returning a `4x1` state derivative.

- [ ] **Step 1: Write failing tests for defaults, equilibrium, signs, and invalid parameters**

Create MATLAB function based tests that assert the documented parameter values, zero equilibrium derivative, positive angular acceleration from positive tilt, negative angular acceleration from positive control, and a `twsbr:params:invalid_value` error for nonpositive mass.

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```powershell
matlab -batch "cd('D:/Research/srtp'); results=runtests('tests/test_twsbr_model.m'); assertSuccess(results)"
```

Expected: failures because `twsbr_params` and `twsbr_dynamics` do not exist.

- [ ] **Step 3: Implement the default parameter structure and validation**

Use the sample values `body_mass=1.20`, `wheel_mass_equiv=0.30`, `com_length=0.18`, `body_inertia=0.025`, `wheel_radius=0.05`, `viscous_damping=0.05`, `gravity=9.81`, `motor_force_gain=12.0`, `u_max=1.0`, `theta_fail_deg=30.0`, and `x_limit=5.0`. Accept an optional scalar override structure and reject unknown fields.

- [ ] **Step 4: Implement the nonlinear state equation**

Implement the documented mass matrix solution and validate the state length, finite scalar inputs, and positive determinant.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run the same MATLAB command. Expected: all Task 1 tests pass with no warnings.

### Task 2: Analytical and numerical linearization

**Files:**
- Create: `twsbr_linear_model.m`
- Create: `twsbr_numerical_linearize.m`
- Modify: `tests/test_twsbr_model.m`

**Interfaces:**
- Consumes: `twsbr_params`, `twsbr_dynamics`.
- Produces: `[a_matrix,b_matrix,c_matrix,d_matrix] = twsbr_linear_model(params)`.
- Produces: `[a_matrix,b_matrix] = twsbr_numerical_linearize(params,state_zero,u_zero,epsilon)`.

- [ ] **Step 1: Add failing matrix agreement and controllability tests**

Test analytical versus central difference matrices with absolute tolerance `1e-7` and relative tolerance `1e-6`. Build `[B,A*B,A^2*B,A^3*B]` explicitly and assert rank 4.

- [ ] **Step 2: Run tests and verify RED**

Expected: failures because the linearization functions do not exist.

- [ ] **Step 3: Implement the analytical matrices**

Use the upright equilibrium Jacobian derived from the nonlinear equations and return `C=eye(4)` and `D=zeros(4,1)`.

- [ ] **Step 4: Implement central difference linearization**

Use default state `zeros(4,1)`, input `0`, and epsilon `1e-6`, while allowing explicit arguments.

- [ ] **Step 5: Run tests and verify GREEN**

Expected: every non Simulink test passes.

### Task 3: MATLAB open loop simulation

**Files:**
- Create: `simulate_open_loop.m`
- Modify: `tests/test_twsbr_model.m`

**Interfaces:**
- Consumes: `twsbr_params`, `twsbr_dynamics`.
- Produces: `simulation = simulate_open_loop(params,initial_state,stop_time,make_plot)` with fields `time`, `state`, `input`, and `figure_handle`.

- [ ] **Step 1: Add failing equilibrium and falling trajectory tests**

Assert a zero initial state stays below `1e-10` and a 3 degree initial tilt grows in magnitude during a 2 second uncontrolled simulation.

- [ ] **Step 2: Run tests and verify RED**

Expected: failures because `simulate_open_loop` does not exist.

- [ ] **Step 3: Implement ODE simulation and optional four panel plot**

Use `ode45`, zero control and disturbances, SI state storage, and degree conversion only for plots.

- [ ] **Step 4: Run tests and verify GREEN**

Expected: all numerical plant tests pass.

### Task 4: Programmatic Simulink model generation

**Files:**
- Create: `build_twsbr_simulink.m`
- Create: `run_simulink_open_loop.m`
- Create: `twsbr_plant.slx` through the builder
- Modify: `tests/test_twsbr_model.m`

**Interfaces:**
- Consumes: all plant functions and parameters.
- Produces: `model_path = build_twsbr_simulink()`.
- Produces: `simulation = run_simulink_open_loop(params,initial_state,stop_time)` with `time` and `state` fields.

- [ ] **Step 1: Add failing model build and trajectory agreement tests**

Assert the builder returns an existing `.slx`, the model compiles and runs, and its states agree with MATLAB open loop results to maximum absolute error `1e-4` after interpolation to common timestamps.

- [ ] **Step 2: Run tests and verify RED**

Expected: failures because builder and Simulink runner do not exist.

- [ ] **Step 3: Implement the Simulink builder**

Create top level English named ports and signals for `u`, `force_disturbance`, `torque_disturbance`, and `state`. Build a `nonlinear_plant` subsystem with a continuous integrator and a MATLAB Function block containing the same equations. Embed parameters read from `twsbr_params` when building, and rebuild after parameter changes.

- [ ] **Step 4: Implement the Simulink open loop runner**

Set initial state, stop time, zero inputs, solver settings, and signal logging without relying on undeclared base workspace variables.

- [ ] **Step 5: Generate the model and run tests to verify GREEN**

Expected: the model file exists, compiles, runs, and trajectory agreement passes.

### Task 5: One command workflow, artifacts, and documentation

**Files:**
- Create: `run_project.m`
- Create: `README.md`
- Create: `results/open_loop_results.mat` through execution
- Create: `results/open_loop_response.png` through execution
- Modify: `tests/test_twsbr_model.m`

**Interfaces:**
- Consumes: all previous public functions.
- Produces: reproducible test output, generated model, MAT results, and PNG figure.

- [ ] **Step 1: Add a failing end to end smoke test**

Assert that a clean run creates the expected model and result artifacts with nonempty contents.

- [ ] **Step 2: Run the smoke test and verify RED**

Expected: failure because `run_project` and artifacts do not exist.

- [ ] **Step 3: Implement `run_project` and README**

The entry script adds only project paths, runs all tests, builds the model, runs both open loop cases, saves results, exports the four panel response figure, and prints the analytical matrices and controllability rank. README documents equations, assumptions, file roles, invocation, expected outputs, and future PID connection points.

- [ ] **Step 4: Run full verification**

Run:

```powershell
matlab -batch "cd('D:/Research/srtp'); run_project"
```

Expected: exit code 0, all tests pass, `twsbr_plant.slx` exists, and the two result artifacts are nonempty.

- [ ] **Step 5: Inspect generated artifacts and source naming**

List all new files and confirm every basename satisfies `^[A-Za-z][A-Za-z0-9_]*$`. Confirm the saved figure is readable and the model contains only English named blocks and signals.

