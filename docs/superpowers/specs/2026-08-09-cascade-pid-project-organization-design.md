# Cascade PID and Project Organization Design

## 1. Goal

Add a MATLAB and Simulink cascade controller to the existing two wheeled self balancing robot project and reorganize every current implementation file by purpose without breaking existing entry commands or MATLAB path resolution.

The cascade controller follows the provided sample project at `D:\Research\EI_Meeting\twsbr_control_study`: a position PID outer loop creates a bounded body tilt reference, and an attitude PD inner loop creates the unsaturated motor command. The existing nonlinear plant, actuator convention, state order, and verified attitude-only PID remain unchanged in behavior.

This phase does not implement steering, yaw control, sensor fusion, embedded deployment, automatic gain optimization, Monte Carlo experiments, or comparisons with LQR and LQI.

## 2. Reference implementation findings

The sample controller in `controllers/cascade_pid.py` uses:

```text
position_error = x_reference - x
theta_reference = clip(
    kp_x * position_error
  + ki_x * position_integral
  - kd_x * x_dot,
    -theta_reference_limit,
    theta_reference_limit)

theta_error = theta - theta_reference
u_raw = kp_theta * theta_error + kd_theta * theta_dot
```

One common actuator saturation rule produces `u` from `u_raw`. The position integral is updated after actuator saturation without anti-windup feedback. The sample separates models, controllers, experiments, visualization, tests, and outputs into purpose-based directories. Both the control structure and organization principle are retained in the MATLAB design.

## 3. Naming rules

- All new files, folders, functions, variables, Simulink models, blocks, signals, tests, and result files use English names.
- MATLAB files and functions use `lower_snake_case` and each primary function name matches its filename.
- Names begin with an English letter and contain only English letters, digits, and underscores, except standard repository files such as `.gitignore` and `README.md`.
- Purpose directory names are plural English nouns where appropriate.

## 4. Project organization

The project root retains only user-facing entry points and repository metadata:

```text
srtp/
|-- .gitignore
|-- README.md
|-- setup_project.m
|-- run_project.m
|-- run_attitude_pid.m
|-- run_cascade_pid.m
|-- models/
|-- controllers/
|-- simulation/
|-- scenarios/
|-- builders/
|   `-- private/
|-- simulink_models/
|-- visualization/
|-- workflows/
|-- tests/
|-- results/
`-- docs/
```

### 4.1 File responsibilities

`models/` contains physical plant parameters, nonlinear dynamics, linearization, and numerical integration:

```text
twsbr_params.m
twsbr_dynamics.m
twsbr_linear_model.m
twsbr_numerical_linearize.m
twsbr_rk4_step.m
```

`controllers/` contains controller parameters and deterministic controller update functions:

```text
attitude_pid_params.m
attitude_pid_step.m
cascade_pid_params.m
cascade_pid_step.m
```

`simulation/` contains MATLAB simulations and Simulink execution adapters:

```text
simulate_open_loop.m
simulate_attitude_pid.m
simulate_cascade_pid.m
run_simulink_open_loop.m
run_attitude_pid_simulink.m
run_cascade_pid_simulink.m
```

`scenarios/` contains immutable scenario factories:

```text
attitude_pid_scenarios.m
cascade_pid_scenarios.m
```

`builders/` contains public reproducible Simulink builders. `builders/private/` contains their private model creation functions:

```text
build_twsbr_simulink.m
build_attitude_pid_simulink.m
build_cascade_pid_simulink.m
private/create_twsbr_simulink_model.m
private/create_attitude_pid_simulink_model.m
private/create_cascade_pid_simulink_model.m
```

`simulink_models/` contains generated `.slx` artifacts:

```text
twsbr_plant.slx
twsbr_attitude_pid.slx
twsbr_cascade_pid.slx
```

`visualization/` contains response plotting functions:

```text
plot_attitude_pid_results.m
plot_cascade_pid_results.m
```

`workflows/` contains the implementation behind stable root entry points:

```text
run_project_workflow.m
run_attitude_pid_workflow.m
run_cascade_pid_workflow.m
```

`tests/`, `results/`, and `docs/` retain their existing purposes. Future plant models, controllers, simulations, scenarios, builders, visualizations, workflows, tests, and artifacts must be added to the corresponding purpose directory rather than the repository root.

## 5. Path compatibility

`setup_project.m` is the only path registration function. It derives the absolute project root from its own file location and explicitly adds only:

```text
models
controllers
simulation
scenarios
builders
visualization
workflows
```

It does not use unrestricted `genpath` and does not add `builders/private`, `results`, `simulink_models`, generated caches, or documentation to the MATLAB path.

The root functions `run_project.m`, `run_attitude_pid.m`, and `run_cascade_pid.m` keep their public names. Each calls `setup_project.m` before dispatching to its uniquely named workflow function. Existing commands remain valid:

```matlab
run_project
run_attitude_pid
run_cascade_pid
```

Builders and workflows derive the project root from their own absolute file paths and use absolute paths for SLX, MAT, and PNG files. Tests initialize the project path explicitly. A path regression test changes to a temporary external working directory, places only the repository root on the MATLAB path, and verifies that each root entry point can locate every internal dependency and artifact.

## 6. Cascade controller

### 6.1 State and references

The existing plant state remains:

```text
[x; x_dot; theta; theta_dot]
```

The cascade controller accepts `x_reference` and reads all four plant states. Position feedback is used only by the outer loop. Tilt and angular velocity are used by the inner loop.

### 6.2 Outer position PID

```text
position_error = x_reference - x

theta_reference_raw =
    kp_x * position_error
  + ki_x * position_integral
  - kd_x * x_dot

theta_reference = clip(
    theta_reference_raw,
    -theta_reference_limit,
    theta_reference_limit)
```

The derivative term uses measured `x_dot`; it does not numerically differentiate position error. The outer integral is updated once per controller sample after common actuator saturation:

```text
position_integral_next = position_integral
                       + sample_time * position_error
```

To match the sample, actuator saturation does not feed an anti-windup correction into the outer integrator. A symmetric `position_integral_limit = 1e6 m s` provides only a numerical overflow guard.

### 6.3 Inner attitude PD

```text
theta_error = theta - theta_reference
u_raw = kp_theta * theta_error + kd_theta * theta_dot
u = clip(u_raw, -u_max, u_max)
```

The inner loop intentionally has no integral term. The actuator saturation remains external to the controller in MATLAB and Simulink.

### 6.4 Timing and limits

```text
sample_time = 0.01 s
plant_step = 0.001 s
theta_reference_limit = 12 deg
u_max = plant_params.u_max = 1.0
```

The MATLAB simulator updates the controller every ten plant steps and holds the command between controller samples. Simulink uses the same discrete controller period and a continuous nonlinear plant with maximum solver step `0.001 s`.

### 6.5 Initial gains

The sample's verified frozen PID values are used as initial gains:

```text
kp_x = 0.24100028146267993
ki_x = 0.0003962067755988572
kd_x = 0.1930824033173246
kp_theta = 9.254929149177556
kd_theta = 1.0113335430173094
```

If the frozen MATLAB scenarios fail, only these five gains may change. Plant parameters, timing, reference and actuator limits, scenarios, and acceptance thresholds remain fixed. Final gain values are recorded in the README and MAT result.

### 6.6 Controller interface and diagnostics

```matlab
params = cascade_pid_params(overrides, plant_params)
[control, next_state] = cascade_pid_step( ...
    controller_state, plant_state, x_reference, params)
```

`controller_state` contains `position_integral`. `control` contains:

```text
position_error
position_integral
theta_reference_raw
theta_reference
theta_error
u_raw
u
saturated
```

The input state, controller state, reference, parameters, and outputs must be finite. Parameter overrides reject unknown fields. Gains are finite and nonnegative; time steps and limits are positive; `sample_time / plant_step` is an integer.

## 7. MATLAB simulation

`simulate_cascade_pid.m` reuses `twsbr_rk4_step.m` and the existing nonlinear dynamics. Its scenario interface contains:

```text
name
initial_state
duration
x_reference
force_disturbance
torque_disturbance
```

The simulation records aligned time, state, references, controller diagnostics, disturbances, success status, failure reason, and metrics. It fails on nonfinite state, body tilt beyond `theta_fail_deg`, or position beyond `x_limit` because position regulation is an objective of the cascade controller.

Metrics include maximum and final tilt, maximum and final position error, position settling time, disturbance recovery time, integral absolute position error, integral absolute tilt error, maximum raw control, saturation duration, maximum tilt reference, and maximum position integral.

## 8. Frozen scenarios

`cascade_pid_scenarios.m` returns exactly five scenarios:

1. `zero_state`: zero state and zero reference for `2 s`.
2. `positive_position_step`: reference changes from `0 m` to `+0.5 m` at `1 s`; duration `8 s`.
3. `negative_position_step`: reference changes from `0 m` to `-0.5 m` at `1 s`; duration `8 s`.
4. `initial_tilt`: initial tilt `+5 deg`, zero position reference; duration `5 s`.
5. `force_impulse`: reference changes to `+0.5 m` at `1 s`; a `5 N` horizontal force acts for `4.0 <= time < 4.2 s`; duration `8 s`.

All unspecified disturbances and references are zero.

## 9. Acceptance criteria

All scenarios must:

- keep `abs(theta) <= 30 deg`;
- keep state, references, controller diagnostics, disturbances, and metrics finite;
- keep `abs(theta_reference) <= 12 deg + 1e-12`;
- keep `abs(u) <= u_max + 1e-12`;
- avoid `abs(x) > x_limit`.

The positive and negative position step scenarios must:

- finish with absolute position error below `0.05 m`;
- enter and remain inside an absolute position error band of `0.05 m` no later than `5 s` after the step begins;
- finish with absolute body tilt below `0.5 deg`.

The initial tilt scenario must finish with absolute position error below `0.05 m` and absolute body tilt below `0.5 deg`.

The force impulse scenario must return to and remain inside an absolute position error band of `0.10 m` no later than `3 s` after the impulse ends.

The MATLAB and Simulink positive position step trajectories must differ by less than `0.2 deg` in body tilt and `0.01 m` in position after interpolation to common timestamps.

## 10. Simulink architecture

The generated model is:

```text
simulink_models/twsbr_cascade_pid.slx
```

Its root data flow is:

```text
x_reference
    -> position_pid
    -> theta_reference
    -> attitude_pd
    -> u_raw
    -> actuator_saturation
    -> nonlinear_plant
    -> sampled_state feedback
```

The builder copies the existing nonlinear plant subsystem, updates its embedded plant parameters, and adds a discrete MATLAB Function cascade controller. The controller script uses the same gains, timing, limits, equations, integral update order, and diagnostics as MATLAB. Every new block and root signal has an English MATLAB-compatible name. Scenario inputs are provided through the model workspace without undeclared base-workspace variables.

## 11. Result artifacts and workflow

`run_cascade_pid.m` calls `setup_project.m` and dispatches to `run_cascade_pid_workflow.m`. The workflow:

1. creates validated plant and cascade controller parameters;
2. runs all five MATLAB scenarios;
3. builds the cascade Simulink model;
4. runs the positive position step in Simulink;
5. checks acceptance and MATLAB/Simulink agreement;
6. exports the response figure;
7. saves all parameters, trajectories, diagnostics, metrics, acceptance decisions, and comparison values;
8. optionally runs the complete test suite.

It creates:

```text
results/cascade_pid_results.mat
results/cascade_pid_response.png
simulink_models/twsbr_cascade_pid.slx
```

The response figure has five panels: position and reference, body tilt and tilt reference, angular velocity, raw and applied control, and position integral state.

## 12. Migration and testing

The directory migration and cascade controller are implemented test-first. Migration tests freeze the public root entry names and artifact locations before moving files. After each file group moves, focused tests verify resolution with `which`, function execution, builder private-function access, and artifact paths.

New tests cover:

- explicit setup paths and exclusion of private, result, model artifact, and cache directories;
- root entry compatibility from the project root and from an external working directory with only the root on the path;
- all existing plant and attitude PID behavior after migration;
- cascade parameter validation and unknown override rejection;
- literal outer PID and inner PD signs and terms;
- reference and actuator saturation;
- integral update, reset, bounds, and deterministic repeated execution;
- multirate MATLAB simulation and aligned diagnostics;
- exact scenario definitions and every acceptance criterion;
- Simulink warning-free build, English block and signal names, limits, and MATLAB agreement;
- generation and readability of MAT, PNG, and SLX artifacts;
- zero MATLAB Code Analyzer findings for all project `.m` files;
- absence of generated `slprj` and `.slxc` caches from Git.

The complete preexisting 45-test suite must remain green after migration. New cascade, path, and organization tests are added rather than replacing existing coverage.

## 13. Documentation and Git delivery

`README.md` is updated with the classified project tree, setup behavior, preserved entry commands, cascade equations, initial or tuned final gains, acceptance scenarios, result artifacts, and the distinction between software simulation and physical deployment.

After implementation, the complete MATLAB and Simulink suite runs in a fresh batch session. Code analysis, English naming, model rebuild, artifact readability, cache exclusion, Git diff scope, and local/remote synchronization are verified. The completed implementation is committed locally and pushed to the configured GitHub `origin` on `main`, following the established project delivery preference.
