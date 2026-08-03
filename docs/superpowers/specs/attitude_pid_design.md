# Basic Attitude PID Design

## 1. Goal

Add a basic attitude PID controller to the existing two wheeled self balancing robot project. The controller stabilizes body tilt at the upright reference while allowing wheel position and wheel velocity to move freely. Matching MATLAB and Simulink implementations use the same plant parameters, PID parameters, sample time, actuator limits, and test scenarios.

This phase does not implement position control, velocity control, cascade PID, steering, yaw control, automatic gain optimization, sensor fusion, or embedded deployment.

## 2. Reference implementation findings

The Python sample project uses a common nonlinear plant, a normalized actuator command, and one shared actuator saturation rule. Its controller named `PID` is a cascade controller: a position PID creates a bounded tilt reference and an attitude PD creates the raw actuator command. The sample uses the sign convention `theta_error = theta - theta_reference` and updates dynamic controller state after actuator saturation.

The MATLAB implementation retains the useful conventions from the sample:

- the nonlinear plant remains shared by every controller;
- the controller produces an unsaturated command `u_raw`;
- one common saturation rule produces `u` in `[-u_max, u_max]`;
- controller state is updated at a fixed sample time;
- simulation records both raw and applied control signals;
- controller and plant failures remain visible in results.

Unlike the sample attitude inner loop, this phase includes a real integral term so the controller is a complete PID rather than a PD controller.

## 3. Naming rules

All new files, folders, functions, variables, tests, Simulink models, blocks, signals, and result files use English names. Names begin with an English letter and contain only English letters, digits, and underscores. MATLAB functions and variables use `lower_snake_case`, and each primary function name exactly matches its `.m` filename.

## 4. Control objective and sign convention

The controlled state is body tilt only:

```text
theta_reference = 0 rad
measured signals = theta, theta_dot
uncontrolled signals = x, x_dot
```

The existing plant produces negative angular acceleration from a positive actuator command. Therefore the controller uses the same sign convention as the sample:

\[
e_\theta = \theta - \theta_{reference}
\]

\[
u_{raw} = K_p e_\theta + K_i I_\theta + K_d \dot{\theta}
\]

\[
u = \operatorname{clip}(u_{raw}, -u_{max}, u_{max})
\]

A positive tilt therefore produces a positive control command, which produces corrective negative angular acceleration.

The derivative contribution uses the measured state `theta_dot` directly. It does not numerically differentiate `theta`, so the basic controller does not require a derivative filter in the ideal simulation model.

## 5. Default PID parameters

The initial parameter set is derived from the upright linear model and then accepted or adjusted only through the defined simulation tests:

| Parameter | Default | Unit | Meaning |
|---|---:|---|---|
| `kp` | 1.90 | 1/rad | proportional gain |
| `ki` | 0.20 | 1/(rad s) | integral gain |
| `kd` | 0.18 | s/rad | angular velocity gain |
| `sample_time` | 0.01 | s | discrete controller period |
| `plant_step` | 0.001 | s | MATLAB nonlinear RK4 integration step |
| `integral_limit` | 0.50 | rad s | symmetric integral state limit |
| `u_max` | 1.00 | 1 | normalized actuator limit inherited from plant parameters |

Final committed gain values may differ from the initial set when required to pass the acceptance scenarios. Any adjustment is recorded in the result file and README. Tests verify behavior rather than a hard coded gain value.

## 6. Discrete controller state update

The controller state contains one scalar:

```text
integral_error
```

At every controller sample:

1. Read `theta`, `theta_dot`, and `theta_reference`.
2. Compute `theta_error = theta - theta_reference`.
3. Compute `u_raw` from the current integral state.
4. Apply the common actuator saturation to obtain `u`.
5. Detect saturation with tolerance `1e-12`.
6. Update the integral state only when the actuator is not saturated or when the current error drives the raw command back toward the unsaturated interval.
7. Clamp the updated integral state to `[-integral_limit, integral_limit]`.

The conditional integration rule is:

```text
integrate when not saturated
or when sign(theta_error) differs from sign(u_raw)
```

This is a basic clamping anti windup method. It prevents the integral state from increasing actuator saturation while retaining integral action during normal recovery.

The controller returns diagnostics containing:

```text
theta_error
integral_error
u_raw
u
saturated
```

## 7. MATLAB architecture

The MATLAB implementation adds focused files:

```text
attitude_pid_params.m
attitude_pid_step.m
twsbr_rk4_step.m
simulate_attitude_pid.m
run_attitude_pid.m
```

Responsibilities:

- `attitude_pid_params.m` returns validated PID parameters and accepts a scalar override structure.
- `attitude_pid_step.m` performs one deterministic discrete PID update and returns the applied control, diagnostics, and next controller state.
- `twsbr_rk4_step.m` advances the existing nonlinear plant by one fixed step while holding control and disturbances constant.
- `simulate_attitude_pid.m` runs the controller at `sample_time` and the plant at `plant_step`, records all state and control signals, detects falling or nonfinite state, and returns metrics.
- `run_attitude_pid.m` executes the acceptance scenarios, builds the Simulink model, saves MAT results, exports figures, and prints a compact summary.

The MATLAB simulator uses SI units internally. Plotting converts angle and angular velocity to degrees and degrees per second.

## 8. Simulink architecture

The generated closed loop model is:

```text
twsbr_attitude_pid.slx
```

The model reuses the nonlinear plant subsystem from `twsbr_plant.slx` and adds:

```text
theta_reference
state_sampling
attitude_pid
actuator_saturation
nonlinear_plant
state_log
u_raw_log
u_log
theta_error_log
integral_error_log
```

Data flow:

```text
state -> state_sampling -> attitude_pid -> actuator_saturation -> nonlinear_plant
  ^                                                                  |
  |__________________________________________________________________|
```

The state sampling block runs at `sample_time`. The PID subsystem is discrete and holds its output between controller updates. The nonlinear plant remains continuous. The actuator saturation block is outside the PID subsystem and uses `u_max` from the shared parameters.

The PID subsystem uses `theta` and `theta_dot`; it does not feed back `x` or `x_dot`. Every block, signal, logged variable, and model name follows the English naming rule.

## 9. Simulation scenarios

The controller is evaluated with frozen PID parameters in the following deterministic scenarios:

1. `zero_state`: zero initial state, zero disturbances, 2 s duration.
2. `positive_tilt`: positive 5 degree initial tilt, 5 s duration.
3. `negative_tilt`: negative 5 degree initial tilt, 5 s duration.
4. `large_tilt`: positive 8 degree initial tilt, 5 s duration.
5. `torque_impulse`: zero initial state with a 0.05 N m body torque from 1.00 s through 1.05 s, 5 s duration.

Position drift is recorded but is not an acceptance failure in this phase.

## 10. Metrics and acceptance criteria

Each simulation records:

- maximum absolute tilt;
- final absolute tilt;
- settling time into a symmetric angle band;
- integral absolute tilt error;
- maximum raw control magnitude;
- saturation duration;
- maximum absolute position drift;
- success flag and failure reason.

Acceptance criteria:

- `zero_state` remains numerically at equilibrium.
- Positive and negative 5 degree initial tilt do not cross the 30 degree falling threshold.
- Positive and negative 5 degree cases enter and remain inside `+/-0.5 deg` within 2.0 s.
- The 8 degree case remains upright and finishes inside `+/-0.5 deg`.
- The torque impulse case returns inside `+/-0.5 deg` within 2.0 s after the impulse ends.
- Applied control never exceeds `u_max + 1e-12`.
- Controller integral state never exceeds `integral_limit + 1e-12`.
- All logged values remain finite.
- MATLAB and Simulink positive tilt trajectories differ by less than 0.2 degree in tilt after interpolation to common timestamps.
- Rebuilding the Simulink model does not produce warnings.

If the initial gains fail an acceptance criterion, only `kp`, `ki`, and `kd` may be adjusted. Scenario definitions, actuator limit, sample time, plant parameters, and thresholds remain fixed.

## 11. Tests

New tests cover:

- PID parameter validation and unknown override rejection;
- proportional, integral, and derivative term signs;
- actuator saturation and conditional integral clamping;
- controller reset and deterministic repeated execution;
- RK4 equilibrium preservation and convergence against the existing ODE model;
- all deterministic MATLAB acceptance scenarios;
- Simulink model build, update, run, and warning free rebuild;
- MATLAB and Simulink tilt trajectory agreement;
- generation of the expected MAT, PNG, and SLX artifacts;
- English naming audit for new files and Simulink blocks.

## 12. Result artifacts

The workflow creates:

```text
results/attitude_pid_results.mat
results/attitude_pid_response.png
twsbr_attitude_pid.slx
```

The response figure contains body tilt, body angular velocity, raw and applied control, and wheel position. The MAT file contains plant parameters, PID parameters, scenario trajectories, diagnostics, metrics, and MATLAB versus Simulink comparison data.

## 13. Documentation and Git delivery

`README.md` is updated with the PID equations, sign convention, execution command, generated artifacts, acceptance scenarios, and the explicit limitation that position is not controlled.

After implementation, the complete MATLAB and Simulink test suite is run from a fresh MATLAB batch session. When all checks pass, changes are committed locally and pushed to the configured GitHub `origin` remote on the current branch, following the user's default delivery preference.
