# Two Wheeled Self Balancing Robot Control Project

This project provides matching MATLAB and Simulink models of the longitudinal dynamics of a two wheeled self balancing robot. It preserves the open-loop plant, basic attitude PID and cascade PID workflows and adds a unified five-controller study with fuzzy cascade PID, LQR and LQI. The plant is an equivalent nonlinear cart pole model for software simulation and control verification.

## Project structure

```text
srtp/
|-- .gitignore
|-- README.md
|-- setup_project.m
|-- run_project.m
|-- run_attitude_pid.m
|-- run_cascade_pid.m
|-- run_control_study.m
|-- config/
|-- optimization/
|-- experiments/
|-- evaluation/
|-- reporting/
|-- models/
|   |-- twsbr_params.m
|   |-- twsbr_dynamics.m
|   |-- twsbr_linear_model.m
|   |-- twsbr_numerical_linearize.m
|   `-- twsbr_rk4_step.m
|-- controllers/
|   |-- attitude_pid_params.m
|   |-- attitude_pid_step.m
|   |-- cascade_pid_params.m
|   |-- cascade_pid_step.m
|   `-- validate_cascade_pid_timing.m
|-- simulation/
|   |-- simulate_open_loop.m
|   |-- simulate_attitude_pid.m
|   |-- simulate_cascade_pid.m
|   |-- run_simulink_open_loop.m
|   |-- run_attitude_pid_simulink.m
|   `-- run_cascade_pid_simulink.m
|-- scenarios/
|   |-- attitude_pid_scenarios.m
|   `-- cascade_pid_scenarios.m
|-- builders/
|   |-- build_twsbr_simulink.m
|   |-- build_attitude_pid_simulink.m
|   |-- build_cascade_pid_simulink.m
|   `-- private/
|-- simulink_models/
|-- visualization/
|   |-- plot_attitude_pid_results.m
|   `-- plot_cascade_pid_results.m
|-- workflows/
|   |-- run_project_workflow.m
|   |-- run_attitude_pid_workflow.m
|   `-- run_cascade_pid_workflow.m
|-- tests/
|-- results/
`-- docs/
```

The five stable public root entry points are `setup_project.m`, `run_project.m`, `run_attitude_pid.m`, `run_cascade_pid.m`, and `run_control_study.m`. The tree above shows selected files. setup_project.m uses an explicit allowlist: `models`, `controllers`, `simulation`, `scenarios`, `builders`, `visualization`, `workflows`, `config`, `optimization`, `experiments`, `evaluation`, and `reporting`. It never uses unrestricted `genpath`, and excludes private helpers, tests, results, generated models, documentation, and caches.

## State and inputs

The state order is:

```text
[x; x_dot; theta; theta_dot]
```

- `x`: wheel axle position in meters.
- `x_dot`: wheel axle velocity in meters per second.
- `theta`: body tilt from the upright equilibrium in radians.
- `theta_dot`: body angular velocity in radians per second.
- `u`: normalized motor command converted to horizontal force by `motor_force_gain`.
- `force_disturbance`: external horizontal force in newtons.
- `torque_disturbance`: external body torque in newton meters.

The upright equilibrium is `theta = 0`. The plant does not clip `u`; actuator saturation belongs outside the plant so all controllers share the same limit.

## Main files and directories

- `models/`: validated plant parameters, nonlinear dynamics, linearization, and RK4 integration.
- `controllers/`: deterministic attitude and cascade controller parameter/update functions.
- `scenarios/`: immutable attitude and cascade acceptance scenarios.
- `simulation/`: MATLAB simulations and Simulink execution adapters.
- `builders/`: reproducible public model builders; private creation functions remain isolated in `builders/private/`.
- `visualization/`: response plotting and PNG export.
- `workflows/`: implementations behind the stable root wrappers.
- `simulink_models/`: generated SLX artifacts, never added to the MATLAB path.
- `results/`: generated MAT and PNG artifacts, never added to the MATLAB path.
- `tests/`: MATLAB unit and integration tests.

## Quick start

Open MATLAB and run the plant workflow:

```matlab
cd("D:/Research/srtp")
run_project
```

To generate plant results without running the test suite first:

```matlab
summary = run_project(false);
```

To build and verify the basic attitude PID, run:

```matlab
cd("D:/Research/srtp")
summary = run_attitude_pid;
```

For a faster PID result export without rerunning all tests:

```matlab
summary = run_attitude_pid(false);
```

To build, verify, and export the cascade position PID:

```matlab
summary = run_cascade_pid;
```

To export cascade results without rerunning the full test suite:

```matlab
summary = run_cascade_pid(false);
```

## Basic attitude PID

The controller uses only body tilt and angular velocity:

```text
theta_error = theta - theta_reference
u_raw = kp * theta_error + ki * integral_error + kd * theta_dot
u = clip(u_raw, -u_max, u_max)
```

The default upright reference is zero. The discrete controller runs every `0.01 s`; the MATLAB nonlinear plant uses a `0.001 s` RK4 step. The default gains are `kp = 1.90`, `ki = 0.20`, and `kd = 0.18`. Conditional integration prevents the integral state from increasing actuator saturation, and both the actuator and integral state have symmetric limits.

The frozen acceptance scenarios are zero state, positive 5 degree tilt, negative 5 degree tilt, positive 8 degree tilt, and a `0.05 N m` torque impulse. MATLAB and Simulink use the same gains, sign convention, sample time, actuator limit, plant parameters, and scenarios.

## Cascade position PID

The outer loop converts position error into a bounded body-tilt reference. The derivative term uses measured velocity rather than a numerical derivative of error:

```text
position_error = x_reference - x
theta_reference_raw = kp_x * position_error
                    + ki_x * position_integral
                    - kd_x * x_dot
theta_reference = clip(theta_reference_raw, -12 deg, +12 deg)
```

The inner loop uses the project sign convention that positive body tilt is corrected by a positive raw command:

```text
theta_error = theta - theta_reference
u_raw = kp_theta * theta_error + kd_theta * theta_dot
u = clip(u_raw, -u_max, +u_max)
```

The position integral is updated after actuator saturation without anti-windup feedback. The controller sample time is `0.01 s`; the nonlinear MATLAB plant step and maximum Simulink solver step are `0.001 s`. The final frozen gains are:

```text
kp_x = 0.24100028146267993
ki_x = 0.0003962067755988572
kd_x = 0.1930824033173246
kp_theta = 9.254929149177556
kd_theta = 1.0113335430173094
```

The five frozen scenarios are:

1. `zero_state`: zero state and zero reference for `2 s`.
2. `positive_position_step`: position reference steps to `+0.5 m` at `1 s`; duration `8 s`.
3. `negative_position_step`: position reference steps to `-0.5 m` at `1 s`; duration `8 s`.
4. `initial_tilt`: initial body tilt is `+5 deg` with zero position reference; duration `5 s`.
5. `force_impulse`: the reference steps to `+0.5 m` at `1 s`, and a `5 N` force acts for `4.0 <= t < 4.2 s`; duration `8 s`.

All scenarios must remain finite, keep absolute body tilt at or below `30 deg`, keep the tilt reference at or below `12 deg`, respect the actuator limit, and stay inside the plant position limit. The scenario-specific acceptance thresholds are:

- Positive and negative position steps: final absolute position error below `0.05 m`, position settling within `5 s` of the step, and final absolute tilt below `0.5 deg`.
- Initial tilt: final absolute position error below `0.05 m` and final absolute tilt below `0.5 deg`.
- Force impulse: return to and remain within a `0.10 m` position-error band within `3 s` after the impulse ends.
- MATLAB/Simulink positive-step agreement: maximum tilt difference below `0.2 deg` and maximum position difference below `0.01 m`.

`run_cascade_pid_workflow.m` runs all five MATLAB scenarios, rebuilds the generated Simulink model, runs the Simulink `positive_position_step`, evaluates every threshold, and saves diagnostic artifacts before raising an acceptance or equivalence error. The response PNG always uses the MATLAB `positive_position_step` trajectory and the shared five-panel plot.

## Generated outputs

The workflows create:

```text
results/open_loop_results.mat
results/open_loop_response.png
simulink_models/twsbr_plant.slx
results/attitude_pid_results.mat
results/attitude_pid_response.png
simulink_models/twsbr_attitude_pid.slx
results/cascade_pid_results.mat
results/cascade_pid_response.png
simulink_models/twsbr_cascade_pid.slx
```

The open loop MAT file contains default parameters, analytical state space matrices, the controllability matrix and rank, equilibrium and 3 degree simulations, and the matching Simulink simulation. The attitude PID MAT file contains plant and PID parameters, all MATLAB scenario trajectories and metrics, the positive tilt Simulink trajectory, acceptance decisions, and the MATLAB versus Simulink comparison. `results/cascade_pid_results.mat` contains the top-level variables `plant_params`, `cascade_params`, `scenarios`, `matlab_simulations`, `simulink_simulation`, `acceptance`, and `comparison`. `results/cascade_pid_response.png` is a five-panel positive-position-step response.

## Model verification

The automated tests verify:

- zero state and zero input form an equilibrium;
- positive tilt produces falling angular acceleration;
- positive motor input produces corrective negative angular acceleration;
- analytical and numerical linearization agree;
- the linearized plant has controllability rank four;
- an uncontrolled 3 degree initial tilt diverges from upright;
- MATLAB and Simulink open loop state trajectories agree;
- the plant model can be rebuilt without warnings;
- PID proportional, integral, derivative, saturation, and anti windup behavior;
- all five basic attitude PID acceptance scenarios;
- applied control and integral state limits;
- warning free Simulink attitude PID reconstruction and English block names;
- MATLAB and Simulink closed loop tilt agreement within 0.2 degree;
- exact cascade scenarios, multirate simulation, controller limits, and acceptance criteria;
- warning free cascade model reconstruction with English block and signal names;
- MATLAB/Simulink cascade position and tilt agreement;
- generation of the expected MAT, PNG, and SLX artifacts.

## Scope and deployment limitation

The basic attitude PID deliberately uses only `theta` and `theta_dot`, so position drift remains expected in that workflow. The cascade controller adds software position regulation through `x` and `x_dot`. Both controllers produce an unsaturated command `u_raw`; one shared actuator block applies the limit before the signal enters the plant. These MATLAB and Simulink results are software simulations, not evidence of safe physical deployment. Hardware use still requires sensor processing, actuator calibration, real-time timing validation, safety interlocks, and experimental verification.

## Unified five-controller study

`run_control_study` runs a reproducible comparison of `ATTITUDE_PID`,
`CASCADE_PID`, `FUZZY_PID`, `LQR`, and `LQI`. The basic attitude PID is
deliberately the attitude-only baseline: it cannot observe or track position,
but its position error remains in the common metrics as a limited-capability
control group. Cascade PID uses position PID plus attitude PD; fuzzy PID uses
the same outer loop with fixed-rule online inner PID gain adjustment; LQR is
discrete full-state regulation; and LQI adds position-error integration.

MATLAB with Simulink, Control System Toolbox, and Statistics and Machine
Learning Toolbox is required for the complete study. Run either planned mode
from the project root:

```matlab
summary = run_control_study("quick", true);
summary = run_control_study("full", true);
```

The first argument selects frozen study budgets. Quick uses population 24,
240 DE evaluations per controller, seed 0, and ten paired Monte Carlo cases.
Full uses population 40, 3200 evaluations per controller, seeds 0 through 9,
and 200 paired Monte Carlo cases. The default 10-second training and held-out
durations are intentionally not shortened by Quick mode. All five controllers
receive the same numerical plant, sampling, saturation, scenarios, objective,
failure policy, DE evaluation budget, and seed policy. Frozen parameters are
selected from training data only; held-out deterministic and paired Monte
Carlo data are never used for tuning or replacement vectors. Monte Carlo
perturbs only the physical plant: LQR/LQI feedback gains are decoded from the
same nominal plant for every trial. `simulate_control_system` accepts an
optional seventh nominal-design-plant argument; six-argument calls continue
to use their supplied plant for both design and physics. Raw results include
the decoded `controller_parameters` used for that run.

For test-only integration checks, an optional third structure may supply
explicit `frozen_vectors`, an isolated `output_root`, and explicit stage
switches such as `run_monte_carlo=false` or `run_simulink=false`. This bypass
is not a valid scientific result: the manifest records the skipped stages and
the frozen-vector bypass. Normal use should keep the documented two-argument
calls above.

Each run writes `results/control_study_<mode>/` (or the explicit output root):

```text
tuning/                 DE runs (MAT/CSV) and frozen-vector JSON
deterministic/          held-out metrics, parameters, and complexity CSVs
raw/                    indexed raw deterministic MAT traces
monte_carlo/            paired all-trial metrics CSV
simulink_validation/    five-controller equivalence summary CSV
figures/                F1-F6 PNG and vector-PDF artifacts
statistics.xlsx         flat raw and summary sheets
statistics_*.csv        descriptive, omnibus, and pairwise outputs
training_scenarios.json
heldout_scenarios.json
run_manifest.json
```

The manifest records the effective options, pre-run source-dirty status,
seed/configuration provenance,
frozen vectors, Git commit, platform/toolboxes, stage status, and SHA-256
hashes of `experiment_config.m`, `objective_config.m`, and
`monte_carlo_config.m`. Rerunning with a stage disabled rewrites the managed
table outputs as typed empty products and removes only the twelve managed
figure names, so prior artifacts cannot be misrepresented as current results.

Git is optional provenance, not a simulation prerequisite. The validator
identifies the source root independently of Git. When Git metadata cannot be
read, `git_commit` and/or `context.source_dirty` are `"unavailable"`; dirty
status is a Boolean only when Git status was successfully observed.

Per-trial `simulation_runtime_seconds` preserves total simulator walltime
(`runtime_seconds` in raw MAT results). `controller_runtime_seconds` times
only `controller_step` plus `controller_after_actuation`, excluding controller
construction/reset, actuator clipping, validation outside these calls,
plant integration and logging. `controller_evaluation_count` counts attempted
lifecycles; `controller_completed_count` counts those completing both calls.
A failed attempt contributes its elapsed time and attempted count.
`mean_step_runtime_us` is controller time divided by attempted evaluations,
not by plant samples. It is `NaN` (missing) for zero attempts, with zero
controller time; failed rows remain in MAT/CSV/XLSX output. Runtime variation
is excluded from deterministic training-objective details.

Omnibus statistics report `configured_group_count` (all controller groups
present in the input) and `analyzed_group_count` (groups with successful rows).
The backward-compatible `group_count` remains the configured count;
`successful_n` counts analyzed observations. Kruskal-Wallis uses only successful
groups and returns a missing p-value when fewer than two groups are available.
F4-F6 explicitly name controllers without successful trials and do not invent
scores or replacement successes.

### Fuzzy gain policy: implementation clarification

The historical design spec asks for fixed positive safety ranges frozen before
training. The approved detailed controller plan instead specifies nonnegative
online gains, clipped to `[0, 4 * candidate_base_gain]`; these bounds stay fixed
during each candidate's online execution but vary between trained candidates.
The implementation follows that detailed policy. This is an explicit departure
from candidate-independent, strictly positive caps, not literal compliance with
the stricter high-level wording. The final-review ruling preserves the numeric
and Simulink laws. Adopting global positive caps later requires approved cap
values, matching controller changes, new tests and retuning.

`control_energy` is the integral of squared normalized applied input. It is a
control-cost proxy, not measured battery or electrical energy. Quick's ten
Monte Carlo trials check the pipeline and provide exploratory comparisons;
they are not enough on their own for strong general performance claims. Full
is the larger planned experiment, but it too is software-only and does not
demonstrate hardware safety or deployment readiness.
