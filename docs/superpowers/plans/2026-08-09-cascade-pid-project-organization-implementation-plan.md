# Cascade PID and Project Organization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reorganize the existing two-wheel self-balancing robot MATLAB project into purpose-based folders without breaking entry points, then add and verify a position-attitude cascade PID controller that follows the supplied sample implementation.

**Architecture:** Keep five stable files at the repository root. setup_project.m explicitly adds only executable source folders, while root run files delegate to workflow functions. The cascade controller uses a position PID outer loop to produce a limited attitude reference and an attitude PD inner loop to produce one motor command, followed by the shared plant actuator limit. MATLAB and generated Simulink models use the same plant parameters, controller timing, scenarios, limits, and acceptance criteria.

**Tech Stack:** MATLAB scripts and functions, Simulink programmatic model construction, matlab.unittest, Git, and the existing nonlinear two-wheel self-balancing robot model.

## Global Constraints

- [ ] Use English file, folder, function, block, port, and signal names.
- [ ] Follow MATLAB function-file naming rules and keep the primary function name equal to its file name.
- [ ] Preserve the public root entry points: setup_project.m, run_project.m, run_attitude_pid.m, and run_cascade_pid.m.
- [ ] Do not use genpath. Add only models, controllers, simulation, scenarios, builders, visualization, and workflows.
- [ ] Do not add tests, results, docs, simulink_models, private folders, or generated cache folders to the MATLAB search path.
- [ ] Keep all generated Simulink models in simulink_models and all generated reports in results.
- [ ] Tune only kp_x, ki_x, kd_x, kp_theta, and kd_theta if frozen acceptance tests fail.
- [ ] Finish every task with a green focused test run before committing it.

---

## Task 1: Add explicit project path setup

**Files:**

- Create: setup_project.m
- Create: tests/test_project_paths.m
- Test: tests/test_project_paths.m

- [ ] Write a failing path test that temporarily creates every planned source folder when necessary, calls setup_project, and verifies that the seven source folders are present on the MATLAB path.
- [ ] In the same test, verify that tests, results, docs, simulink_models, builders/private, slprj, and files ending in .slxc are not added by setup_project.
- [ ] Verify that setup_project returns absolute paths for project_root, result_directory, model_directory, test_directory, and missing_code_directories.
- [ ] Run the focused test and confirm it fails because setup_project does not yet exist:

~~~matlab
cd('D:/Research/srtp');
results = runtests('tests/test_project_paths.m');
assertSuccess(results);
~~~

- [ ] Implement setup_project.m with an explicit folder allowlist:

~~~matlab
function paths = setup_project()
project_root = string(fileparts(mfilename("fullpath")));
directory_names = ["models"; "controllers"; "simulation"; "scenarios"; ...
    "builders"; "visualization"; "workflows"];
code_directories = fullfile(project_root, directory_names);
existing = isfolder(code_directories);
for index = 1:numel(code_directories)
    if existing(index)
        addpath(code_directories(index), "-end");
    end
end
paths.project_root = project_root;
paths.code_directories = code_directories(existing);
paths.missing_code_directories = code_directories(~existing);
paths.result_directory = fullfile(project_root, "results");
paths.model_directory = fullfile(project_root, "simulink_models");
paths.test_directory = fullfile(project_root, "tests");
end
~~~

- [ ] Ensure repeated calls are deterministic and do not change the current working directory.
- [ ] Run the focused path tests and the current non-Simulink regression suite. Expect all tests to pass.
- [ ] Commit:

~~~powershell
git add setup_project.m tests/test_project_paths.m
git commit -m "build: add explicit MATLAB project paths"
~~~

---

## Task 2: Move plant, controller, scenario, simulation, and plot sources

**Files:**

- Create: tests/test_project_organization.m
- Move to models/: twsbr_params.m, twsbr_dynamics.m, twsbr_linear_model.m, twsbr_numerical_linearize.m, rk4_step.m
- Move to controllers/: attitude_pid_params.m, attitude_pid_step.m
- Move to scenarios/: attitude_pid_scenarios.m
- Move to simulation/: simulate_open_loop.m, simulate_attitude_pid.m, run_simulink_open_loop.m, run_attitude_pid_simulink.m
- Move to visualization/: plot_attitude_pid_results.m
- Modify: tests that assume root-relative source locations

- [ ] Add organization tests that call setup_project and verify the moved public functions resolve from their purpose folders.
- [ ] Verify each source file is absent from the project root and that only one MATLAB function with each public name is visible.
- [ ] Run the organization test before moving files and confirm it fails on the expected locations.
- [ ] Create the destination folders and move the tracked files with Git-aware moves.
- [ ] Update tests to derive the repository root from test file paths or setup_project output instead of assuming the current directory.
- [ ] Do not add the final missing_code_directories-empty assertion yet; builders and workflows are completed in later tasks.
- [ ] Run test_project_paths.m, test_project_organization.m, and all existing non-Simulink tests. Expect all tests to pass.
- [ ] Commit:

~~~powershell
git add models controllers scenarios simulation visualization tests
git commit -m "refactor: organize MATLAB source by purpose"
~~~

---

## Task 3: Move builders, private creators, and Simulink models

**Files:**

- Move to builders/: build_twsbr_simulink.m, build_attitude_pid_simulink.m
- Move to builders/private/: create_twsbr_simulink_model.m, create_attitude_pid_simulink_model.m
- Move to simulink_models/: twsbr_plant.slx, twsbr_attitude_pid.slx
- Modify: builders and private creators
- Modify: simulation/run_simulink_open_loop.m
- Modify: simulation/run_attitude_pid_simulink.m
- Modify: affected tests

- [ ] Extend organization tests to require the builder functions under builders and private creator files under builders/private.
- [ ] Add tests that assert generated model paths are exactly under setup_project().model_directory.
- [ ] Run the focused tests before the move and confirm the expected location failures.
- [ ] Move the files with Git-aware moves.
- [ ] Update builders to derive project_root one directory above their own folder, create simulink_models when missing, and save models using absolute paths.
- [ ] Keep private creator functions reachable only through builder functions; do not add builders/private to the path.
- [ ] Update Simulink runners to load and simulate models from absolute model paths returned by setup_project.
- [ ] Rebuild both existing Simulink models and verify that the tracked models remain in simulink_models.
- [ ] Remove only generated slprj and .slxc cache artifacts created by this task, then verify Git status contains no cache files.
- [ ] Run focused builder, artifact, and Simulink tests. Expect all tests to pass.
- [ ] Commit:

~~~powershell
git add builders simulink_models simulation tests
git commit -m "refactor: isolate Simulink builders and models"
~~~

---

## Task 4: Preserve stable root workflows

**Files:**

- Create: workflows/run_project_workflow.m
- Create: workflows/run_attitude_pid_workflow.m
- Modify: run_project.m
- Modify: run_attitude_pid.m
- Modify: tests/test_project_paths.m
- Modify: tests/test_project_organization.m
- Create or modify: tests/test_root_entry_points.m

- [ ] Move the current bodies of run_project and run_attitude_pid into workflow functions without changing inputs, outputs, result files, or console summaries.
- [ ] Replace run_project.m with a thin wrapper:

~~~matlab
function summary = run_project(run_tests_flag)
setup_project();
if nargin < 1
    summary = run_project_workflow();
else
    summary = run_project_workflow(run_tests_flag);
end
end
~~~

- [ ] Apply the same wrapper pattern to run_attitude_pid.m and run_attitude_pid_workflow.m.
- [ ] Add a compatibility test that removes project subfolders from the path, adds only the project root, changes to a temporary external directory, and calls each stable root wrapper.
- [ ] Use onCleanup so the test always restores the original path and working directory.
- [ ] Extend path tests to require missing_code_directories to be empty after all seven planned source folders exist.
- [ ] Enforce the root MATLAB allowlist: setup_project.m, run_project.m, run_attitude_pid.m, and, after Task 9, run_cascade_pid.m. Permit no implementation functions at root.
- [ ] Run root-entry, path, organization, existing MATLAB, and existing Simulink tests. Expect all tests to pass.
- [ ] Commit:

~~~powershell
git add workflows run_project.m run_attitude_pid.m tests
git commit -m "refactor: preserve stable project entry points"
~~~

---

## Task 5: Implement the cascade PID parameter and step functions

**Files:**

- Create: controllers/cascade_pid_params.m
- Create: controllers/cascade_pid_step.m
- Create: tests/test_cascade_pid_step.m

- [ ] Write tests that freeze these sample-derived defaults:

~~~matlab
kp_x = 0.24100028146267993;
ki_x = 0.0003962067755988572;
kd_x = 0.1930824033173246;
kp_theta = 9.254929149177556;
kd_theta = 1.0113335430173094;
sample_time = 0.01;
theta_reference_limit = deg2rad(12);
position_integral_limit = 1e6;
~~~

- [ ] Verify cascade_pid_params receives plant parameters or u_max and returns a validated scalar parameter structure.
- [ ] Write a literal sign-convention test using kp_x=0.1, ki_x=0, kd_x=0, kp_theta=2, kd_theta=0, theta_reference_limit=1, u_max=10, x_reference=0.5, and a zero state. Require theta_reference=0.05, theta_error=-0.05, u_raw=-0.1, and next position integral=0.005.
- [ ] Add tests for derivative terms, attitude-rate feedback, positive and negative reference clipping, actuator saturation, integral clamping, reset state, finite input validation, and deterministic repeated calls.
- [ ] Run the focused test and confirm it fails because the functions do not exist.
- [ ] Implement the exact sample structure:

~~~matlab
position_error = x_reference - x;
theta_reference_raw = kp_x * position_error ...
    + ki_x * position_integral - kd_x * x_dot;
theta_reference = min(max(theta_reference_raw, -theta_reference_limit), ...
    theta_reference_limit);
theta_error = theta - theta_reference;
u_raw = kp_theta * theta_error + kd_theta * theta_dot;
u = min(max(u_raw, -u_max), u_max);
next_position_integral = position_integral ...
    + sample_time * position_error;
next_position_integral = min(max(next_position_integral, ...
    -position_integral_limit), position_integral_limit);
~~~

- [ ] Return diagnostics containing position_error, theta_reference_raw, theta_reference, theta_error, u_raw, u, and saturated.
- [ ] Run focused tests plus all current non-Simulink tests. Expect all tests to pass.
- [ ] Commit:

~~~powershell
git add controllers/cascade_pid_params.m controllers/cascade_pid_step.m tests/test_cascade_pid_step.m
git commit -m "feat: add cascade PID controller core"
~~~

---

## Task 6: Add cascade scenarios and MATLAB simulation

**Files:**

- Create: scenarios/cascade_pid_scenarios.m
- Create: simulation/simulate_cascade_pid.m
- Create: tests/test_cascade_pid_scenarios.m
- Create: tests/test_simulate_cascade_pid.m

- [ ] Define exactly five scenarios: zero_state for 2 s; positive_position_step to +0.5 m at 1 s for 8 s; negative_position_step to -0.5 m at 1 s for 8 s; initial_tilt at +5 degrees for 5 s; and force_impulse with a +0.5 m reference at 1 s plus 5 N for 4.0 <= t < 4.2 s for 8 s.
- [ ] Use vector-safe command handles:

~~~matlab
zero_signal = @(t) zeros(size(t));
positive_step = @(t) 0.5 .* double(t >= 1);
negative_step = @(t) -0.5 .* double(t >= 1);
impulse = @(t) 5 .* double(t >= 4 & t < 4.2);
~~~

- [ ] Store finite scenario metadata including reference_start and disturbance_end, using zero where an event is not applicable.
- [ ] Test exact names, durations, initial conditions, references, and disturbance boundary behavior.
- [ ] Implement fixed-step nonlinear simulation with plant_step=0.001 s and controller sample_time=0.01 s. Hold the controller command between controller updates.
- [ ] Log time, state, position_reference, position_error, theta_reference, theta_reference_raw, u_raw, u, disturbance_force, position_integral, and saturated.
- [ ] Compute maximum absolute tilt, final tilt, maximum absolute position, final position error, position settling/recovery time, position IAE, tilt IAE, maximum absolute command, maximum absolute attitude reference, and saturation duration.
- [ ] Integrate saturation over intervals with sum(diff(time).*double(saturated(1:end-1))) and use trapz for IAE metrics.
- [ ] Mark a run failed for nonfinite values, abs(theta)>30 degrees, abs(x)>x_limit, abs(theta_reference)>12 degrees, or abs(u)>u_max.
- [ ] Add focused zero-state, deterministic, dimensional, limit, metric, and failure-detection tests.
- [ ] Run the two new test files and all existing non-Simulink tests. Expect all tests to pass.
- [ ] Commit:

~~~powershell
git add scenarios/cascade_pid_scenarios.m simulation/simulate_cascade_pid.m tests/test_cascade_pid_scenarios.m tests/test_simulate_cascade_pid.m
git commit -m "feat: add cascade PID MATLAB simulation"
~~~

---

## Task 7: Freeze acceptance tests and add cascade plotting

**Files:**

- Create: tests/test_cascade_pid_acceptance.m
- Create: visualization/plot_cascade_pid_results.m
- Create: tests/test_plot_cascade_pid_results.m
- Modify only if required by failing acceptance: controllers/cascade_pid_params.m

- [ ] Encode the approved all-scenario conditions: finite signals, tilt no greater than 30 degrees, attitude reference no greater than 12 degrees, command no greater than u_max, and position no greater than x_limit.
- [ ] For both position steps require final absolute position error below 0.05 m, entry into and continued residence within +/-0.05 m by absolute time 6 s, and final absolute tilt below 0.5 degrees.
- [ ] For initial tilt require final absolute position error below 0.05 m and final absolute tilt below 0.5 degrees.
- [ ] For force impulse require recovery into and continued residence within +/-0.10 m position error no later than 3 s after the impulse ends.
- [ ] Run the frozen acceptance tests with the sample-derived gains.
- [ ] If they fail, diagnose the failed scenario and tune only kp_x, ki_x, kd_x, kp_theta, and kd_theta. Record the accepted numeric values directly in cascade_pid_params.m. Do not add an optimizer or weaken thresholds.
- [ ] Implement a five-panel English plot: position/reference; tilt/attitude reference; angular rate; raw/applied command; position integral.
- [ ] Verify the plotting function accepts a simulation result and an output path, creates the parent results folder, writes a PNG, and closes only figures it owns.
- [ ] Run acceptance and plot tests plus all non-Simulink tests. Expect all tests to pass.
- [ ] Commit:

~~~powershell
git add controllers/cascade_pid_params.m tests/test_cascade_pid_acceptance.m visualization/plot_cascade_pid_results.m tests/test_plot_cascade_pid_results.m
git commit -m "test: freeze cascade PID acceptance criteria"
~~~

---

## Task 8: Generate and compare the cascade Simulink model

**Files:**

- Create: builders/build_cascade_pid_simulink.m
- Create: builders/private/create_cascade_pid_simulink_model.m
- Create: simulation/run_cascade_pid_simulink.m
- Generate: simulink_models/twsbr_cascade_pid.slx
- Create: tests/test_cascade_pid_simulink_model.m
- Create: tests/test_cascade_pid_simulink_equivalence.m

- [ ] Write a builder test that requires English block, port, and signal names; the nonlinear plant; a discrete cascade controller; shared actuator saturation; external force input; and logged comparison signals.
- [ ] Write an equivalence test for positive_position_step requiring maximum absolute tilt difference below 0.2 degrees and maximum absolute position difference below 0.01 m.
- [ ] Run these tests before implementation and confirm they fail because the builder and model do not exist.
- [ ] Build the model programmatically and save it only at setup_project().model_directory/twsbr_cascade_pid.slx.
- [ ] Implement the cascade controller in a MATLAB Function block with persistent position integral, reset initialization, 0.01 s discrete updates, 12 degree attitude-reference clipping, and 1e6 integral clipping.
- [ ] Keep actuator saturation outside the controller calculation so u_raw and applied u are both logged.
- [ ] Name all generated lines and verify names cover time-aligned position reference, position, tilt, attitude reference, angular rate, raw command, applied command, position integral, and disturbance force.
- [ ] Implement run_cascade_pid_simulink to accept parameters and one scenario, return the same public log fields and metrics as simulate_cascade_pid, and load models by absolute path.
- [ ] Run model-structure and equivalence tests, then all Simulink tests. Expect all tests to pass.
- [ ] Remove only generated cache artifacts, verify none are tracked or untracked, and commit:

~~~powershell
git add builders simulation simulink_models/twsbr_cascade_pid.slx tests
git commit -m "feat: add cascade PID Simulink model"
~~~

---

## Task 9: Add the cascade workflow, root entry point, docs, and artifacts

**Files:**

- Create: workflows/run_cascade_pid_workflow.m
- Create: run_cascade_pid.m
- Modify: tests/test_root_entry_points.m
- Modify: tests/test_project_organization.m
- Modify: README.md
- Generate: results/cascade_pid_results.mat
- Generate: results/cascade_pid_response.png

- [ ] Write a root-entry test requiring run_cascade_pid to work from an external directory when only the project root is initially on the MATLAB path.
- [ ] Implement run_cascade_pid.m as a thin setup-and-delegate wrapper matching the existing public wrapper style.
- [ ] Implement run_cascade_pid_workflow to build the Simulink model, run all five MATLAB scenarios, run the positive-step Simulink scenario, calculate acceptance and equivalence, optionally run tests, save artifacts, and return a summary structure.
- [ ] Save these top-level MAT variables: plant_params, cascade_params, scenarios, matlab_simulations, simulink_simulation, acceptance, and comparison.
- [ ] Make any acceptance or equivalence failure raise a clear error after artifacts are saved for diagnosis.
- [ ] Generate results/cascade_pid_response.png using the positive-position-step MATLAB result and the five-panel plotting function.
- [ ] Update README with the final folder tree, stable entry points, cascade equations and sign convention, default scenarios, acceptance thresholds, artifact locations, and the statement that setup_project uses an explicit allowlist.
- [ ] Finalize the organization test root allowlist to exactly setup_project.m, run_project.m, run_attitude_pid.m, and run_cascade_pid.m.
- [ ] Run run_cascade_pid(false) from the project root and an external temporary directory. Expect accepted MATLAB scenarios, accepted MATLAB/Simulink comparison, and both result artifacts.
- [ ] Run affected root-entry, organization, workflow, acceptance, artifact, and README tests. Expect all tests to pass.
- [ ] Commit:

~~~powershell
git add run_cascade_pid.m workflows/run_cascade_pid_workflow.m README.md tests results/cascade_pid_results.mat results/cascade_pid_response.png
git commit -m "feat: add cascade PID project workflow"
~~~

---

## Task 10: Complete full verification, review, and Git publication

**Files:**

- Verify: all project source, tests, generated models, results, and documentation
- Modify: only files required by verified failures or review findings

- [ ] Run the full public workflow with tests enabled:

~~~matlab
cd('D:/Research/srtp');
summary = run_cascade_pid(true);
assert(summary.accepted);
~~~

- [ ] Record the exact matlab.unittest pass count and confirm zero failures, incompletes, and filtered tests.
- [ ] Run checkcode on every tracked .m file and require zero actionable messages.
- [ ] Verify every tracked file and folder name is English and every MATLAB filename is a valid MATLAB identifier.
- [ ] Verify all generated Simulink blocks, ports, and lines have English names.
- [ ] Verify an external-directory call works with only D:/Research/srtp initially added to the MATLAB path.
- [ ] Verify the root contains no implementation .m files beyond the four stable entry points.
- [ ] Verify Git status contains no slprj directories, .slxc files, temporary files, editor files, or unrelated regenerated model/result binaries.
- [ ] Request a read-only code review. Apply only verified findings and rerun the smallest relevant regression followed by the full suite.
- [ ] Run Git whitespace validation and inspect the final diff and status:

~~~powershell
git diff --check
git status --short
git diff --stat origin/main...HEAD
~~~

- [ ] If final review fixes are needed, create one final focused commit. Otherwise keep the task commits unchanged.
- [ ] Push main and verify the local and remote commit IDs match:

~~~powershell
git push origin main
git rev-parse HEAD
git rev-parse origin/main
~~~

- [ ] Report the final commit ID, push status, exact test count, MATLAB/Simulink maximum tilt and position differences, acceptance result, and absolute paths to the MAT, PNG, SLX, README, design, and implementation-plan files.
