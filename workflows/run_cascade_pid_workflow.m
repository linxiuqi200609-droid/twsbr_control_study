function summary = run_cascade_pid_workflow(run_tests_flag)
%RUN_CASCADE_PID_WORKFLOW Build, verify, and export cascade PID results.

if nargin < 1
    run_tests_flag = true;
end
if ~islogical(run_tests_flag) || ~isscalar(run_tests_flag)
    error("twsbr:cascade_pid:invalid_test_flag", ...
        "run_tests_flag must be a logical scalar.");
end

project_paths = setup_project();
project_root = project_paths.project_root;
if ~isfolder(project_paths.result_directory)
    mkdir(project_paths.result_directory);
end

plant_params = twsbr_params();
cascade_params = cascade_pid_params(struct(), plant_params);
scenarios = cascade_pid_scenarios();
scenario_names = fieldnames(scenarios);
matlab_simulations = struct();
acceptance = struct();

for index = 1:numel(scenario_names)
    scenario_name = scenario_names{index};
    matlab_simulations.(scenario_name) = simulate_cascade_pid( ...
        plant_params, cascade_params, scenarios.(scenario_name));
    acceptance.(scenario_name) = evaluate_acceptance( ...
        scenario_name, matlab_simulations.(scenario_name), plant_params);
end

model_path = build_cascade_pid_simulink(plant_params, cascade_params);
simulink_simulation = run_cascade_pid_simulink( ...
    plant_params, cascade_params, scenarios.positive_position_step);
comparison = compare_positive_step( ...
    matlab_simulations.positive_position_step, ...
    simulink_simulation, scenarios.positive_position_step, cascade_params);

results_path = fullfile( ...
    project_paths.result_directory, "cascade_pid_results.mat");
figure_path = fullfile( ...
    project_paths.result_directory, "cascade_pid_response.png");
figure_path = plot_cascade_pid_results( ...
    matlab_simulations.positive_position_step, figure_path);
save(results_path, "plant_params", "cascade_params", "scenarios", ...
    "matlab_simulations", "simulink_simulation", "acceptance", ...
    "comparison");

rejected_scenarios = rejected_scenario_names(acceptance, scenario_names);
if ~isempty(rejected_scenarios)
    error("twsbr:cascade_pid:acceptance_failure", ...
        "Cascade PID acceptance failed for %s. Diagnostics saved to %s.", ...
        strjoin(rejected_scenarios, ", "), results_path);
end
if ~comparison.accepted
    error("twsbr:cascade_pid:equivalence_failure", ...
        "MATLAB/Simulink agreement failed: %s. Diagnostics saved to %s.", ...
        comparison.failure_reason, results_path);
end

test_results = matlab.unittest.TestResult.empty;
if run_tests_flag
    test_results = runtests(project_paths.test_directory);
    assert_cascade_pid_test_success(test_results);
end

summary = struct();
summary.project_root = project_root;
summary.model_path = model_path;
summary.results_path = results_path;
summary.figure_path = figure_path;
summary.plant_params = plant_params;
summary.cascade_params = cascade_params;
summary.scenarios = scenarios;
summary.matlab_simulations = matlab_simulations;
summary.simulink_simulation = simulink_simulation;
summary.acceptance = acceptance;
summary.comparison = comparison;
summary.accepted = logical(all(structfun( ...
    @(result) result.accepted, acceptance)) && comparison.accepted);
summary.test_results = test_results;

print_summary(summary, scenario_names);
end

function result = evaluate_acceptance( ...
    scenario_name, simulation, plant_params)
finite_logs = simulation_logs_are_finite(simulation);
tilt_within_limit = max(abs(rad2deg(simulation.state(:, 3)))) <= 30.0;
theta_reference_within_limit = ...
    max(abs(rad2deg(simulation.theta_reference))) <= 12.0 + 1e-12;
actuator_within_limit = ...
    max(abs(simulation.u)) <= plant_params.u_max + 1e-12;
position_within_limit = ...
    max(abs(simulation.state(:, 1))) <= plant_params.x_limit;
common_accepted = simulation.success && finite_logs && ...
    tilt_within_limit && theta_reference_within_limit && ...
    actuator_within_limit && position_within_limit;

behavior_accepted = true;
switch scenario_name
    case {"positive_position_step", "negative_position_step"}
        behavior_accepted = ...
            simulation.metrics.final_abs_position_error < 0.05 && ...
            simulation.metrics.position_settling_time <= 5.0 && ...
            simulation.metrics.final_abs_tilt_deg < 0.5;
    case "initial_tilt"
        behavior_accepted = ...
            simulation.metrics.final_abs_position_error < 0.05 && ...
            simulation.metrics.final_abs_tilt_deg < 0.5;
    case "force_impulse"
        behavior_accepted = ...
            simulation.metrics.disturbance_recovery_time <= 3.0;
end

result = struct();
result.accepted = common_accepted && behavior_accepted;
result.common_accepted = common_accepted;
result.behavior_accepted = behavior_accepted;
result.success = simulation.success;
result.failure_reason = simulation.failure_reason;
result.finite_logs = finite_logs;
result.tilt_within_limit = tilt_within_limit;
result.theta_reference_within_limit = theta_reference_within_limit;
result.actuator_within_limit = actuator_within_limit;
result.position_within_limit = position_within_limit;
result.metrics = simulation.metrics;
end

function finite = simulation_logs_are_finite(simulation)
numeric_fields = {"time", "state", "position_reference", ...
    "position_error", "theta_reference", "theta_reference_raw", ...
    "theta_error", "u_raw", "u", "disturbance_force", ...
    "position_integral", "saturated"};
finite = true;
for index = 1:numel(numeric_fields)
    values = simulation.(numeric_fields{index});
    finite = finite && all(isfinite(values), "all");
end

metric_names = fieldnames(simulation.metrics);
for index = 1:numel(metric_names)
    value = simulation.metrics.(metric_names{index});
    finite = finite && isnumeric(value) && isscalar(value) && ...
        isfinite(value);
end
end

function comparison = compare_positive_step( ...
    matlab_simulation, simulink_simulation, scenario, cascade_params)
comparison = struct();
comparison.maximum_tilt_difference_deg = realmax;
comparison.maximum_position_difference_m = realmax;
comparison.tilt_tolerance_deg = 0.2;
comparison.position_tolerance_m = 0.01;
comparison.common_sample_count = 0;
comparison.failure_reason = "";
comparison.accepted = false;

if ~matlab_simulation.success || ~simulink_simulation.success
    comparison.failure_reason = "MATLAB or Simulink simulation failed";
    return
end
if numel(matlab_simulation.time) < 2 || ...
        numel(simulink_simulation.time) < 2
    comparison.failure_reason = "Insufficient trajectory samples";
    return
end

common_time = (0:cascade_params.plant_step:scenario.duration).';
matlab_state = interp1(matlab_simulation.time, ...
    matlab_simulation.state, common_time, "pchip");
simulink_state = interp1(simulink_simulation.time, ...
    simulink_simulation.state, common_time, "pchip");
comparison.common_sample_count = numel(common_time);
if any(~isfinite(matlab_state), "all") || ...
        any(~isfinite(simulink_state), "all")
    comparison.failure_reason = "Interpolation produced nonfinite values";
    return
end

comparison.maximum_tilt_difference_deg = rad2deg(max(abs( ...
    matlab_state(:, 3) - simulink_state(:, 3))));
comparison.maximum_position_difference_m = max(abs( ...
    matlab_state(:, 1) - simulink_state(:, 1)));
comparison.accepted = ...
    comparison.maximum_tilt_difference_deg < ...
    comparison.tilt_tolerance_deg && ...
    comparison.maximum_position_difference_m < ...
    comparison.position_tolerance_m;
if ~comparison.accepted
    comparison.failure_reason = ...
        "Trajectory differences exceed frozen tolerances";
end
end

function names = rejected_scenario_names(acceptance, scenario_names)
rejected = false(size(scenario_names));
for index = 1:numel(scenario_names)
    rejected(index) = ~acceptance.(scenario_names{index}).accepted;
end
names = string(scenario_names(rejected));
end

function print_summary(summary, scenario_names)
fprintf("\nCascade PID results\n");
fprintf("  Outer gains: Kp=%.12g, Ki=%.12g, Kd=%.12g\n", ...
    summary.cascade_params.kp_x, summary.cascade_params.ki_x, ...
    summary.cascade_params.kd_x);
fprintf("  Inner gains: Kp=%.12g, Kd=%.12g\n", ...
    summary.cascade_params.kp_theta, summary.cascade_params.kd_theta);
for index = 1:numel(scenario_names)
    scenario_name = scenario_names{index};
    result = summary.acceptance.(scenario_name);
    fprintf("  %-24s accepted=%d, final position error=%.6f m, ", ...
        scenario_name, result.accepted, ...
        result.metrics.final_abs_position_error);
    fprintf("final tilt=%.6f deg\n", ...
        result.metrics.final_abs_tilt_deg);
end
fprintf("  MATLAB/Simulink maximum tilt difference: %.9f deg\n", ...
    summary.comparison.maximum_tilt_difference_deg);
fprintf("  MATLAB/Simulink maximum position difference: %.9f m\n", ...
    summary.comparison.maximum_position_difference_m);
fprintf("  Model:  %s\n", summary.model_path);
fprintf("  Data:   %s\n", summary.results_path);
fprintf("  Figure: %s\n\n", summary.figure_path);
end
