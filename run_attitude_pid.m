function summary = run_attitude_pid(run_tests_flag)
%RUN_ATTITUDE_PID Build, simulate, verify, and export attitude PID results.

if nargin < 1
    run_tests_flag = true;
end
if ~islogical(run_tests_flag) || ~isscalar(run_tests_flag)
    error("twsbr:attitude_pid:invalid_test_flag", ...
        "run_tests_flag must be a logical scalar.");
end

project_root = fileparts(mfilename("fullpath"));
project_paths = setup_project();
results_directory = fullfile(project_root, "results");
if ~isfolder(results_directory)
    mkdir(results_directory);
end

plant_params = twsbr_params();
pid_params = attitude_pid_params(struct(), plant_params);
scenarios = attitude_pid_scenarios();
scenario_names = fieldnames(scenarios);
matlab_simulations = struct();
acceptance = struct();

for index = 1:numel(scenario_names)
    scenario_name = scenario_names{index};
    simulation = simulate_attitude_pid( ...
        plant_params, pid_params, scenarios.(scenario_name));
    matlab_simulations.(scenario_name) = simulation;
    acceptance.(scenario_name) = evaluate_acceptance( ...
        scenario_name, simulation, pid_params);
end

simulink_simulation = run_attitude_pid_simulink( ...
    plant_params, pid_params, scenarios.positive_tilt);
matlab_theta = interp1(matlab_simulations.positive_tilt.time, ...
    matlab_simulations.positive_tilt.state(:, 3), ...
    simulink_simulation.time, "pchip");
comparison = struct();
comparison.maximum_tilt_error_deg = rad2deg(max(abs( ...
    matlab_theta - simulink_simulation.state(:, 3))));
comparison.accepted = comparison.maximum_tilt_error_deg < 0.2;

model_path = fullfile(project_paths.model_directory, "twsbr_attitude_pid.slx");
results_path = fullfile(results_directory, "attitude_pid_results.mat");
figure_path = fullfile(results_directory, "attitude_pid_response.png");
figure_handle = plot_attitude_pid_results( ...
    matlab_simulations.positive_tilt);
figure_cleanup = onCleanup(@() close_if_valid(figure_handle));
exportgraphics(figure_handle, figure_path, "Resolution", 160);
save(results_path, "plant_params", "pid_params", ...
    "matlab_simulations", "simulink_simulation", ...
    "comparison", "acceptance");
close(figure_handle);
clear figure_cleanup;

test_results = matlab.unittest.TestResult.empty;
if run_tests_flag
    test_results = runtests(fullfile(project_root, "tests"));
    assertSuccess(test_results);
end

summary = struct();
summary.project_root = project_root;
summary.model_path = model_path;
summary.results_path = results_path;
summary.figure_path = figure_path;
summary.plant_params = plant_params;
summary.pid_params = pid_params;
summary.matlab_simulations = matlab_simulations;
summary.simulink_simulation = simulink_simulation;
summary.comparison = comparison;
summary.acceptance = acceptance;
summary.test_results = test_results;

print_summary(summary, scenario_names);
end

function result = evaluate_acceptance(scenario_name, simulation, pid_params)
within_limits = max(abs(simulation.u)) <= pid_params.u_max + 1e-12 && ...
    max(abs(simulation.integral_error)) <= ...
    pid_params.integral_limit + 1e-12;
finite_logs = all(isfinite(simulation.state), "all") && ...
    all(isfinite(simulation.u)) && all(isfinite(simulation.integral_error));
behavior_accepted = false;

switch scenario_name
    case "zero_state"
        behavior_accepted = max(abs(simulation.state), [], "all") < 1e-10;
    case {"positive_tilt", "negative_tilt"}
        behavior_accepted = simulation.metrics.settling_time <= 2.0 && ...
            simulation.metrics.final_abs_tilt_deg < 0.5;
    case "large_tilt"
        behavior_accepted = simulation.metrics.final_abs_tilt_deg < 0.5;
    case "torque_impulse"
        behavior_accepted = simulation.metrics.settling_time - 1.05 <= 2.0;
end

result = struct();
result.accepted = simulation.success && within_limits && ...
    finite_logs && behavior_accepted;
result.success = simulation.success;
result.failure_reason = simulation.failure_reason;
result.metrics = simulation.metrics;
end

function print_summary(summary, scenario_names)
fprintf("\nBasic attitude PID results\n");
fprintf("  Gains: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", ...
    summary.pid_params.kp, summary.pid_params.ki, summary.pid_params.kd);
for index = 1:numel(scenario_names)
    scenario_name = scenario_names{index};
    result = summary.acceptance.(scenario_name);
    fprintf("  %-16s accepted=%d, settling=%.3f s, final=%.4f deg\n", ...
        scenario_name, result.accepted, result.metrics.settling_time, ...
        result.metrics.final_abs_tilt_deg);
end
fprintf("  MATLAB/Simulink maximum tilt difference: %.6f deg\n", ...
    summary.comparison.maximum_tilt_error_deg);
fprintf("  Model:  %s\n", summary.model_path);
fprintf("  Data:   %s\n", summary.results_path);
fprintf("  Figure: %s\n\n", summary.figure_path);
end

function close_if_valid(figure_handle)
if isgraphics(figure_handle)
    close(figure_handle);
end
end
