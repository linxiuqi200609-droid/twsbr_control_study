function tests = test_cascade_pid_simulink_equivalence
%TEST_CASCADE_PID_SIMULINK_EQUIVALENCE Compare MATLAB and Simulink cascades.
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function teardownOnce(test_case)
if bdIsLoaded("twsbr_cascade_pid")
    close_system("twsbr_cascade_pid", 0);
end
path(test_case.TestData.original_path);
end

function test_positive_position_step_matches_matlab_trajectory(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct(), plant_params);
scenarios = cascade_pid_scenarios();
scenario = scenarios.positive_position_step;

matlab_simulation = simulate_cascade_pid(plant_params, params, scenario);
simulink_simulation = run_cascade_pid_simulink( ...
    plant_params, params, scenario);

verifyEqual(test_case, fieldnames(simulink_simulation), ...
    fieldnames(matlab_simulation));
verifyEqual(test_case, fieldnames(simulink_simulation.metrics), ...
    fieldnames(matlab_simulation.metrics));
verifyTrue(test_case, simulink_simulation.success);
verifyEqual(test_case, simulink_simulation.failure_reason, "");
verifyEqual(test_case, size(simulink_simulation.state, 2), 4);

sample_count = numel(simulink_simulation.time);
vector_fields = ["position_reference", "position_error", ...
    "theta_reference", "theta_reference_raw", "theta_error", ...
    "u_raw", "u", "disturbance_force", "position_integral", "saturated"];
for index = 1:numel(vector_fields)
    verifyEqual(test_case, ...
        size(simulink_simulation.(vector_fields(index))), [sample_count, 1], ...
        vector_fields(index));
end
verifyTrue(test_case, all(isfinite(simulink_simulation.time)));
verifyTrue(test_case, all(isfinite(simulink_simulation.state), "all"));
verifyTrue(test_case, all(isfinite(struct2array( ...
    simulink_simulation.metrics))));

common_time = (0:params.plant_step:scenario.duration).';
matlab_state = interp1(matlab_simulation.time, matlab_simulation.state, ...
    common_time, "pchip");
simulink_state = interp1(simulink_simulation.time, ...
    simulink_simulation.state, common_time, "pchip");
maximum_tilt_difference_deg = rad2deg(max(abs( ...
    matlab_state(:, 3) - simulink_state(:, 3))));
maximum_position_difference = max(abs( ...
    matlab_state(:, 1) - simulink_state(:, 1)));

verifyLessThan(test_case, maximum_tilt_difference_deg, 0.2);
verifyLessThan(test_case, maximum_position_difference, 0.01);
end
