function tests = test_cascade_pid_simulink_review_fixes
%TEST_CASCADE_PID_SIMULINK_REVIEW_FIXES Formal review regressions.
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

function test_nonfinite_raw_control_is_a_finite_failure(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct("kd_theta", realmax), plant_params);
scenario = cascade_pid_scenarios().zero_state;
scenario.name = "raw_control_overflow";
scenario.initial_state(4) = 2.0;
scenario.duration = 0.001;

verify_failure_matches_matlab(test_case, plant_params, params, scenario, ...
    "nonfinite_control");
end

function test_nonfinite_initial_state_matches_matlab_failure(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct(), plant_params);
scenario = short_zero_scenario("nonfinite_initial_state");
scenario.initial_state(1) = NaN;

verify_failure_matches_matlab(test_case, plant_params, params, scenario, ...
    "nonfinite_state");
end

function test_nonfinite_initial_position_reference_matches_matlab(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct(), plant_params);
scenario = short_zero_scenario("nonfinite_position_reference");
scenario.x_reference = @first_sample_nan;

verify_failure_matches_matlab(test_case, plant_params, params, scenario, ...
    "nonfinite_signal");
end

function test_nonfinite_initial_force_matches_matlab(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct(), plant_params);
scenario = short_zero_scenario("nonfinite_force");
scenario.force_disturbance = @first_sample_nan;

verify_failure_matches_matlab(test_case, plant_params, params, scenario, ...
    "nonfinite_signal");
end

function test_nonfinite_initial_torque_matches_matlab(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct(), plant_params);
scenario = short_zero_scenario("nonfinite_torque");
scenario.torque_disturbance = @first_sample_nan;

verify_failure_matches_matlab(test_case, plant_params, params, scenario, ...
    "nonfinite_signal");
end

function test_nondefault_sample_time_is_rejected(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct("sample_time", 0.02), plant_params);
scenario = cascade_pid_scenarios().zero_state;

verifyError(test_case, @() build_cascade_pid_simulink( ...
    plant_params, params), "twsbr:cascade_simulation:invalid_timing");
verifyError(test_case, @() run_cascade_pid_simulink( ...
    plant_params, params, scenario), ...
    "twsbr:cascade_simulation:invalid_timing");
end

function test_nondefault_plant_step_is_rejected(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct("plant_step", 0.002), plant_params);
scenario = cascade_pid_scenarios().zero_state;

verifyError(test_case, @() build_cascade_pid_simulink( ...
    plant_params, params), "twsbr:cascade_simulation:invalid_timing");
verifyError(test_case, @() run_cascade_pid_simulink( ...
    plant_params, params, scenario), ...
    "twsbr:cascade_simulation:invalid_timing");
end

function scenario = short_zero_scenario(name)
scenario = cascade_pid_scenarios().zero_state;
scenario.name = name;
scenario.duration = 0.001;
end

function values = first_sample_nan(time)
values = zeros(size(time));
values(time == 0.0) = NaN;
end

function verify_failure_matches_matlab( ...
    test_case, plant_params, params, scenario, expected_reason)
matlab_result = simulate_cascade_pid(plant_params, params, scenario);
simulink_result = run_cascade_pid_simulink(plant_params, params, scenario);

verifyFalse(test_case, matlab_result.success);
verifyFalse(test_case, simulink_result.success);
verifyEqual(test_case, matlab_result.failure_reason, expected_reason);
verifyEqual(test_case, simulink_result.failure_reason, ...
    matlab_result.failure_reason);
verify_public_numerics_are_finite(test_case, matlab_result);
verify_public_numerics_are_finite(test_case, simulink_result);
end

function verify_public_numerics_are_finite(test_case, simulation)
numeric_fields = {"time", "state", "position_reference", ...
    "position_error", "theta_reference", "theta_reference_raw", ...
    "theta_error", "u_raw", "u", "disturbance_force", ...
    "position_integral", "saturated"};
for index = 1:numel(numeric_fields)
    values = simulation.(numeric_fields{index});
    verifyTrue(test_case, all(isfinite(values), "all"), ...
        numeric_fields{index});
end
metric_names = fieldnames(simulation.metrics);
for index = 1:numel(metric_names)
    value = simulation.metrics.(metric_names{index});
    verifyTrue(test_case, isnumeric(value) && isscalar(value) && ...
        isfinite(value), metric_names{index});
end
end
