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

function test_delayed_nonfinite_position_reference_is_a_finite_failure(test_case)
verify_delayed_nonfinite_signal(test_case, "x_reference");
end

function test_delayed_nonfinite_force_is_a_finite_failure(test_case)
verify_delayed_nonfinite_signal(test_case, "force_disturbance");
end

function test_delayed_nonfinite_torque_is_a_finite_failure(test_case)
verify_delayed_nonfinite_signal(test_case, "torque_disturbance");
end

function test_unsettled_event_metrics_exceed_the_observed_horizon(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct( ...
    "kp_x", 0.0, "ki_x", 0.0, "kd_x", 0.0, ...
    "kp_theta", 0.0, "kd_theta", 0.0), plant_params);
scenario = cascade_pid_scenarios().zero_state;
scenario.name = "unsettled_event_horizon";
scenario.duration = 0.004;
scenario.x_reference = @(time) 0.5 .* ones(size(time));
scenario.reference_start = 0.001;
scenario.disturbance_end = 0.001;

matlab_result = simulate_cascade_pid(plant_params, params, scenario);
simulink_result = run_cascade_pid_simulink(plant_params, params, scenario);
observed_horizon = 0.003;

verifyTrue(test_case, matlab_result.success);
verifyTrue(test_case, simulink_result.success);
verifyGreaterThan(test_case, ...
    matlab_result.metrics.position_settling_time, observed_horizon);
verifyGreaterThan(test_case, ...
    simulink_result.metrics.position_settling_time, observed_horizon);
verifyGreaterThan(test_case, ...
    matlab_result.metrics.disturbance_recovery_time, observed_horizon);
verifyGreaterThan(test_case, ...
    simulink_result.metrics.disturbance_recovery_time, observed_horizon);
verifyEqual(test_case, simulink_result.metrics.position_settling_time, ...
    matlab_result.metrics.position_settling_time, "AbsTol", 1e-15);
verifyEqual(test_case, simulink_result.metrics.disturbance_recovery_time, ...
    matlab_result.metrics.disturbance_recovery_time, "AbsTol", 1e-15);
verifyFalse(test_case, ...
    matlab_result.metrics.position_settling_time <= observed_horizon);
verifyFalse(test_case, ...
    simulink_result.metrics.disturbance_recovery_time <= observed_horizon);
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

function values = delayed_nan(time)
values = zeros(size(time));
values(time >= 0.003 - 1e-12) = NaN;
end

function verify_delayed_nonfinite_signal(test_case, signal_field)
plant_params = twsbr_params();
params = cascade_pid_params(struct(), plant_params);
scenario = cascade_pid_scenarios().zero_state;
scenario.name = "delayed_nonfinite_" + signal_field;
scenario.duration = 0.004;
scenario.(signal_field) = @delayed_nan;

matlab_result = simulate_cascade_pid(plant_params, params, scenario);
simulink_result = run_cascade_pid_simulink(plant_params, params, scenario);

normal_prefix_scenario = cascade_pid_scenarios().zero_state;
normal_prefix_scenario.name = "normal_prefix_" + signal_field;
normal_prefix_scenario.duration = 0.002;
normal_simulink_prefix = run_cascade_pid_simulink( ...
    plant_params, params, normal_prefix_scenario);

verifyFalse(test_case, matlab_result.success);
verifyFalse(test_case, simulink_result.success);
verifyEqual(test_case, matlab_result.failure_reason, "nonfinite_signal");
verifyEqual(test_case, simulink_result.failure_reason, "nonfinite_signal");
verify_public_numerics_are_finite(test_case, matlab_result);
verify_public_numerics_are_finite(test_case, simulink_result);
verifyEqual(test_case, simulink_result.time(end), matlab_result.time(end), ...
    "AbsTol", 1e-15);
verify_simulink_prefix_matches(test_case, ...
    simulink_result, normal_simulink_prefix);
end

function verify_simulink_prefix_matches(test_case, actual, expected)
verifyEqual(test_case, actual.time, expected.time, "AbsTol", 1e-15);
verifyEqual(test_case, actual.state, expected.state, "AbsTol", 1e-12);
vector_fields = {"position_reference", "position_error", ...
    "theta_reference", "theta_reference_raw", "theta_error", ...
    "u_raw", "u", "disturbance_force", "position_integral"};
for index = 1:numel(vector_fields)
    field_name = vector_fields{index};
    verifyEqual(test_case, actual.(field_name), expected.(field_name), ...
        "AbsTol", 1e-12, field_name);
end
verifyEqual(test_case, actual.saturated, expected.saturated);
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
