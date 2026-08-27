function tests = test_control_metrics
%TEST_CONTROL_METRICS Tests for unified control-study metrics.
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function teardownOnce(test_case)
path(test_case.TestData.original_path);
end

function test_metrics_match_simple_trajectory_and_schema(test_case)
simulation = simple_simulation();
scenario = metric_scenario("linear", 2.0);
metrics = calculate_control_metrics(simulation, scenario, twsbr_params());
expected_fields = ["controller"; "scenario"; "split"; "seed"; ...
    "success"; "simulation_success"; "failure_reason"; ...
    "survived_time"; "theta_rms_deg"; "max_abs_theta_deg"; ...
    "max_abs_theta_rad"; "theta_itae"; "attitude_settling_time"; ...
    "position_itae"; "final_abs_position_error"; ...
    "position_settling_time"; "position_overshoot"; ...
    "position_drift"; "control_energy"; "saturation_time"; ...
    "saturation_ratio"; "disturbance_recovery_time"; ...
    "controller_runtime_seconds"; "mean_step_runtime_us"];

verifyTrue(test_case, isscalar(metrics) && isstruct(metrics));
verifyEqual(test_case, string(fieldnames(metrics)), expected_fields);
verifyEqual(test_case, metrics.controller, "TEST");
verifyEqual(test_case, metrics.scenario, "linear");
verifyEqual(test_case, metrics.split, "test");
verifyEqual(test_case, metrics.seed, 3);
verifyTrue(test_case, metrics.success);
verifyTrue(test_case, metrics.simulation_success);
verifyEqual(test_case, metrics.failure_reason, "");
verifyEqual(test_case, metrics.position_itae, 0.5, "AbsTol", 1e-12);
verifyEqual(test_case, metrics.control_energy, 0.75, "AbsTol", 1e-12);
verifyEqual(test_case, metrics.max_abs_theta_rad, 0.1, "AbsTol", 1e-12);
verifyEqual(test_case, metrics.max_abs_theta_deg, rad2deg(0.1), ...
    "AbsTol", 1e-12);
verifyEqual(test_case, metrics.theta_rms_deg, ...
    rad2deg(0.1 / sqrt(3)), "AbsTol", 1e-12);
verifyEqual(test_case, metrics.saturation_time, 0.5, "AbsTol", 1e-12);
verifyEqual(test_case, metrics.saturation_ratio, 1 / 3, "AbsTol", 1e-12);
verifyEqual(test_case, metrics.controller_runtime_seconds, 0.003);
verifyEqual(test_case, metrics.mean_step_runtime_us, 1000);
verify_finite_numeric_metrics(test_case, metrics);
end

function test_reverse_scan_settling_and_recovery_times(test_case)
simulation = simple_simulation();
simulation.time = (0:0.5:2).';
simulation.state = [0, 0, deg2rad(3.0), 0; ...
    0.9, 0, deg2rad(1.0), 0; ...
    0.85, 0, deg2rad(2.5), 0; ...
    0.94, 0, deg2rad(1.0), 0; ...
    0.96, 0, deg2rad(1.0), 0];
simulation.position_reference = ones(5, 1);
simulation.u_raw = zeros(5, 1);
simulation.u = zeros(5, 1);
simulation.saturated = false(5, 1);
simulation.survived_time = 2.0;
simulation.runtime_seconds = 0.005;
scenario = metric_scenario("settling", 2.0);
scenario.reference_start = 0.5;
scenario.disturbance_end = 1.0;

metrics = calculate_control_metrics(simulation, scenario, twsbr_params());

verifyEqual(test_case, metrics.attitude_settling_time, 1.5);
verifyEqual(test_case, metrics.position_settling_time, 1.5);
verifyEqual(test_case, metrics.disturbance_recovery_time, 0.5);
verifyEqual(test_case, metrics.position_overshoot, 0.0);
verifyEqual(test_case, metrics.position_drift, 0.96);

simulation.state(:, 1) = 0;
simulation.state(:, 3) = deg2rad(4);
unsettled = calculate_control_metrics(simulation, scenario, twsbr_params());
verifyEqual(test_case, unsettled.attitude_settling_time, scenario.duration);
verifyEqual(test_case, unsettled.position_settling_time, scenario.duration);
verifyEqual(test_case, unsettled.disturbance_recovery_time, scenario.duration);
end

function test_position_overshoot_follows_target_direction(test_case)
simulation = simple_simulation();
simulation.state(:, 1) = [0; 1.3; 0.9];
positive = calculate_control_metrics(simulation, ...
    metric_scenario("positive", 2.0), twsbr_params());
verifyEqual(test_case, positive.position_overshoot, 0.3, "AbsTol", 1e-15);

simulation.position_reference = -ones(3, 1);
simulation.state(:, 1) = [0; -1.4; -0.9];
negative = calculate_control_metrics(simulation, ...
    metric_scenario("negative", 2.0), twsbr_params());
verifyEqual(test_case, negative.position_overshoot, 0.4, "AbsTol", 1e-15);

simulation.position_reference = zeros(3, 1);
zero_target = calculate_control_metrics(simulation, ...
    metric_scenario("zero", 2.0), twsbr_params());
verifyEqual(test_case, zero_target.position_overshoot, 0.0);
end

function test_task_failure_is_reported_without_overwriting_simulation_failure(test_case)
simulation = make_unsettled_successful_simulation();
scenario = metric_scenario("unsettled", simulation.time(end));
metrics = calculate_control_metrics(simulation, scenario, twsbr_params());
verifyFalse(test_case, metrics.success);
verifyTrue(test_case, metrics.simulation_success);
verifyEqual(test_case, metrics.failure_reason, "task_not_settled");

simulation.success = false;
simulation.failure_reason = "tilt_limit";
failed = calculate_control_metrics(simulation, scenario, twsbr_params());
verifyFalse(test_case, failed.success);
verifyFalse(test_case, failed.simulation_success);
verifyEqual(test_case, failed.failure_reason, "tilt_limit");
end

function test_empty_failed_simulation_returns_finite_metrics(test_case)
simulation = simple_simulation();
simulation.time = zeros(0, 1);
simulation.state = zeros(0, 4);
simulation.position_reference = zeros(0, 1);
simulation.u_raw = zeros(0, 1);
simulation.u = zeros(0, 1);
simulation.saturated = false(0, 1);
simulation.success = false;
simulation.failure_reason = "nonfinite_control";
simulation.survived_time = 0.0;
simulation.runtime_seconds = 0.002;
scenario = metric_scenario("empty_failure", 2.0);
scenario.x_reference = @(~) 0.75;

metrics = calculate_control_metrics(simulation, scenario, twsbr_params());

verifyFalse(test_case, metrics.success);
verifyEqual(test_case, metrics.failure_reason, "nonfinite_control");
verifyEqual(test_case, metrics.theta_rms_deg, 0.0);
verifyEqual(test_case, metrics.max_abs_theta_deg, 0.0);
verifyEqual(test_case, metrics.theta_itae, 0.0);
verifyEqual(test_case, metrics.position_itae, 0.0);
verifyEqual(test_case, metrics.control_energy, 0.0);
verifyEqual(test_case, metrics.saturation_time, 0.0);
verifyEqual(test_case, metrics.saturation_ratio, 0.0);
verifyEqual(test_case, metrics.final_abs_position_error, 0.75);
verifyEqual(test_case, metrics.attitude_settling_time, 2.0);
verifyEqual(test_case, metrics.position_settling_time, 2.0);
verifyEqual(test_case, metrics.disturbance_recovery_time, 0.0);
verifyEqual(test_case, metrics.mean_step_runtime_us, 2000.0);
verify_finite_numeric_metrics(test_case, metrics);
end

function test_extreme_finite_failed_log_produces_only_finite_metrics(test_case)
simulation = simple_simulation();
simulation.state = [realmax / 2, 0, realmax / 2, 0; ...
    -realmax / 2, 0, -realmax / 2, 0; ...
    realmax / 2, 0, realmax / 2, 0];
simulation.position_reference = ...
    [-realmax / 2; realmax / 2; -realmax / 2];
simulation.u_raw = realmax * ones(3, 1);
simulation.u = realmax * ones(3, 1);
simulation.success = false;
simulation.failure_reason = "nonfinite_state";
metrics = calculate_control_metrics(simulation, ...
    metric_scenario("extreme_failed", 2.0), twsbr_params());

verifyFalse(test_case, metrics.success);
verifyEqual(test_case, metrics.failure_reason, "nonfinite_state");
verify_finite_numeric_metrics(test_case, metrics);
end

function test_duration_must_be_strictly_positive(test_case)
simulation = empty_failed_simulation();
scenario = metric_scenario("zero_duration", 0.0);

verifyError(test_case, @() calculate_control_metrics( ...
    simulation, scenario, twsbr_params()), ...
    "twsbr:metrics:invalid_input");
end

function test_survival_and_success_timing_contracts_are_enforced(test_case)
simulation = simple_simulation();
scenario = metric_scenario("timing", 2.0);

past_duration = simulation;
past_duration.survived_time = 2.1;
verifyError(test_case, @() calculate_control_metrics( ...
    past_duration, scenario, twsbr_params()), ...
    "twsbr:metrics:invalid_input");

short_survival = simulation;
short_survival.survived_time = 1.9;
verifyError(test_case, @() calculate_control_metrics( ...
    short_survival, scenario, twsbr_params()), ...
    "twsbr:metrics:invalid_input");

short_log = simulation;
short_log.time(end) = 1.9;
verifyError(test_case, @() calculate_control_metrics( ...
    short_log, scenario, twsbr_params()), ...
    "twsbr:metrics:invalid_input");

failed_past_duration = simulation;
failed_past_duration.success = false;
failed_past_duration.failure_reason = "tilt_limit";
failed_past_duration.survived_time = 2.1;
verifyError(test_case, @() calculate_control_metrics( ...
    failed_past_duration, scenario, twsbr_params()), ...
    "twsbr:metrics:invalid_input");

time_tolerance = 4 * eps(scenario.duration);
within_tolerance = simulation;
within_tolerance.survived_time = scenario.duration - time_tolerance;
within_tolerance.time(end) = scenario.duration - time_tolerance;
metrics = calculate_control_metrics(within_tolerance, scenario, twsbr_params());
verifyTrue(test_case, metrics.simulation_success);
end

function test_malformed_time_is_rejected_before_time_arithmetic(test_case)
simulation = simple_simulation();
scenario = metric_scenario("malformed_time", 2.0);
bad_cell = simulation;
bad_cell.time = {0; 1; 2};
bad_struct = simulation;
bad_struct.time = struct("sample", {0; 1; 2});

verifyError(test_case, @() calculate_control_metrics( ...
    bad_cell, scenario, twsbr_params()), ...
    "twsbr:metrics:invalid_input");
verifyError(test_case, @() calculate_control_metrics( ...
    bad_struct, scenario, twsbr_params()), ...
    "twsbr:metrics:invalid_input");
end

function test_failed_log_survival_time_matches_published_samples(test_case)
scenario = metric_scenario("failed_timing", 2.0);
plant = twsbr_params();
empty_failure = empty_failed_simulation();
empty_failure.survived_time = 0.1;
verifyError(test_case, @() calculate_control_metrics( ...
    empty_failure, scenario, plant), "twsbr:metrics:invalid_input");

logged_failure = simple_simulation();
logged_failure.success = false;
logged_failure.failure_reason = "tilt_limit";
logged_failure.survived_time = 1.5;
verifyError(test_case, @() calculate_control_metrics( ...
    logged_failure, scenario, plant), "twsbr:metrics:invalid_input");

time_tolerance = 4 * eps(scenario.duration);
empty_within_tolerance = empty_failed_simulation();
empty_within_tolerance.survived_time = time_tolerance;
empty_metrics = calculate_control_metrics( ...
    empty_within_tolerance, scenario, plant);
verifyFalse(test_case, empty_metrics.simulation_success);

logged_within_tolerance = simple_simulation();
logged_within_tolerance.success = false;
logged_within_tolerance.failure_reason = "tilt_limit";
logged_within_tolerance.survived_time = ...
    logged_within_tolerance.time(end) - time_tolerance;
logged_metrics = calculate_control_metrics( ...
    logged_within_tolerance, scenario, plant);
verifyFalse(test_case, logged_metrics.simulation_success);
end

function test_invalid_inputs_have_stable_error(test_case)
simulation = simple_simulation();
scenario = metric_scenario("valid", 2.0);
plant = twsbr_params();
invalid_calls = { ...
    @() calculate_control_metrics([], scenario, plant), ...
    @() calculate_control_metrics(rmfield(simulation, "state"), scenario, plant), ...
    @() calculate_control_metrics(setfield(simulation, "time", [0; 2; 1]), scenario, plant), ... %#ok<SFLD>
    @() calculate_control_metrics(setfield(simulation, "u", [0; 1]), scenario, plant), ... %#ok<SFLD>
    @() calculate_control_metrics(setfield(simulation, "state", ... %#ok<SFLD>
        [simulation.state(:, 1:3), nan(3, 1)]), scenario, plant), ...
    @() calculate_control_metrics(simulation, rmfield(scenario, "duration"), plant), ...
    @() calculate_control_metrics(simulation, scenario, rmfield(plant, "u_max"))};
for index = 1:numel(invalid_calls)
    verifyError(test_case, invalid_calls{index}, "twsbr:metrics:invalid_input");
end

empty_success = simulation;
empty_success.time = zeros(0, 1);
empty_success.state = zeros(0, 4);
empty_success.position_reference = zeros(0, 1);
empty_success.u_raw = zeros(0, 1);
empty_success.u = zeros(0, 1);
empty_success.saturated = false(0, 1);
verifyError(test_case, @() calculate_control_metrics( ...
    empty_success, scenario, plant), "twsbr:metrics:invalid_input");
end

function simulation = empty_failed_simulation()
simulation = simple_simulation();
simulation.time = zeros(0, 1);
simulation.state = zeros(0, 4);
simulation.position_reference = zeros(0, 1);
simulation.u_raw = zeros(0, 1);
simulation.u = zeros(0, 1);
simulation.saturated = false(0, 1);
simulation.success = false;
simulation.failure_reason = "nonfinite_control";
simulation.survived_time = 0.0;
simulation.runtime_seconds = 0.0;
end

function simulation = simple_simulation()
simulation = struct();
simulation.controller_name = "TEST";
simulation.scenario_name = "linear";
simulation.seed = 3;
simulation.time = [0; 1; 2];
simulation.state = [0, 0, 0, 0; 0.5, 0, 0.1, 0; 1, 0, 0, 0];
simulation.position_reference = [1; 1; 1];
simulation.u_raw = [0; 0.5; 1];
simulation.u = [0; 0.5; 1];
simulation.saturated = [false; false; true];
simulation.success = true;
simulation.failure_reason = "";
simulation.survived_time = 2;
simulation.runtime_seconds = 0.003;
end

function simulation = make_unsettled_successful_simulation()
simulation = struct();
simulation.controller_name = "TEST";
simulation.scenario_name = "unsettled";
simulation.seed = 0;
simulation.time = (0:0.1:1).';
sample_count = numel(simulation.time);
simulation.state = [ones(sample_count, 1), 0.5 * ones(sample_count, 1), ...
    0.1 * ones(sample_count, 1), zeros(sample_count, 1)];
simulation.position_reference = zeros(sample_count, 1);
simulation.u_raw = zeros(sample_count, 1);
simulation.u = zeros(sample_count, 1);
simulation.saturated = false(sample_count, 1);
simulation.success = true;
simulation.failure_reason = "";
simulation.survived_time = 1.0;
simulation.runtime_seconds = 0.001;
end

function scenario = metric_scenario(name, duration)
scenario = struct("name", name, "split", "test", ...
    "reference_start", 0.0, "disturbance_end", 0.0, ...
    "duration", duration);
end

function verify_finite_numeric_metrics(test_case, metrics)
fields = fieldnames(metrics);
for index = 1:numel(fields)
    value = metrics.(fields{index});
    if isnumeric(value)
        verifyTrue(test_case, isscalar(value) && isfinite(value));
    end
end
end
