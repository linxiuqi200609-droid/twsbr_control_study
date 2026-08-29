function tests = test_simulink_equivalence
%TEST_SIMULINK_EQUIVALENCE Validate common MATLAB/Simulink comparisons.
local_functions = localfunctions;
function_names = string(cellfun(@func2str, local_functions, ...
    "UniformOutput", false));
tests = functiontests(local_functions(~endsWith( ...
    function_names, "starter_vectors_for_test")));
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

% Mutation caught: changing a frozen equivalence tolerance or comparing raw
% commands rather than the common saturated/applied command.
function test_comparison_uses_frozen_thresholds(test_case)
matlab_result = make_small_common_result(0.0);
simulink_result = make_small_common_result(0.001);
simulink_result.u_raw = 100.0 * ones(2, 1);
comparison = compare_matlab_simulink(matlab_result, simulink_result, 0.001);

verifyEqual(test_case, comparison.tilt_tolerance_deg, 0.2);
verifyEqual(test_case, comparison.position_tolerance_m, 0.01);
verifyEqual(test_case, comparison.input_tolerance, 0.02);
verifyTrue(test_case, comparison.accepted);
verifyLessThan(test_case, comparison.max_applied_input_difference, 0.02);
end

% Mutation caught: replacing strict less-than acceptance with less-than-or-equal.
function test_comparison_rejects_exact_tolerance_boundaries(test_case)
matlab_result = make_small_common_result(0.0);
simulink_result = make_small_common_result(0.0);

simulink_result.state(:, 3) = deg2rad(0.2);
comparison = compare_matlab_simulink(matlab_result, simulink_result, 0.001);
verifyFalse(test_case, comparison.accepted);

simulink_result = make_small_common_result(0.0);
simulink_result.state(:, 1) = 0.01;
comparison = compare_matlab_simulink(matlab_result, simulink_result, 0.001);
verifyFalse(test_case, comparison.accepted);

simulink_result = make_small_common_result(0.0);
simulink_result.u = 0.02 * ones(2, 1);
comparison = compare_matlab_simulink(matlab_result, simulink_result, 0.001);
verifyFalse(test_case, comparison.accepted);
end

% Mutation caught: accepting malformed results, nonmonotone time, or zero step.
function test_comparison_rejects_invalid_shapes_times_and_steps(test_case)
valid_result = make_small_common_result(0.0);
bad_state = valid_result;
bad_state.state = zeros(2, 3);
verifyError(test_case, @() compare_matlab_simulink( ...
    bad_state, valid_result, 0.001), "twsbr:equivalence:invalid_result");

bad_time = valid_result;
bad_time.time = [0.001; 0.0];
verifyError(test_case, @() compare_matlab_simulink( ...
    bad_time, valid_result, 0.001), "twsbr:equivalence:invalid_time");

verifyError(test_case, @() compare_matlab_simulink( ...
    valid_result, valid_result, 0.0), "twsbr:equivalence:invalid_step");

verifyError(test_case, @() compare_matlab_simulink( ...
    valid_result, valid_result, nan), "twsbr:equivalence:invalid_step");
verifyError(test_case, @() compare_matlab_simulink( ...
    valid_result, valid_result, 0.001 + 1i), "twsbr:equivalence:invalid_step");

empty_time = valid_result;
empty_time.time = zeros(0, 1);
empty_time.state = zeros(0, 4);
empty_time.u = zeros(0, 1);
verifyError(test_case, @() compare_matlab_simulink( ...
    empty_time, valid_result, 0.001), "twsbr:equivalence:invalid_time");

nonfinite_u = valid_result;
nonfinite_u.u(2) = inf;
verifyError(test_case, @() compare_matlab_simulink( ...
    nonfinite_u, valid_result, 0.001), "twsbr:equivalence:invalid_result");

no_overlap = valid_result;
no_overlap.time = [0.01; 0.011];
verifyError(test_case, @() compare_matlab_simulink( ...
    no_overlap, valid_result, 0.001), "twsbr:equivalence:no_overlap");
end

% Mutation caught: ignoring a fuzzy effective-gain disagreement or accepting
% an equality-at-tolerance relative error.
function test_comparison_rejects_fuzzy_gain_mutation(test_case)
matlab_result = with_fuzzy_diagnostics( ...
    make_small_common_result(0.0), 1000000.0);
simulink_result = with_fuzzy_diagnostics( ...
    make_small_common_result(0.0), 1000000.0);
simulink_result.diagnostics(2).kp_theta = 1000001.0;

comparison = compare_matlab_simulink(matlab_result, simulink_result, 0.001);
verifyEqual(test_case, comparison.max_fuzzy_gain_relative_error, 1e-6, ...
    "AbsTol", 1e-18);
verifyFalse(test_case, comparison.fuzzy_gain_accepted);
verifyFalse(test_case, comparison.accepted);
end

% Mutation caught: treating malformed fuzzy diagnostics as absent validation.
function test_comparison_rejects_malformed_fuzzy_diagnostics(test_case)
matlab_result = with_fuzzy_diagnostics(make_small_common_result(0.0), 1.0);
simulink_result = with_fuzzy_diagnostics(make_small_common_result(0.0), 1.0);
simulink_result.diagnostics = simulink_result.diagnostics(1);
verifyError(test_case, @() compare_matlab_simulink( ...
    matlab_result, simulink_result, 0.001), ...
    "twsbr:equivalence:invalid_diagnostics");
end

% Mutation caught: shifting one controller update by a complete controller
% period while leaving state and applied-input samples unchanged.
function test_comparison_rejects_one_controller_period_timing_mutation(test_case)
matlab_result = make_timed_common_result();
simulink_result = make_timed_common_result();
simulink_result.timing = struct( ...
    "u_raw_time", [0.0; 0.01; 0.02], "u_time", [0.0; 0.01; 0.02], ...
    "controller_update_time", [0.0; 0.02], "sample_time", 0.01, ...
    "validated", true);
verifyError(test_case, @() compare_matlab_simulink( ...
    matlab_result, simulink_result, 0.001), ...
    "twsbr:equivalence:invalid_timing");
end

% Mutation caught: rejecting a duration that ends in a valid partial control
% interval, or extrapolating a command rather than holding its final update.
function test_generated_runner_accepts_partial_final_control_interval(test_case)
[vectors, config] = starter_vectors_for_test();
scenarios = representative_scenarios();
scenario = scenarios.position;
scenario.duration = 3.205;
matlab_result = simulate_control_system("LQR", vectors.LQR, twsbr_params(), ...
    config, scenario, config.global_seed);
simulink_result = run_controller_simulink("LQR", vectors.LQR, ...
    twsbr_params(), config, scenario);
comparison = compare_matlab_simulink( ...
    matlab_result, simulink_result, config.plant_step);

verifyEqual(test_case, simulink_result.time(end), 3.205, "AbsTol", 1e-15);
verifyEqual(test_case, simulink_result.timing.controller_update_time(end), ...
    3.20, "AbsTol", 1e-15);
verifyTrue(test_case, comparison.accepted);
end

% Mutation caught: replacing legacy raw log timing with a synthesized schedule
% or discarding the raw axes before the generic runner can validate them.
function test_legacy_runner_exposes_actual_raw_log_timing(test_case)
[vectors, config] = starter_vectors_for_test();
scenarios = representative_scenarios();
params = decode_controller_vector("ATTITUDE_PID", vectors.ATTITUDE_PID, ...
    twsbr_params(), config);
legacy = run_attitude_pid_simulink(twsbr_params(), params, ...
    legacy_attitude_scenario(scenarios.attitude), true);
generic = run_controller_simulink("ATTITUDE_PID", vectors.ATTITUDE_PID, ...
    twsbr_params(), config, scenarios.attitude);

verifyTrue(test_case, isfield(legacy, "timing"));
verifyEqual(test_case, generic.timing.u_raw_time, legacy.timing.u_raw_time);
verifyEqual(test_case, generic.timing.u_time, legacy.timing.u_time);
end

% Mutation caught: accepting an unsupported controller instead of preserving a
% closed five-name dispatch contract.
function test_runner_rejects_unsupported_controller(test_case)
[~, config] = starter_vectors_for_test();
scenario = representative_scenarios();
verifyError(test_case, @() run_controller_simulink("UNKNOWN", 0, ...
    twsbr_params(), config, scenario.attitude), ...
    "twsbr:simulink_runner:unsupported_controller");
end

% Mutation caught: accepting nonzero or malformed measurement noise even
% though the generated Simulink models have no noise input.
function test_runner_rejects_unsupported_measurement_noise(test_case)
[vectors, config] = starter_vectors_for_test();
scenarios = representative_scenarios();
nonzero_noise = scenarios.attitude;
nonzero_noise.measurement_noise_std(1) = 1e-3;
verifyError(test_case, @() run_controller_simulink( ...
    "ATTITUDE_PID", vectors.ATTITUDE_PID, twsbr_params(), config, ...
    nonzero_noise), "twsbr:simulink_runner:invalid_noise");

malformed_noise = scenarios.attitude;
malformed_noise.measurement_noise_std = zeros(3, 1);
verifyError(test_case, @() run_controller_simulink( ...
    "ATTITUDE_PID", vectors.ATTITUDE_PID, twsbr_params(), config, ...
    malformed_noise), "twsbr:simulink_runner:invalid_noise");
end

% Mutation caught: allowing invalid reference or disturbance outputs to reach
% either a model workspace or a legacy runner with inconsistent identifiers.
function test_runner_rejects_invalid_scenario_signal_outputs(test_case)
[vectors, config] = starter_vectors_for_test();
scenarios = representative_scenarios();
invalid_scenarios = repmat(scenarios.attitude, 3, 1);
invalid_scenarios(1).x_reference = @(~) [0.0; 0.0];
invalid_scenarios(2).force_disturbance = @(~) 1.0 + 1.0i;
invalid_scenarios(3).torque_disturbance = @(~) nan;
for index = 1:numel(invalid_scenarios)
    verifyError(test_case, @() run_controller_simulink( ...
        "ATTITUDE_PID", vectors.ATTITUDE_PID, twsbr_params(), config, ...
        invalid_scenarios(index)), "twsbr:simulink_runner:invalid_signal");
end
end

% Mutation caught: omitting a dispatch path, reordering names, changing a
% representative scenario, or returning incomplete common result records.
function test_validation_batch_returns_all_controller_names(test_case)
[vectors, config] = starter_vectors_for_test();
table_out = run_simulink_validation_batch(vectors, twsbr_params(), config);

verifyEqual(test_case, height(table_out), 5);
verifyEqual(test_case, table_out.controller, config.controller_names);
verifyTrue(test_case, all(table_out.accepted));
verifyEqual(test_case, table_out.scenario(1), "T1_initial_tilt_5deg");
verifyEqual(test_case, table_out.scenario(2:end), ...
    repmat("T2_position_step_0p5m", 4, 1));
verifyTrue(test_case, all(ismember(["max_tilt_difference_deg"; ...
    "max_position_difference_m"; "max_applied_input_difference"; ...
    "tilt_tolerance_deg"; "position_tolerance_m"; "input_tolerance"], ...
    string(table_out.Properties.VariableNames))));
end

% Mutation caught: losing normalized result fields or bypassing shared
% saturation when adapting one of the legacy controller runners.
function test_legacy_runner_normalizes_common_result_fields(test_case)
[vectors, config] = starter_vectors_for_test();
scenarios = representative_scenarios();
result = run_controller_simulink("ATTITUDE_PID", vectors.ATTITUDE_PID, ...
    twsbr_params(), config, scenarios.attitude);

required_fields = ["controller_name"; "scenario_name"; "time"; "state"; ...
    "u"; "success"; "failure_reason"; "diagnostics"; "timing"];
verifyTrue(test_case, all(isfield(result, required_fields)));
verifyEqual(test_case, result.controller_name, "ATTITUDE_PID");
verifyEqual(test_case, size(result.state, 2), 4);
verifyEqual(test_case, numel(result.u), numel(result.time));
verifyEqual(test_case, result.timing.sample_time, 0.01, "AbsTol", 1e-15);
verifyTrue(test_case, result.timing.validated);
end

% Mutation caught: omitting effective fuzzy gains or silently accepting a
% material MATLAB/Simulink fuzzy-gain trajectory disagreement.
function test_fuzzy_gain_comparison_is_reported_and_accepted(test_case)
[vectors, config] = starter_vectors_for_test();
table_out = run_simulink_validation_batch(vectors, twsbr_params(), config);
fuzzy_row = table_out.controller == "FUZZY_PID";

verifyEqual(test_case, sum(fuzzy_row), 1);
verifyLessThan(test_case, table_out.max_fuzzy_gain_relative_error(fuzzy_row), 1e-6);
verifyTrue(test_case, table_out.fuzzy_gain_accepted(fuzzy_row));
verifyTrue(test_case, table_out.accepted(fuzzy_row));
end

% Mutation caught: introducing a model build, update, or run warning into the
% representative validation pipeline.
function test_validation_batch_runs_without_warnings(test_case)
[vectors, config] = starter_vectors_for_test();
verifyWarningFree(test_case, @() run_simulink_validation_batch( ...
    vectors, twsbr_params(), config));
end

function scenarios = representative_scenarios()
training = training_scenarios(3.2);
scenarios = struct("attitude", training.T1_initial_tilt_5deg, ...
    "position", training.T2_position_step_0p5m);
end

function result = make_small_common_result(offset)
result = struct();
result.time = [0; 0.001];
result.state = zeros(2,4);
result.state(:,1) = offset;
result.state(:,3) = offset;
result.u = offset*ones(2,1);
end

function result = make_timed_common_result()
result = struct();
result.time = [0.0; 0.01; 0.02];
result.state = zeros(3, 4);
result.u = zeros(3, 1);
end

function result = with_fuzzy_diagnostics(result, gain)
result.diagnostics = repmat(struct("kp_theta", gain, "ki_theta", gain, ...
    "kd_theta", gain), numel(result.time), 1);
end

function scenario = legacy_attitude_scenario(generic_scenario)
scenario = struct( ...
    "name", generic_scenario.name, ...
    "initial_state", generic_scenario.initial_state, ...
    "duration", generic_scenario.duration, ...
    "theta_reference", @(~) 0.0, ...
    "force_disturbance", generic_scenario.force_disturbance, ...
    "torque_disturbance", generic_scenario.torque_disturbance);
end

function [vectors, config] = starter_vectors_for_test()
config = experiment_config("quick");
vectors = struct();
vectors.ATTITUDE_PID = log10([1.9,0.2,0.18]);
vectors.CASCADE_PID = log10([0.241,0.000396,0.193,9.255,1.011]);
vectors.FUZZY_PID = [log10([0.241,0.000396,0.193, ...
    9.255,0.05,1.011]),0.2,0.2,0.2];
vectors.LQR = log10([10,1,200,10,0.1]);
vectors.LQI = log10([10,1,200,10,100,0.1]);
end
