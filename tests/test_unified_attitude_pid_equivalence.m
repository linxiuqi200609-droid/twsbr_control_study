function tests = test_unified_attitude_pid_equivalence
%TEST_UNIFIED_ATTITUDE_PID_EQUIVALENCE Guard the legacy attitude trajectory.
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

function test_positive_tilt_matches_legacy_at_machine_precision(test_case)
plant = twsbr_params();
pid_params = attitude_pid_params(struct(), plant);
config = experiment_config("quick");
scenarios = attitude_pid_scenarios();
legacy_scenario = scenarios.positive_tilt;
generic_scenario = struct( ...
    "name", legacy_scenario.name, ...
    "duration", legacy_scenario.duration, ...
    "initial_state", legacy_scenario.initial_state, ...
    "x_reference", legacy_scenario.theta_reference, ...
    "force_disturbance", legacy_scenario.force_disturbance, ...
    "torque_disturbance", legacy_scenario.torque_disturbance, ...
    "reference_start", 0.0, ...
    "disturbance_end", 0.0, ...
    "measurement_noise_std", zeros(4, 1));

legacy = simulate_attitude_pid(plant, pid_params, legacy_scenario);
common = simulate_control_system("ATTITUDE_PID", ...
    log10([pid_params.kp, pid_params.ki, pid_params.kd]), ...
    plant, config, generic_scenario, 0);

verify_trajectory_equivalence(test_case, common, legacy);
end

function verify_trajectory_equivalence(test_case, common, legacy)
verifyEqual(test_case, size(common.time), size(legacy.time));
verifyEqual(test_case, size(common.state), size(legacy.state));
verifyEqual(test_case, size(common.u_raw), size(legacy.u_raw));
verifyEqual(test_case, size(common.u), size(legacy.u));
if ~isequal(size(common.time), size(legacy.time)) || ...
        ~isequal(size(common.state), size(legacy.state)) || ...
        ~isequal(size(common.u_raw), size(legacy.u_raw)) || ...
        ~isequal(size(common.u), size(legacy.u))
    return
end

maximum_time_difference = max(abs(common.time - legacy.time), [], "all");
maximum_state_difference = max(abs(common.state - legacy.state), [], "all");
maximum_raw_control_difference = max( ...
    abs(common.u_raw - legacy.u_raw), [], "all");
maximum_applied_input_difference = max(abs(common.u - legacy.u), [], "all");

verifyLessThanOrEqual(test_case, maximum_time_difference, 1e-14, ...
    sprintf("Maximum time difference: %.17g", maximum_time_difference));
verifyLessThanOrEqual(test_case, maximum_state_difference, 1e-12, ...
    sprintf("Maximum state difference: %.17g", maximum_state_difference));
verifyLessThanOrEqual(test_case, maximum_raw_control_difference, 1e-12, ...
    sprintf("Maximum raw control difference: %.17g", ...
    maximum_raw_control_difference));
verifyLessThanOrEqual(test_case, maximum_applied_input_difference, 1e-12, ...
    sprintf("Maximum applied input difference: %.17g", ...
    maximum_applied_input_difference));
verifyEqual(test_case, common.time, legacy.time, "AbsTol", 1e-14);
verifyEqual(test_case, common.state, legacy.state, "AbsTol", 1e-12);
verifyEqual(test_case, common.u_raw, legacy.u_raw, "AbsTol", 1e-12);
verifyEqual(test_case, common.u, legacy.u, "AbsTol", 1e-12);
verifyEqual(test_case, common.saturated, legacy.saturated);
verifyEqual(test_case, common.success, legacy.success);
verifyEqual(test_case, common.failure_reason, legacy.failure_reason);
end
