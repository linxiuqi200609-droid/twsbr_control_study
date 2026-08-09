function tests = test_attitude_pid_simulink
%TEST_ATTITUDE_PID_SIMULINK Integration tests for the closed loop model.
tests = functiontests(localfunctions);
end

function setupOnce(~)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function test_model_builds_without_warnings(test_case)
verifyNotEmpty(test_case, ver("simulink"));
lastwarn("");
model_path = build_attitude_pid_simulink();
[warning_message, ~] = lastwarn;

verifyTrue(test_case, isfile(model_path));
verifyEqual(test_case, string(warning_message), "");
end

function test_all_model_block_names_are_english(test_case)
model_path = build_attitude_pid_simulink();
model_name = "twsbr_attitude_pid";
load_system(model_path);
cleanup = onCleanup(@() close_if_loaded(model_name));
blocks = find_system(model_name, "Type", "Block");

for index = 1:numel(blocks)
    block_name = string(get_param(blocks{index}, "Name"));
    verifyNotEmpty(test_case, regexp(block_name, ...
        "^[A-Za-z][A-Za-z0-9_]*$", "once"), block_name);
end
end

function test_zero_state_remains_at_equilibrium(test_case)
scenarios = attitude_pid_scenarios();
simulation = run_attitude_pid_simulink( ...
    twsbr_params(), attitude_pid_params(), scenarios.zero_state);

verifyTrue(test_case, simulation.success);
verifyLessThan(test_case, max(abs(simulation.state), [], "all"), 1e-10);
end

function test_applied_control_respects_actuator_limit(test_case)
scenarios = attitude_pid_scenarios();
pid_params = attitude_pid_params();
simulation = run_attitude_pid_simulink( ...
    twsbr_params(), pid_params, scenarios.positive_tilt);

verifyLessThanOrEqual(test_case, max(abs(simulation.u)), ...
    pid_params.u_max + 1e-12);
verifyTrue(test_case, all(isfinite(simulation.state), "all"));
end

function test_positive_tilt_matches_matlab_trajectory(test_case)
plant_params = twsbr_params();
pid_params = attitude_pid_params(struct(), plant_params);
scenarios = attitude_pid_scenarios();
scenario = scenarios.positive_tilt;
matlab_simulation = simulate_attitude_pid( ...
    plant_params, pid_params, scenario);
simulink_simulation = run_attitude_pid_simulink( ...
    plant_params, pid_params, scenario);

matlab_theta = interp1(matlab_simulation.time, ...
    matlab_simulation.state(:, 3), simulink_simulation.time, "pchip");
maximum_tilt_error_deg = rad2deg(max(abs( ...
    matlab_theta - simulink_simulation.state(:, 3))));

verifyLessThan(test_case, maximum_tilt_error_deg, 0.2);
verifyLessThan(test_case, simulink_simulation.metrics.final_abs_tilt_deg, 0.5);
end

function close_if_loaded(model_name)
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
end
