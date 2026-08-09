function tests = test_attitude_pid_determinism
%TEST_ATTITUDE_PID_DETERMINISM Repeated calls must be deterministic.
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

function test_identical_inputs_produce_identical_results(test_case)
pid_params = attitude_pid_params();
controller_state = struct("integral_error", 0.1);

[first_control, first_state] = attitude_pid_step( ...
    controller_state, 0.2, 0.3, 0.05, pid_params);
[second_control, second_state] = attitude_pid_step( ...
    controller_state, 0.2, 0.3, 0.05, pid_params);

verifyEqual(test_case, second_control, first_control);
verifyEqual(test_case, second_state, first_state);
end
