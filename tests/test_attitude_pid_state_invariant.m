function tests = test_attitude_pid_state_invariant
%TEST_ATTITUDE_PID_STATE_INVARIANT Controller state limit regression tests.
tests = functiontests(localfunctions);
end

function test_saturated_step_clamps_out_of_range_integral_state(test_case)
pid_params = attitude_pid_params();
controller_state = struct( ...
    "integral_error", pid_params.integral_limit + 0.1);

[~, next_state] = attitude_pid_step( ...
    controller_state, 1.0, 0.0, 0.0, pid_params);

verifyEqual(test_case, next_state.integral_error, ...
    pid_params.integral_limit, "AbsTol", 1e-12);
end
