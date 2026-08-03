function tests = test_attitude_pid_controller
%TEST_ATTITUDE_PID_CONTROLLER Unit tests for the basic attitude PID.
tests = functiontests(localfunctions);
end

function test_parameter_overrides_are_applied(test_case)
plant_params = twsbr_params();
overrides = struct("kp", 2.0, "ki", 0.3, "kd", 0.25);
pid_params = attitude_pid_params(overrides, plant_params);

verifyEqual(test_case, pid_params.kp, 2.0, "AbsTol", 1e-12);
verifyEqual(test_case, pid_params.ki, 0.3, "AbsTol", 1e-12);
verifyEqual(test_case, pid_params.kd, 0.25, "AbsTol", 1e-12);
verifyEqual(test_case, pid_params.u_max, plant_params.u_max, "AbsTol", 1e-12);
end

function test_unknown_parameter_is_rejected(test_case)
verifyError(test_case, ...
    @() attitude_pid_params(struct("unknown_gain", 1.0), twsbr_params()), ...
    "twsbr:attitude_pid:unknown_field");
end

function test_invalid_time_ratio_is_rejected(test_case)
overrides = struct("sample_time", 0.015, "plant_step", 0.01);

verifyError(test_case, ...
    @() attitude_pid_params(overrides, twsbr_params()), ...
    "twsbr:attitude_pid:invalid_time_ratio");
end

function test_proportional_term_has_correct_sign(test_case)
pid_params = make_params(2.0, 0.0, 0.0, 1.0);
state = struct("integral_error", 0.0);
[control, ~] = attitude_pid_step(state, 0.1, 0.0, 0.0, pid_params);

verifyEqual(test_case, control.theta_error, 0.1, "AbsTol", 1e-12);
verifyEqual(test_case, control.u_raw, 0.2, "AbsTol", 1e-12);
verifyEqual(test_case, control.u, 0.2, "AbsTol", 1e-12);
end

function test_integral_term_uses_current_state(test_case)
pid_params = make_params(0.0, 2.0, 0.0, 2.0);
state = struct("integral_error", 0.1);
[control, ~] = attitude_pid_step(state, 0.0, 0.0, 0.0, pid_params);

verifyEqual(test_case, control.u_raw, 0.2, "AbsTol", 1e-12);
end

function test_derivative_term_uses_measured_angular_velocity(test_case)
pid_params = make_params(0.0, 0.0, 0.5, 1.0);
state = struct("integral_error", 0.0);
[control, ~] = attitude_pid_step(state, 0.0, 0.2, 0.0, pid_params);

verifyEqual(test_case, control.u_raw, 0.1, "AbsTol", 1e-12);
end

function test_saturation_stops_integral_growth(test_case)
pid_params = make_params(20.0, 1.0, 0.0, 0.5);
state = struct("integral_error", 0.0);
[control, next_state] = attitude_pid_step(state, 0.1, 0.0, 0.0, pid_params);

verifyTrue(test_case, control.saturated);
verifyEqual(test_case, control.u_raw, 2.0, "AbsTol", 1e-12);
verifyEqual(test_case, control.u, 1.0, "AbsTol", 1e-12);
verifyEqual(test_case, next_state.integral_error, 0.0, "AbsTol", 1e-12);
end

function test_integral_updates_when_unsaturated(test_case)
pid_params = make_params(0.0, 1.0, 0.0, 0.5);
state = struct("integral_error", 0.0);
[~, next_state] = attitude_pid_step(state, 0.1, 0.0, 0.0, pid_params);

verifyEqual(test_case, next_state.integral_error, 0.001, "AbsTol", 1e-12);
end

function test_integral_updates_when_error_reduces_saturation(test_case)
pid_params = make_params(0.0, 2.0, 0.0, 2.0);
state = struct("integral_error", 1.0);
[control, next_state] = attitude_pid_step(state, -0.1, 0.0, 0.0, pid_params);

verifyTrue(test_case, control.saturated);
verifyEqual(test_case, next_state.integral_error, 0.999, "AbsTol", 1e-12);
end

function test_integral_state_is_clamped(test_case)
pid_params = make_params(0.0, 0.0, 0.0, 0.5);
state = struct("integral_error", 0.4999);
[~, next_state] = attitude_pid_step(state, 0.1, 0.0, 0.0, pid_params);

verifyEqual(test_case, next_state.integral_error, 0.5, "AbsTol", 1e-12);
end

function pid_params = make_params(kp, ki, kd, integral_limit)
overrides = struct( ...
    "kp", kp, ...
    "ki", ki, ...
    "kd", kd, ...
    "integral_limit", integral_limit);
pid_params = attitude_pid_params(overrides, twsbr_params());
end
