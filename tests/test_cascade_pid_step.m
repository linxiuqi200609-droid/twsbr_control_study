function tests = test_cascade_pid_step
%TEST_CASCADE_PID_STEP Unit tests for the discrete cascade PID controller.
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

function test_defaults_are_frozen_and_plant_limit_is_used(test_case)
plant_params = twsbr_params(struct("u_max", 1.7));
params = cascade_pid_params(struct(), plant_params);

verifyEqual(test_case, params.kp_x, 0.24100028146267993, "AbsTol", 1e-15);
verifyEqual(test_case, params.ki_x, 0.0003962067755988572, "AbsTol", 1e-18);
verifyEqual(test_case, params.kd_x, 0.1930824033173246, "AbsTol", 1e-15);
verifyEqual(test_case, params.kp_theta, 9.254929149177556, "AbsTol", 1e-14);
verifyEqual(test_case, params.kd_theta, 1.0113335430173094, "AbsTol", 1e-15);
verifyEqual(test_case, params.sample_time, 0.01, "AbsTol", 1e-15);
verifyEqual(test_case, params.plant_step, 0.001, "AbsTol", 1e-15);
verifyEqual(test_case, params.theta_reference_limit, deg2rad(12), "AbsTol", 1e-15);
verifyEqual(test_case, params.position_integral_limit, 1e6, "AbsTol", 1e-6);
verifyEqual(test_case, params.u_max, 1.7, "AbsTol", 1e-15);
end

function test_parameter_overrides_and_scalar_actuator_limit_are_validated(test_case)
overrides = struct("kp_x", 0.3, "plant_step", 0.002, ...
    "sample_time", 0.02, "u_max", 2.4);
params = cascade_pid_params(overrides, 1.8);

verifyEqual(test_case, params.kp_x, 0.3, "AbsTol", 1e-15);
verifyEqual(test_case, params.plant_step, 0.002, "AbsTol", 1e-15);
verifyEqual(test_case, params.sample_time, 0.02, "AbsTol", 1e-15);
verifyEqual(test_case, params.u_max, 2.4, "AbsTol", 1e-15);
verifyError(test_case, @() cascade_pid_params(struct("unknown", 1), 1.0), ...
    "twsbr:cascade_pid:unknown_field");
verifyError(test_case, @() cascade_pid_params(struct("kd_x", NaN), 1.0), ...
    "twsbr:cascade_pid:invalid_value");
verifyError(test_case, @() cascade_pid_params(struct(), 0.0), ...
    "twsbr:cascade_pid:invalid_plant_params");
verifyError(test_case, @() cascade_pid_params( ...
    struct("sample_time", 0.015, "plant_step", 0.01), 1.0), ...
    "twsbr:cascade_pid:invalid_time_ratio");
end

function test_hand_derived_sign_convention(test_case)
params = cascade_pid_params(struct( ...
    "kp_x", 0.1, "ki_x", 0.0, "kd_x", 0.0, ...
    "kp_theta", 2.0, "kd_theta", 0.0, ...
    "theta_reference_limit", 1.0, "u_max", 10.0), 10.0);
controller_state = struct("position_integral", 0.0);
plant_state = [0.0; 0.0; 0.0; 0.0];

[control, next_state] = cascade_pid_step( ...
    controller_state, plant_state, 0.5, params);

verifyEqual(test_case, control.theta_reference, 0.05, "AbsTol", 1e-12);
verifyEqual(test_case, control.theta_error, -0.05, "AbsTol", 1e-12);
verifyEqual(test_case, control.u_raw, -0.1, "AbsTol", 1e-12);
verifyEqual(test_case, control.u, -0.1, "AbsTol", 1e-12);
verifyFalse(test_case, control.saturated);
verifyEqual(test_case, control.position_integral, 0.0, "AbsTol", 1e-12);
verifyEqual(test_case, next_state.position_integral, 0.005, "AbsTol", 1e-12);
end

function test_outer_derivative_uses_measured_position_rate(test_case)
params = make_params(struct("kp_x", 0.0, "ki_x", 0.0, "kd_x", 0.5, ...
    "kp_theta", 1.0, "kd_theta", 0.0));
[control, ~] = cascade_pid_step(struct("position_integral", 0.0), ...
    [0.0; 0.4; 0.0; 0.0], 0.0, params);

verifyEqual(test_case, control.theta_reference_raw, -0.2, "AbsTol", 1e-12);
verifyEqual(test_case, control.theta_error, 0.2, "AbsTol", 1e-12);
verifyEqual(test_case, control.u_raw, 0.2, "AbsTol", 1e-12);
end

function test_inner_derivative_uses_measured_attitude_rate(test_case)
params = make_params(struct("kp_x", 0.0, "ki_x", 0.0, "kd_x", 0.0, ...
    "kp_theta", 0.0, "kd_theta", 0.7));
[control, ~] = cascade_pid_step(struct("position_integral", 0.0), ...
    [0.0; 0.0; 0.0; -0.3], 0.0, params);

verifyEqual(test_case, control.u_raw, -0.21, "AbsTol", 1e-12);
end

function test_positive_reference_is_clipped(test_case)
params = make_params(struct("kp_x", 2.0, "ki_x", 0.0, "kd_x", 0.0, ...
    "kp_theta", 1.0, "theta_reference_limit", 0.2));
[control, ~] = cascade_pid_step(struct("position_integral", 0.0), ...
    [0.0; 0.0; 0.0; 0.0], 1.0, params);

verifyEqual(test_case, control.theta_reference_raw, 2.0, "AbsTol", 1e-12);
verifyEqual(test_case, control.theta_reference, 0.2, "AbsTol", 1e-12);
end

function test_negative_reference_is_clipped(test_case)
params = make_params(struct("kp_x", 2.0, "ki_x", 0.0, "kd_x", 0.0, ...
    "kp_theta", 1.0, "theta_reference_limit", 0.2));
[control, ~] = cascade_pid_step(struct("position_integral", 0.0), ...
    [0.0; 0.0; 0.0; 0.0], -1.0, params);

verifyEqual(test_case, control.theta_reference_raw, -2.0, "AbsTol", 1e-12);
verifyEqual(test_case, control.theta_reference, -0.2, "AbsTol", 1e-12);
end

function test_actuator_saturation_reports_clipped_control(test_case)
params = make_params(struct("kp_x", 0.0, "ki_x", 0.0, "kd_x", 0.0, ...
    "kp_theta", 3.0, "kd_theta", 0.0, "u_max", 0.4));
[control, ~] = cascade_pid_step(struct("position_integral", 0.0), ...
    [0.0; 0.0; 1.0; 0.0], 0.0, params);

verifyEqual(test_case, control.u_raw, 3.0, "AbsTol", 1e-12);
verifyEqual(test_case, control.u, 0.4, "AbsTol", 1e-12);
verifyTrue(test_case, control.saturated);
end

function test_integral_updates_when_actuator_is_saturated(test_case)
params = make_params(struct("kp_x", 0.0, "ki_x", 0.0, "kd_x", 0.0, ...
    "kp_theta", 3.0, "kd_theta", 0.0, "u_max", 0.4));
[~, next_state] = cascade_pid_step(struct("position_integral", 0.0), ...
    [0.0; 0.0; 1.0; 0.0], 0.5, params);

verifyEqual(test_case, next_state.position_integral, 0.005, "AbsTol", 1e-12);
end

function test_position_integral_is_clamped(test_case)
params = make_params(struct("position_integral_limit", 0.1));
[~, next_state] = cascade_pid_step(struct("position_integral", 0.099), ...
    [0.0; 0.0; 0.0; 0.0], 1.0, params);

verifyEqual(test_case, next_state.position_integral, 0.1, "AbsTol", 1e-12);
end

function test_zero_position_integral_resets_controller_state(test_case)
params = make_params(struct("kp_x", 0.0, "ki_x", 2.0, "kd_x", 0.0, ...
    "kp_theta", 1.0, "kd_theta", 0.0));
[control, next_state] = cascade_pid_step(struct("position_integral", 0.0), ...
    [0.0; 0.0; 0.0; 0.0], 0.0, params);

verifyEqual(test_case, control.theta_reference_raw, 0.0, "AbsTol", 1e-12);
verifyEqual(test_case, next_state.position_integral, 0.0, "AbsTol", 1e-12);
end

function test_nonfinite_inputs_are_rejected(test_case)
params = make_params(struct());
verifyError(test_case, @() cascade_pid_step( ...
    struct("position_integral", NaN), [0.0; 0.0; 0.0; 0.0], 0.0, params), ...
    "twsbr:cascade_pid:invalid_input");
verifyError(test_case, @() cascade_pid_step( ...
    struct("position_integral", 0.0), [0.0; Inf; 0.0; 0.0], 0.0, params), ...
    "twsbr:cascade_pid:invalid_input");
verifyError(test_case, @() cascade_pid_step( ...
    struct("position_integral", 0.0), [0.0; 0.0; 0.0; 0.0], NaN, params), ...
    "twsbr:cascade_pid:invalid_input");
end

function test_identical_calls_are_deterministic(test_case)
params = make_params(struct());
controller_state = struct("position_integral", 0.2);
plant_state = [0.1; -0.2; 0.03; 0.04];

[first_control, first_state] = cascade_pid_step( ...
    controller_state, plant_state, 0.4, params);
[second_control, second_state] = cascade_pid_step( ...
    controller_state, plant_state, 0.4, params);

verifyEqual(test_case, second_control, first_control);
verifyEqual(test_case, second_state, first_state);
end

function test_control_reports_exact_fields_and_current_integral(test_case)
params = make_params(struct("kp_x", 0.0, "ki_x", 2.0, "kd_x", 0.0, ...
    "kp_theta", 1.0, "kd_theta", 0.0, "theta_reference_limit", 1.0));
[control, next_state] = cascade_pid_step(struct("position_integral", 0.3), ...
    [0.0; 0.0; 0.0; 0.0], 0.4, params);

expected_fields = sort(cellstr(["position_integral"; "position_error"; ...
    "theta_reference_raw"; "theta_reference"; "theta_error"; "u_raw"; ...
    "u"; "saturated"]));
verifyEqual(test_case, sort(fieldnames(control)), expected_fields);
verifyEqual(test_case, fieldnames(next_state), {'position_integral'});
verifyEqual(test_case, control.position_integral, 0.3, "AbsTol", 1e-12);
verifyEqual(test_case, control.theta_reference_raw, 0.6, "AbsTol", 1e-12);
verifyEqual(test_case, next_state.position_integral, 0.304, "AbsTol", 1e-12);
end

function test_inner_loop_uses_clipped_attitude_reference(test_case)
params = make_params(struct("kp_x", 10.0, "ki_x", 0.0, "kd_x", 0.0, ...
    "kp_theta", 2.0, "kd_theta", 0.0, "theta_reference_limit", 0.2));
[control, ~] = cascade_pid_step(struct("position_integral", 0.0), ...
    [0.0; 0.0; 0.5; 0.0], 1.0, params);

verifyEqual(test_case, control.theta_reference_raw, 10.0, "AbsTol", 1e-12);
verifyEqual(test_case, control.theta_reference, 0.2, "AbsTol", 1e-12);
verifyEqual(test_case, control.theta_error, 0.3, "AbsTol", 1e-12);
verifyEqual(test_case, control.u_raw, 0.6, "AbsTol", 1e-12);
end

function test_negative_saturation_and_integral_lower_limit(test_case)
params = make_params(struct("kp_x", 0.0, "ki_x", 0.0, "kd_x", 0.0, ...
    "kp_theta", 3.0, "kd_theta", 0.0, "u_max", 0.4, ...
    "position_integral_limit", 0.1));
[control, next_state] = cascade_pid_step(struct("position_integral", -0.099), ...
    [0.0; 0.0; -1.0; 0.0], -1.0, params);

verifyEqual(test_case, control.u_raw, -3.0, "AbsTol", 1e-12);
verifyEqual(test_case, control.u, -0.4, "AbsTol", 1e-12);
verifyTrue(test_case, control.saturated);
verifyEqual(test_case, next_state.position_integral, -0.1, "AbsTol", 1e-12);
end

function test_invalid_parameter_and_override_shapes_are_rejected(test_case)
verifyError(test_case, @() cascade_pid_params(struct("kp_x", [0.1, 0.2]), 1.0), ...
    "twsbr:cascade_pid:invalid_value");
verifyError(test_case, @() cascade_pid_params(struct("kp_x", 1.0 + 1.0i), 1.0), ...
    "twsbr:cascade_pid:invalid_value");
verifyError(test_case, @() cascade_pid_params(struct("kp_x", Inf), 1.0), ...
    "twsbr:cascade_pid:invalid_value");
verifyError(test_case, @() cascade_pid_params(1.0, 1.0), ...
    "twsbr:cascade_pid:invalid_overrides");
verifyError(test_case, @() cascade_pid_params( ...
    struct("kp_x", {0.1, 0.2}), 1.0), ...
    "twsbr:cascade_pid:invalid_overrides");
end

function test_invalid_state_and_plant_state_shape_are_rejected(test_case)
params = make_params(struct());
verifyError(test_case, @() cascade_pid_step( ...
    struct("position_integral", 0.0, "extra", 1.0), ...
    [0.0; 0.0; 0.0; 0.0], 0.0, params), ...
    "twsbr:cascade_pid:invalid_state");
verifyError(test_case, @() cascade_pid_step( ...
    struct("position_integral", 0.0), [0.0; 0.0; 0.0], 0.0, params), ...
    "twsbr:cascade_pid:invalid_input");
verifyError(test_case, @() cascade_pid_step( ...
    struct("position_integral", 0.0), zeros(2, 2), 0.0, params), ...
    "twsbr:cascade_pid:invalid_input");
end

function test_overflowing_intermediate_or_next_state_is_rejected(test_case)
raw_overflow_params = make_params(struct("kp_x", realmax));
verifyError(test_case, @() cascade_pid_step( ...
    struct("position_integral", 0.0), [0.0; 0.0; 0.0; 0.0], realmax, ...
    raw_overflow_params), "twsbr:cascade_pid:nonfinite_output");

integral_overflow_params = make_params(struct("kp_x", 0.0, "ki_x", 0.0, ...
    "kd_x", 0.0));
verifyError(test_case, @() cascade_pid_step( ...
    struct("position_integral", realmax), [0.0; 0.0; 0.0; 0.0], realmax, ...
    integral_overflow_params), "twsbr:cascade_pid:nonfinite_output");
end
function test_subsample_ratio_and_invalid_plant_steps_are_rejected(test_case)
verifyError(test_case, @() cascade_pid_params( ...
    struct("sample_time", 1e-13, "plant_step", 1.0), 1.0), ...
    "twsbr:cascade_pid:invalid_time_ratio");
verifyError(test_case, @() cascade_pid_params(struct("plant_step", 0.0), 1.0), ...
    "twsbr:cascade_pid:invalid_value");
verifyError(test_case, @() cascade_pid_params(struct("plant_step", -0.001), 1.0), ...
    "twsbr:cascade_pid:invalid_value");
verifyError(test_case, @() cascade_pid_params(struct("plant_step", Inf), 1.0), ...
    "twsbr:cascade_pid:invalid_value");
end
function params = make_params(overrides)
params = cascade_pid_params(overrides, twsbr_params());
end
