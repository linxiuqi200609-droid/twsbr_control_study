function tests = test_twsbr_model
%TEST_TWSBR_MODEL Unit tests for the TWSBR plant model.
tests = functiontests(localfunctions);
end

function test_default_parameters_are_valid(test_case)
params = twsbr_params();

verifyEqual(test_case, params.body_mass, 1.20, "AbsTol", 1e-12);
verifyEqual(test_case, params.wheel_mass_equiv, 0.30, "AbsTol", 1e-12);
verifyGreaterThan(test_case, params.com_length, 0.0);
verifyGreaterThan(test_case, params.body_inertia, 0.0);
verifyGreaterThan(test_case, params.motor_force_gain, 0.0);
verifyGreaterThanOrEqual(test_case, params.viscous_damping, 0.0);
end

function test_upright_zero_input_is_equilibrium(test_case)
params = twsbr_params();
state_dot = twsbr_dynamics(0.0, zeros(4, 1), 0.0, params, 0.0, 0.0);

verifyEqual(test_case, state_dot, zeros(4, 1), "AbsTol", 1e-12);
end

function test_positive_tilt_falls_in_positive_direction(test_case)
params = twsbr_params();
state = [0.0; 0.0; deg2rad(3.0); 0.0];
state_dot = twsbr_dynamics(0.0, state, 0.0, params, 0.0, 0.0);

verifyGreaterThan(test_case, state_dot(4), 0.0);
end

function test_positive_control_accelerates_body_negative(test_case)
params = twsbr_params();
state_dot = twsbr_dynamics(0.0, zeros(4, 1), 0.25, params, 0.0, 0.0);

verifyLessThan(test_case, state_dot(4), 0.0);
end

function test_nonpositive_mass_is_rejected(test_case)
overrides = struct("body_mass", 0.0);

verifyError(test_case, @() twsbr_params(overrides), ...
    "twsbr:params:invalid_value");
end

function test_unknown_parameter_is_rejected(test_case)
overrides = struct("unknown_parameter", 1.0);

verifyError(test_case, @() twsbr_params(overrides), ...
    "twsbr:params:unknown_field");
end
