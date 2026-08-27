function tests = test_fuzzy_pid_step
%TEST_FUZZY_PID_STEP Tests for the fuzzy self-tuning cascade PID.
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

function test_fuzzy_pid_parameter_space_is_exact(test_case)
space = controller_parameter_space("FUZZY_PID");

verifyEqual(test_case, space.name, "FUZZY_PID");
verifyEqual(test_case, space.parameter_names, ...
    ["kp_x"; "ki_x"; "kd_x"; "kp_theta_base"; "ki_theta_base"; ...
    "kd_theta_base"; "alpha_p"; "alpha_i"; "alpha_d"]);
verifyEqual(test_case, space.lower_bounds, ...
    [-2, -4, -2, 0, -4, -1.5, 0, 0, 0]);
verifyEqual(test_case, space.upper_bounds, ...
    [0.5, 0, 1, 2, 1, 1.5, 1.5, 1.5, 1.5]);
verifyEqual(test_case, space.dimension, 9);
end

function test_fuzzy_pid_decode_uses_mixed_parameterization(test_case)
plant = twsbr_params();
config = experiment_config("quick");
expected_gains = [0.24, 0.0004, 0.19, 9.25, 0.05, 1.01];
expected_alphas = [0.2, 0.3, 0.4];
params = decode_controller_vector("FUZZY_PID", ...
    [log10(expected_gains), expected_alphas], plant, config);

verifyEqual(test_case, [params.kp_x, params.ki_x, params.kd_x, ...
    params.kp_theta_base, params.ki_theta_base, ...
    params.kd_theta_base], expected_gains, "RelTol", 1e-14);
verifyEqual(test_case, ...
    [params.alpha_p, params.alpha_i, params.alpha_d], expected_alphas);
verifyEqual(test_case, params.theta_error_normalizer, deg2rad(12), ...
    "AbsTol", 1e-15);
verifyEqual(test_case, params.theta_rate_normalizer, deg2rad(120), ...
    "AbsTol", 1e-15);
verifyEqual(test_case, params.kp_theta_max, 4 * expected_gains(4), ...
    "AbsTol", 1e-14);
verifyEqual(test_case, params.ki_theta_max, 4 * expected_gains(5), ...
    "AbsTol", 1e-14);
verifyEqual(test_case, params.kd_theta_max, 4 * expected_gains(6), ...
    "AbsTol", 1e-14);
verifyEqual(test_case, params.sample_time, config.sample_time);
verifyEqual(test_case, params.theta_reference_limit, ...
    config.theta_reference_limit);
verifyEqual(test_case, params.position_integral_limit, ...
    config.position_integral_limit);
verifyEqual(test_case, params.theta_integral_limit, ...
    config.position_integral_limit);
verifyEqual(test_case, params.u_max, plant.u_max);
end

function test_fuzzy_pid_step_matches_cascade_and_inner_formulas(test_case)
params = manual_params();
state = struct("position_integral", 0.1, "theta_integral", 0.2);
plant_state = [0.2; 0.4; 0.1; -0.2];

[control, next_state] = fuzzy_pid_step(state, plant_state, 0.5, params);

verifyEqual(test_case, control.position_error, 0.3, "AbsTol", 1e-15);
verifyEqual(test_case, control.theta_reference_raw, 0.25, ...
    "AbsTol", 1e-15);
verifyEqual(test_case, control.theta_reference, 0.2, "AbsTol", 1e-15);
verifyEqual(test_case, control.theta_error, -0.1, "AbsTol", 1e-15);
verifyEqual(test_case, control.kp_theta, params.kp_theta_base);
verifyEqual(test_case, control.ki_theta, params.ki_theta_base);
verifyEqual(test_case, control.kd_theta, params.kd_theta_base);
verifyEqual(test_case, control.u_raw, -0.1, "AbsTol", 1e-14);
verifyEqual(test_case, control.position_integral, 0.1);
verifyEqual(test_case, control.theta_integral, 0.2);
verifyEqual(test_case, next_state.position_integral, 0.13, ...
    "AbsTol", 1e-15);
verifyEqual(test_case, next_state.theta_integral, 0.19, ...
    "AbsTol", 1e-15);
verifyEqual(test_case, control.diagnostics.adjustments, control.adjustments);
end

function test_fuzzy_pid_zero_state_has_zero_command(test_case)
state = struct("position_integral", 0.0, "theta_integral", 0.0);
[control, next_state] = fuzzy_pid_step( ...
    state, zeros(4, 1), 0.0, nominal_params());

verifyEqual(test_case, control.u_raw, 0.0, "AbsTol", 1e-14);
verifyEqual(test_case, control.theta_reference, 0.0, "AbsTol", 1e-14);
verifyEqual(test_case, next_state, state, "AbsTol", 1e-14);
end

function test_online_gains_are_clipped_to_nonnegative_limits(test_case)
params = manual_params();
params.alpha_p = 1.5;
params.alpha_i = 1.5;
params.alpha_d = 1.5;
params.kp_theta_max = 3.0;
state = struct("position_integral", 0.0, "theta_integral", 0.0);
plant_state = [0; 0; -params.theta_error_normalizer; ...
    params.theta_rate_normalizer];

[control, ~] = fuzzy_pid_step(state, plant_state, 0.0, params);

verifyEqual(test_case, control.adjustments, ...
    struct("delta_kp", 1.0, "delta_ki", -1.0, "delta_kd", 0.0), ...
    "AbsTol", 1e-14);
verifyEqual(test_case, control.kp_theta, params.kp_theta_max, ...
    "AbsTol", 1e-14);
verifyEqual(test_case, control.ki_theta, 0.0, "AbsTol", 1e-14);
verifyEqual(test_case, control.kd_theta, params.kd_theta_base, ...
    "AbsTol", 1e-14);
verifyGreaterThanOrEqual(test_case, ...
    [control.kp_theta, control.ki_theta, control.kd_theta], zeros(1, 3));
verifyLessThanOrEqual(test_case, ...
    [control.kp_theta, control.ki_theta, control.kd_theta], ...
    [params.kp_theta_max, params.ki_theta_max, params.kd_theta_max]);
end

function test_theta_integral_updates_when_unsaturated(test_case)
params = manual_params();
params.u_max = 10.0;
state = struct("position_integral", 0.0, "theta_integral", 0.0);

[control, next_state] = fuzzy_pid_step( ...
    state, [0; 0; 0.2; 0], 0.0, params);

verifyLessThanOrEqual(test_case, abs(control.u_raw), params.u_max);
verifyEqual(test_case, next_state.theta_integral, 0.02, ...
    "AbsTol", 1e-15);
end

function test_theta_integral_freezes_when_saturation_is_reinforced(test_case)
params = manual_params();
params.u_max = 0.1;
state = struct("position_integral", 0.0, "theta_integral", 0.0);

[control, next_state] = fuzzy_pid_step( ...
    state, [0; 0; 0.2; 0], 0.0, params);

verifyGreaterThan(test_case, abs(control.u_raw), params.u_max);
verifyGreaterThanOrEqual(test_case, control.theta_error * control.u_raw, 0);
verifyEqual(test_case, next_state.theta_integral, 0.0);
end

function test_theta_integral_updates_when_saturation_is_corrective(test_case)
params = manual_params();
params.u_max = 0.1;
params.theta_integral_limit = 2.0;
state = struct("position_integral", 0.0, "theta_integral", -1.0);

[control, next_state] = fuzzy_pid_step( ...
    state, [0; 0; 0.2; 0], 0.0, params);

verifyGreaterThan(test_case, abs(control.u_raw), params.u_max);
verifyLessThan(test_case, control.theta_error * control.u_raw, 0);
verifyEqual(test_case, next_state.theta_integral, -0.98, ...
    "AbsTol", 1e-14);
end

function test_both_integrals_are_clamped_in_both_directions(test_case)
params = manual_params();
params.kp_x = 0.0;
params.ki_x = 0.0;
params.kd_x = 0.0;
params.u_max = 100.0;
positive_state = struct("position_integral", 0.49, ...
    "theta_integral", 0.29);
negative_state = struct("position_integral", -0.49, ...
    "theta_integral", -0.29);

[~, positive_next] = fuzzy_pid_step( ...
    positive_state, [0; 0; 0.2; 0], 0.2, params);
[~, negative_next] = fuzzy_pid_step( ...
    negative_state, [0; 0; -0.2; 0], -0.2, params);

verifyEqual(test_case, positive_next, ...
    struct("position_integral", 0.5, "theta_integral", 0.3), ...
    "AbsTol", 1e-14);
verifyEqual(test_case, negative_next, ...
    struct("position_integral", -0.5, "theta_integral", -0.3), ...
    "AbsTol", 1e-14);
end

function test_fuzzy_pid_common_lifecycle_commits_two_state_integrals(test_case)
controller = reset_controller(create_controller( ...
    "FUZZY_PID", nominal_vector(), twsbr_params(), ...
    experiment_config("quick")));
initial_state = struct("position_integral", 0.0, "theta_integral", 0.0);

verifyEqual(test_case, controller.state, initial_state);
verifyEmpty(test_case, controller.pending_state);
verifyEmpty(test_case, controller.pending_legacy_u);

[control, controller] = controller_step(controller, 0.0, ...
    [0; 0; deg2rad(2); 0], 0.5);
verifyEqual(test_case, controller.state, initial_state);
verifyEqual(test_case, fieldnames(controller.pending_state), ...
    {'position_integral'; 'theta_integral'});
verifyEmpty(test_case, controller.pending_legacy_u);

pending_state = controller.pending_state;
controller = controller_after_actuation(controller, control.u_raw, 0.0);
verifyEqual(test_case, controller.state, pending_state);
verifyEmpty(test_case, controller.pending_state);
verifyEmpty(test_case, controller.pending_legacy_u);
end

function test_decode_rejects_invalid_fuzzy_pid_vectors(test_case)
plant = twsbr_params();
config = experiment_config("quick");
invalid_vectors = {zeros(1, 8), zeros(1, 10), ...
    [-3, zeros(1, 8)], [zeros(1, 6), -0.1, 0, 0], ...
    [zeros(1, 6), 0, 0, 1.6], ...
    [zeros(1, 8), NaN], [zeros(1, 8), Inf], ...
    [zeros(1, 8), 1i], zeros(9, 2)};

for index = 1:numel(invalid_vectors)
    verifyError(test_case, @() decode_controller_vector( ...
        "FUZZY_PID", invalid_vectors{index}, plant, config), ...
        "twsbr:controller:invalid_vector");
end
end

function test_fuzzy_pid_step_rejects_invalid_public_inputs(test_case)
params = manual_params();
valid_state = struct("position_integral", 0.0, "theta_integral", 0.0);
invalid_states = {struct(), ...
    struct("position_integral", 0.0), ...
    struct("position_integral", 0.0, "theta_integral", 0.0, "extra", 1), ...
    struct("position_integral", NaN, "theta_integral", 0.0), ...
    struct("position_integral", 0.0, "theta_integral", Inf), ...
    struct("position_integral", 0.51, "theta_integral", 0.0), ...
    struct("position_integral", 0.0, "theta_integral", 0.31), ...
    repmat(valid_state, 1, 2), "invalid"};
invalid_plant_states = {zeros(3, 1), [0; 0; NaN; 0], ...
    [0; 0; Inf; 0], [0; 0; 1i; 0], "invalid", struct()};
invalid_references = {NaN, Inf, 1i, [0, 1], "invalid", struct()};

for index = 1:numel(invalid_states)
    verifyError(test_case, @() fuzzy_pid_step( ...
        invalid_states{index}, zeros(4, 1), 0.0, params), ...
        "twsbr:fuzzy_pid:invalid_input");
end
for index = 1:numel(invalid_plant_states)
    verifyError(test_case, @() fuzzy_pid_step( ...
        valid_state, invalid_plant_states{index}, 0.0, params), ...
        "twsbr:fuzzy_pid:invalid_input");
end
for index = 1:numel(invalid_references)
    verifyError(test_case, @() fuzzy_pid_step( ...
        valid_state, zeros(4, 1), invalid_references{index}, params), ...
        "twsbr:fuzzy_pid:invalid_input");
end

invalid_params = {with_field(params, "alpha_p", 1.6), ...
    with_field(params, "theta_error_normalizer", 0.0), ...
    with_field(params, "kp_theta_max", -1.0), ...
    with_field(params, "sample_time", Inf), ...
    with_field(params, "u_max", 0.0)};
for index = 1:numel(invalid_params)
    verifyError(test_case, @() fuzzy_pid_step( ...
        valid_state, zeros(4, 1), 0.0, invalid_params{index}), ...
        "twsbr:fuzzy_pid:invalid_input");
end
end

function test_after_actuation_rejects_invalid_fuzzy_pid_pending_state(test_case)
controller = reset_controller(create_controller( ...
    "FUZZY_PID", nominal_vector(), twsbr_params(), ...
    experiment_config("quick")));
verifyError(test_case, @() controller_after_actuation( ...
    controller, 0.0, 0.0), "twsbr:controller:invalid_state");
[control, controller] = controller_step( ...
    controller, 0.0, zeros(4, 1), 0.0);

invalid = controller;
invalid.pending_state.extra = 1.0;
verify_pending_error(test_case, invalid, control.u_raw);
invalid = controller;
invalid.pending_state.position_integral = NaN;
verify_pending_error(test_case, invalid, control.u_raw);
invalid = controller;
invalid.pending_state.theta_integral = Inf;
verify_pending_error(test_case, invalid, control.u_raw);
invalid = controller;
invalid.pending_state.position_integral = ...
    invalid.params.position_integral_limit + 1.0;
verify_pending_error(test_case, invalid, control.u_raw);
invalid = controller;
invalid.pending_state.theta_integral = ...
    -invalid.params.theta_integral_limit - 1.0;
verify_pending_error(test_case, invalid, control.u_raw);
invalid = controller;
invalid.pending_legacy_u = 0.0;
verify_pending_error(test_case, invalid, control.u_raw);
end

function verify_pending_error(test_case, controller, u_raw)
verifyError(test_case, @() controller_after_actuation( ...
    controller, u_raw, u_raw), "twsbr:controller:invalid_state");
end

function params = nominal_params()
params = decode_controller_vector("FUZZY_PID", nominal_vector(), ...
    twsbr_params(), experiment_config("quick"));
end

function vector = nominal_vector()
vector = [log10([0.24, 0.0004, 0.19, 9.25, 0.05, 1.01]), ...
    0.2, 0.2, 0.2];
end

function params = manual_params()
params = struct( ...
    "kp_x", 1.0, ...
    "ki_x", 0.5, ...
    "kd_x", 0.25, ...
    "kp_theta_base", 2.0, ...
    "ki_theta_base", 1.0, ...
    "kd_theta_base", 0.5, ...
    "alpha_p", 0.0, ...
    "alpha_i", 0.0, ...
    "alpha_d", 0.0, ...
    "theta_error_normalizer", 1.0, ...
    "theta_rate_normalizer", 1.0, ...
    "kp_theta_max", 8.0, ...
    "ki_theta_max", 4.0, ...
    "kd_theta_max", 2.0, ...
    "sample_time", 0.1, ...
    "theta_reference_limit", 0.2, ...
    "position_integral_limit", 0.5, ...
    "theta_integral_limit", 0.3, ...
    "u_max", 1.0);
end

function value = with_field(value, field_name, field_value)
value.(field_name) = field_value;
end
