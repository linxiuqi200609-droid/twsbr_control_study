function tests = test_lqi_controller
%TEST_LQI_CONTROLLER Unit tests for the discrete LQI controller.
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

function test_lqi_parameter_space_is_exact(test_case)
space = controller_parameter_space("LQI");

verifyEqual(test_case, space.name, "LQI");
verifyEqual(test_case, space.parameter_names, ...
    ["q_x"; "q_x_dot"; "q_theta"; "q_theta_dot"; "q_integral"; "r"]);
verifyEqual(test_case, space.lower_bounds, [-2, -2, -2, -2, -2, -3]);
verifyEqual(test_case, space.upper_bounds, [4, 4, 5, 4, 5, 2]);
verifyEqual(test_case, space.dimension, 6);
end

function test_decoded_lqi_is_finite_discrete_and_stable(test_case)
plant = twsbr_params();
config = experiment_config("quick");
params = decode_controller_vector("LQI", nominal_vector(), plant, config);
[ad, bd] = twsbr_discrete_model(plant, config.sample_time);
[a_aug, b_aug] = twsbr_augmented_lqi_model(ad, bd, config.sample_time);
expected_q = [10, 1, 200, 10, 100];
expected_r = 0.1;
[expected_gain, ~, expected_poles] = dlqr( ...
    a_aug, b_aug, diag(expected_q), expected_r);
gain_aug = [params.state_gain, params.integral_gain];

verifyEqual(test_case, params.q_aug_diag, expected_q, "AbsTol", 1e-12);
verifyEqual(test_case, params.r_value, expected_r, "AbsTol", 1e-14);
verifyEqual(test_case, size(params.state_gain), [1, 4]);
verifyTrue(test_case, all(isfinite(params.state_gain), "all"));
verifyTrue(test_case, isnumeric(params.integral_gain) && ...
    isreal(params.integral_gain) && isscalar(params.integral_gain) && ...
    isfinite(params.integral_gain));
verifyEqual(test_case, gain_aug, expected_gain, "AbsTol", 1e-12);
verifyEqual(test_case, sort(params.closed_loop_eigenvalues), ...
    sort(expected_poles), "AbsTol", 1e-12);
verifyEqual(test_case, sort(params.closed_loop_eigenvalues), ...
    sort(eig(a_aug - b_aug * gain_aug)), "AbsTol", 1e-12);
verifyTrue(test_case, all(abs(params.closed_loop_eigenvalues) < 1));
verifyEqual(test_case, params.sample_time, config.sample_time, ...
    "AbsTol", 1e-15);
verifyEqual(test_case, params.position_integral_limit, ...
    config.position_integral_limit, "AbsTol", 1e-9);
end

function test_lqi_integrates_position_error(test_case)
params = nominal_params();
controller_state = struct("position_integral", 0.2);

[control, next_state] = lqi_step(controller_state, ...
    [0.1; 0; 0; 0], 0.4, params);

verifyEqual(test_case, control.position_error, -0.3, "AbsTol", 1e-15);
verifyEqual(test_case, control.position_integral, 0.2, "AbsTol", 1e-15);
verifyEqual(test_case, next_state, struct("position_integral", 0.197), ...
    "AbsTol", 1e-15);
verifyEqual(test_case, control.diagnostics, ...
    struct("position_integral", 0.2), "AbsTol", 1e-15);
end

function test_lqi_clamps_pending_integral_in_both_directions(test_case)
params = command_test_params();
params.position_integral_limit = 0.5;

[~, positive_state] = lqi_step( ...
    struct("position_integral", 0.499), [1; 0; 0; 0], 0.0, params);
[~, negative_state] = lqi_step( ...
    struct("position_integral", -0.499), [-1; 0; 0; 0], 0.0, params);

verifyEqual(test_case, positive_state.position_integral, 0.5, ...
    "AbsTol", 1e-15);
verifyEqual(test_case, negative_state.position_integral, -0.5, ...
    "AbsTol", 1e-15);
end

function test_lqi_command_uses_current_integral_and_full_state(test_case)
params = command_test_params();
plant_state = [0.1; 0.2; 0.3; 0.4];
controller_state = struct("position_integral", 0.2);

[control, next_state] = lqi_step( ...
    controller_state, plant_state, 0.6, params);

verifyEqual(test_case, control.u_raw, -4.0, "AbsTol", 1e-15);
verifyEqual(test_case, control.theta_reference, 0.0);
verifyEqual(test_case, next_state.position_integral, 0.195, ...
    "AbsTol", 1e-15);
verifyTrue(test_case, isnumeric(control.u_raw) && isreal(control.u_raw) && ...
    isscalar(control.u_raw) && isfinite(control.u_raw));
end

function test_lqi_common_lifecycle_commits_pending_integral(test_case)
controller = reset_controller(create_controller("LQI", nominal_vector(), ...
    twsbr_params(), experiment_config("quick")));

verifyEqual(test_case, controller.state, struct("position_integral", 0.0));
verifyEmpty(test_case, controller.pending_state);
verifyEmpty(test_case, controller.pending_legacy_u);

[control, controller] = controller_step(controller, 0.0, ...
    [0.1; 0; 0; 0], 0.4);
verifyEqual(test_case, controller.state, struct("position_integral", 0.0));
verifyEqual(test_case, controller.pending_state, ...
    struct("position_integral", -0.003), "AbsTol", 1e-15);
verifyEmpty(test_case, controller.pending_legacy_u);

controller = controller_after_actuation(controller, control.u_raw, 0.25);
verifyEqual(test_case, controller.state, ...
    struct("position_integral", -0.003), "AbsTol", 1e-15);
verifyEmpty(test_case, controller.pending_state);
verifyEmpty(test_case, controller.pending_legacy_u);
end

function test_lqi_after_actuation_rejects_invalid_pending_state(test_case)
controller = make_lqi_controller();
verifyError(test_case, @() controller_after_actuation( ...
    controller, 0.0, 0.0), "twsbr:controller:invalid_state");

[control, controller] = controller_step(controller, 0.0, zeros(4, 1), 0.0);
controller.pending_state = struct("position_integral", NaN);
verifyError(test_case, @() controller_after_actuation( ...
    controller, control.u_raw, control.u_raw), ...
    "twsbr:controller:invalid_state");

[control, controller] = controller_step(make_lqi_controller(), ...
    0.0, zeros(4, 1), 0.0);
controller.pending_state.extra = 1.0;
verifyError(test_case, @() controller_after_actuation( ...
    controller, control.u_raw, control.u_raw), ...
    "twsbr:controller:invalid_state");
end

function test_lqi_after_actuation_rejects_legacy_pending_value(test_case)
controller = make_lqi_controller();
[control, controller] = controller_step(controller, 0.0, zeros(4, 1), 0.0);
controller.pending_legacy_u = 0.25;

verifyError(test_case, @() controller_after_actuation( ...
    controller, control.u_raw, control.u_raw), ...
    "twsbr:controller:invalid_state");
end

function test_lqi_step_rejects_current_integral_outside_limit(test_case)
params = command_test_params();
params.position_integral_limit = 0.5;
out_of_range_states = {struct("position_integral", 0.5001), ...
    struct("position_integral", -0.5001)};

for index = 1:numel(out_of_range_states)
    verifyError(test_case, @() lqi_step(out_of_range_states{index}, ...
        zeros(4, 1), 0.0, params), "twsbr:lqi:invalid_input");
end
end

function test_lqi_after_actuation_rejects_pending_integral_outside_limit( ...
        test_case)
controller = make_lqi_controller();
[control, controller] = controller_step(controller, 0.0, zeros(4, 1), 0.0);
integral_limit = controller.params.position_integral_limit;

controller.pending_state.position_integral = integral_limit + 1.0;
verifyError(test_case, @() controller_after_actuation( ...
    controller, control.u_raw, control.u_raw), ...
    "twsbr:controller:invalid_state");

controller.pending_state.position_integral = -integral_limit - 1.0;
verifyError(test_case, @() controller_after_actuation( ...
    controller, control.u_raw, control.u_raw), ...
    "twsbr:controller:invalid_state");
end

function test_decode_rejects_invalid_lqi_vectors(test_case)
plant = twsbr_params();
config = experiment_config("quick");
invalid_vectors = {zeros(1, 5), zeros(1, 7), [-3, 0, 0, 0, 0, 0], ...
    [0, 0, 0, 0, 6, 0], [0, 0, 0, 0, 0, 3], ...
    [0, 0, 0, 0, 0, NaN], [0, 0, 0, 0, 0, Inf], ...
    [0, 0, 0, 0, 0, 1i], zeros(6, 2)};

for index = 1:numel(invalid_vectors)
    verifyError(test_case, @() decode_controller_vector( ...
        "LQI", invalid_vectors{index}, plant, config), ...
        "twsbr:controller:invalid_vector");
end
end

function test_lqi_step_rejects_invalid_public_inputs(test_case)
params = command_test_params();
invalid_controller_states = {struct(), ...
    struct("position_integral", NaN), ...
    struct("position_integral", Inf), ...
    struct("position_integral", 1i), ...
    struct("position_integral", 0.0, "extra", 1.0), ...
    repmat(struct("position_integral", 0.0), 1, 2), 0.0, "invalid"};
invalid_plant_states = {zeros(3, 1), [0; 0; NaN; 0], ...
    [0; 0; Inf; 0], [0; 0; 1i; 0], "invalid", struct()};
invalid_references = {NaN, Inf, 1i, [0, 1], "invalid", struct()};

for index = 1:numel(invalid_controller_states)
    verifyError(test_case, @() lqi_step(invalid_controller_states{index}, ...
        zeros(4, 1), 0.0, params), "twsbr:lqi:invalid_input");
end
for index = 1:numel(invalid_plant_states)
    verifyError(test_case, @() lqi_step( ...
        struct("position_integral", 0.0), invalid_plant_states{index}, ...
        0.0, params), "twsbr:lqi:invalid_input");
end
for index = 1:numel(invalid_references)
    verifyError(test_case, @() lqi_step( ...
        struct("position_integral", 0.0), zeros(4, 1), ...
        invalid_references{index}, params), "twsbr:lqi:invalid_input");
end

invalid_params = {with_field(params, "state_gain", [NaN, 0, 0, 0]), ...
    with_field(params, "integral_gain", Inf), ...
    with_field(params, "sample_time", 0), ...
    with_field(params, "position_integral_limit", Inf)};
for index = 1:numel(invalid_params)
    verifyError(test_case, @() lqi_step( ...
        struct("position_integral", 0.0), zeros(4, 1), 0.0, ...
        invalid_params{index}), "twsbr:lqi:invalid_input");
end
end

function test_decode_rejects_nonstabilizing_candidate_with_stable_id( ...
        test_case)
plant = twsbr_params(struct("motor_force_gain", realmin));

verifyError(test_case, @() decode_controller_vector( ...
    "LQI", nominal_vector(), plant, experiment_config("quick")), ...
    "twsbr:lqi:nonstabilizing_solution");
end

function test_decode_preserves_unexpected_dlqr_error_without_path_pollution( ...
        test_case)
original_path = path;
shadow_directory = tempname;
mkdir(shadow_directory);
warning_state = warning("query", "MATLAB:dispatcher:nameConflict");
cleanup_guard = onCleanup(@() restore_dlqr_shadow( ...
    original_path, shadow_directory, warning_state));
write_dlqr_shadow(shadow_directory);
warning("off", "MATLAB:dispatcher:nameConflict");
addpath(shadow_directory, "-begin");
clear dlqr
rehash

caught_exception = [];
try
    decode_controller_vector("LQI", nominal_vector(), ...
        twsbr_params(), experiment_config("quick"));
catch cause
    caught_exception = cause;
end
delete(cleanup_guard);

verifyEqual(test_case, path, original_path);
verifyFalse(test_case, isfolder(shadow_directory));
assertFalse(test_case, isempty(caught_exception));
verifyEqual(test_case, string(caught_exception.identifier), ...
    "twsbr:test:unexpected_dlqr_failure");
end

function controller = make_lqi_controller()
controller = reset_controller(create_controller("LQI", nominal_vector(), ...
    twsbr_params(), experiment_config("quick")));
end

function params = nominal_params()
params = decode_controller_vector("LQI", nominal_vector(), ...
    twsbr_params(), experiment_config("quick"));
end

function vector = nominal_vector()
vector = log10([10, 1, 200, 10, 100, 0.1]);
end

function params = command_test_params()
params = struct( ...
    "state_gain", [1, 2, 3, 4], ...
    "integral_gain", 5, ...
    "sample_time", 0.01, ...
    "position_integral_limit", 1e6);
end

function value = with_field(value, field_name, field_value)
value.(field_name) = field_value;
end

function write_dlqr_shadow(shadow_directory)
shadow_file = fullfile(shadow_directory, "dlqr.m");
file_id = fopen(shadow_file, "w");
if file_id < 0
    error("twsbr:test:tempfile_failed", ...
        "Unable to create the temporary dlqr shadow.");
end
file_guard = onCleanup(@() fclose(file_id));
contents = sprintf([ ...
    'function varargout = dlqr(varargin)\n', ...
    'error(''twsbr:test:unexpected_dlqr_failure'', ', ...
    '''Injected unexpected dlqr failure.'');\n', ...
    'end\n']);
fprintf(file_id, "%s", contents);
delete(file_guard);
end

function restore_dlqr_shadow(original_path, shadow_directory, warning_state)
path(original_path);
clear dlqr
rehash
if isfolder(shadow_directory)
    rmdir(shadow_directory, "s");
end
warning(warning_state.state, warning_state.identifier);
end
