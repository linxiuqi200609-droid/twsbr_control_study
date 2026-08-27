function tests = test_lqr_controller
%TEST_LQR_CONTROLLER Unit tests for the discrete LQR controller.
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

function test_lqr_parameter_space_is_exact(test_case)
space = controller_parameter_space("LQR");

verifyEqual(test_case, space.name, "LQR");
verifyEqual(test_case, space.parameter_names, ...
    ["q_x"; "q_x_dot"; "q_theta"; "q_theta_dot"; "r"]);
verifyEqual(test_case, space.lower_bounds, [-2, -2, -2, -2, -3]);
verifyEqual(test_case, space.upper_bounds, [4, 4, 5, 4, 2]);
verifyEqual(test_case, space.dimension, 5);
end

function test_decoded_lqr_is_finite_discrete_and_stable(test_case)
plant = twsbr_params();
config = experiment_config("quick");
params = decode_controller_vector("LQR", nominal_vector(), plant, config);
[ad, bd] = twsbr_discrete_model(plant, config.sample_time);

verifyEqual(test_case, params.q_diag, [10, 1, 200, 10], ...
    "AbsTol", 1e-12);
verifyEqual(test_case, params.r_value, 0.1, "AbsTol", 1e-14);
verifyEqual(test_case, size(params.gain), [1, 4]);
verifyTrue(test_case, all(isfinite(params.gain), "all"));
verifyTrue(test_case, all(isfinite(params.closed_loop_eigenvalues), "all"));
verifyEqual(test_case, sort(params.closed_loop_eigenvalues), ...
    sort(eig(ad - bd * params.gain)), "AbsTol", 1e-12);
verifyTrue(test_case, all(abs(params.closed_loop_eigenvalues) < 1));
verifyEqual(test_case, params.sample_time, 0.01, "AbsTol", 1e-15);
end

function test_lqr_uses_position_reference_error_state(test_case)
params = nominal_params();
plant_state = [0.2; -0.1; 0.05; 0.02];
expected_error = [0; -0.1; 0.05; 0.02];

control = lqr_step(plant_state, 0.2, params);

verifyEqual(test_case, control.state_error, expected_error, ...
    "AbsTol", 1e-15);
verifyEqual(test_case, control.u_raw, -params.gain * expected_error, ...
    "AbsTol", 1e-14);
verifyEqual(test_case, control.theta_reference, 0.0);
verifyEqual(test_case, control.diagnostics.state_error_norm, ...
    norm(expected_error), "AbsTol", 1e-15);
end

function test_lqr_command_has_correct_attitude_sign(test_case)
params = nominal_params();
positive_tilt = lqr_step([0; 0; 0.05; 0], 0.0, params);
negative_tilt = lqr_step([0; 0; -0.05; 0], 0.0, params);

verifyGreaterThan(test_case, positive_tilt.u_raw, 0);
verifyLessThan(test_case, negative_tilt.u_raw, 0);
verifyEqual(test_case, positive_tilt.u_raw, -negative_tilt.u_raw, ...
    "AbsTol", 1e-14);
end

function test_lqr_zero_error_has_zero_command(test_case)
control = lqr_step([0.4; 0; 0; 0], 0.4, nominal_params());

verifyEqual(test_case, control.u_raw, 0.0, "AbsTol", 1e-14);
verifyTrue(test_case, isnumeric(control.u_raw) && isreal(control.u_raw) && ...
    isscalar(control.u_raw) && isfinite(control.u_raw));
end

function test_lqr_common_lifecycle_commits_no_op_state(test_case)
plant = twsbr_params();
config = experiment_config("quick");
controller = create_controller("LQR", nominal_vector(), plant, config);
controller = reset_controller(controller);

verify_true_empty_scalar_struct(test_case, controller.state);
verifyEmpty(test_case, controller.pending_state);
verifyEmpty(test_case, controller.pending_legacy_u);

[control, controller] = controller_step(controller, 0.0, ...
    [0; 0; 0.05; 0], 0.0);
verifyTrue(test_case, isfinite(control.u_raw) && isscalar(control.u_raw));
verify_true_empty_scalar_struct(test_case, controller.pending_state);
verifyEmpty(test_case, controller.pending_legacy_u);

controller = controller_after_actuation(controller, control.u_raw, 0.5);
verify_true_empty_scalar_struct(test_case, controller.state);
verifyEmpty(test_case, controller.pending_state);
verifyEmpty(test_case, controller.pending_legacy_u);
end

function test_lqr_after_actuation_rejects_invalid_inputs_and_missing_step( ...
        test_case)
controller = reset_controller(create_controller("LQR", nominal_vector(), ...
    twsbr_params(), experiment_config("quick")));

verifyError(test_case, @() controller_after_actuation( ...
    controller, 0.0, 0.0), "twsbr:controller:invalid_state");
[control, controller] = controller_step(controller, 0.0, zeros(4, 1), 0.0);
verifyError(test_case, @() controller_after_actuation( ...
    controller, NaN, 0.0), "twsbr:controller:invalid_input");
verifyError(test_case, @() controller_after_actuation( ...
    controller, control.u_raw, Inf), "twsbr:controller:invalid_input");
end

function test_decode_rejects_invalid_lqr_vectors(test_case)
plant = twsbr_params();
config = experiment_config("quick");
invalid_vectors = {zeros(1, 4), zeros(1, 6), [-3, 0, 0, 0, 0], ...
    [0, 0, 6, 0, 0], [0, 0, 0, 0, NaN], ...
    [0, 0, 0, 0, Inf], [0, 0, 0, 0, 1i], zeros(5, 2)};

for index = 1:numel(invalid_vectors)
    verifyError(test_case, @() decode_controller_vector( ...
        "LQR", invalid_vectors{index}, plant, config), ...
        "twsbr:controller:invalid_vector");
end
end

function test_decode_rejects_nonstabilizing_candidate_with_stable_id( ...
        test_case)
plant = twsbr_params(struct("motor_force_gain", realmin));

verifyError(test_case, @() decode_controller_vector( ...
    "LQR", nominal_vector(), plant, experiment_config("quick")), ...
    "twsbr:lqr:nonstabilizing_solution");
end

function test_lqr_step_rejects_invalid_public_inputs(test_case)
params = nominal_params();
invalid_states = {zeros(3, 1), [0; 0; NaN; 0], ...
    [0; 0; Inf; 0], [0; 0; 1i; 0], "invalid", struct()};
invalid_references = {NaN, Inf, 1i, [0, 1], "invalid", struct()};

for index = 1:numel(invalid_states)
    verifyError(test_case, @() lqr_step( ...
        invalid_states{index}, 0.0, params), "twsbr:lqr:invalid_input");
end
for index = 1:numel(invalid_references)
    verifyError(test_case, @() lqr_step( ...
        zeros(4, 1), invalid_references{index}, params), ...
        "twsbr:lqr:invalid_input");
end

invalid_params = params;
invalid_params.gain(1) = NaN;
verifyError(test_case, @() lqr_step(zeros(4, 1), 0.0, invalid_params), ...
    "twsbr:lqr:invalid_input");
end

function test_decode_preserves_unexpected_dlqr_error(test_case)
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
    decode_controller_vector("LQR", nominal_vector(), ...
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

function test_lqr_after_actuation_rejects_legacy_pending_value(test_case)
controller = reset_controller(create_controller("LQR", nominal_vector(), ...
    twsbr_params(), experiment_config("quick")));
[control, controller] = controller_step(controller, 0.0, zeros(4, 1), 0.0);
controller.pending_legacy_u = 0.25;

verifyError(test_case, @() controller_after_actuation( ...
    controller, control.u_raw, control.u_raw), ...
    "twsbr:controller:invalid_state");
end

function params = nominal_params()
params = decode_controller_vector("LQR", nominal_vector(), ...
    twsbr_params(), experiment_config("quick"));
end

function vector = nominal_vector()
vector = log10([10, 1, 200, 10, 0.1]);
end

function verify_true_empty_scalar_struct(test_case, value)
verifyTrue(test_case, isstruct(value) && isscalar(value));
verifyEmpty(test_case, fieldnames(value));
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
