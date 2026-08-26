function tests = test_legacy_controller_adapters
%TEST_LEGACY_CONTROLLER_ADAPTERS Unit tests for the common PID lifecycle.
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

function test_attitude_adapter_uses_existing_step_and_ignores_position(test_case)
plant = twsbr_params();
config = experiment_config("quick");
vector = log10([1.9, 0.2, 0.18]);
controller = create_controller("ATTITUDE_PID", vector, plant, config);
controller = reset_controller(controller);
[control, controller] = controller_step(controller, 0.0, ...
    [0; 0; 0.1; 0.2], 5.0);
[legacy, legacy_state] = attitude_pid_step( ...
    struct("integral_error", 0.0), 0.1, 0.2, 0.0, controller.params);

verifyEqual(test_case, control.u_raw, legacy.u_raw, "AbsTol", 1e-15);
verifyEqual(test_case, control.theta_reference, 0.0);
verifyEqual(test_case, control.diagnostics, legacy);
verifyEqual(test_case, controller.state, struct("integral_error", 0.0));
verifyEqual(test_case, controller.pending_state, legacy_state);
verifyEqual(test_case, controller.pending_legacy_u, legacy.u, "AbsTol", 1e-15);

controller = controller_after_actuation(controller, control.u_raw, legacy.u);
verifyEqual(test_case, controller.state, legacy_state);
verifyEmpty(test_case, controller.pending_state);
verifyEmpty(test_case, controller.pending_legacy_u);
end

function test_cascade_adapter_uses_existing_step(test_case)
plant = twsbr_params();
config = experiment_config("quick");
vector = log10([0.24100028146267993, 0.0003962067755988572, ...
    0.1930824033173246, 9.254929149177556, 1.0113335430173094]);
controller = reset_controller(create_controller( ...
    "CASCADE_PID", vector, plant, config));
[control, controller] = controller_step(controller, 0.0, zeros(4, 1), 0.5);
[legacy, legacy_state] = cascade_pid_step( ...
    struct("position_integral", 0.0), zeros(4, 1), 0.5, controller.params);

verifyEqual(test_case, control.u_raw, legacy.u_raw, "AbsTol", 1e-15);
verifyEqual(test_case, control.theta_reference, legacy.theta_reference);
verifyEqual(test_case, control.diagnostics, legacy);
verifyEqual(test_case, controller.pending_state, legacy_state);
verifyEqual(test_case, controller.pending_legacy_u, legacy.u, "AbsTol", 1e-15);

controller = controller_after_actuation(controller, control.u_raw, legacy.u);
verifyEqual(test_case, controller.state, legacy_state);
end

function test_reset_uses_exact_legacy_state_shapes(test_case)
plant = twsbr_params();
config = experiment_config("quick");
attitude = reset_controller(create_controller( ...
    "ATTITUDE_PID", log10([1.9, 0.2, 0.18]), plant, config));
cascade = reset_controller(create_controller("CASCADE_PID", cascade_vector(), plant, config));

verifyEqual(test_case, attitude.state, struct("integral_error", 0.0));
verifyEqual(test_case, cascade.state, struct("position_integral", 0.0));
verifyEmpty(test_case, attitude.pending_state);
verifyEmpty(test_case, cascade.pending_state);
verifyEmpty(test_case, attitude.pending_legacy_u);
verifyEmpty(test_case, cascade.pending_legacy_u);
end

function test_controller_step_rejects_noncolumn_or_nonfinite_state(test_case)
controller = make_attitude_controller();

verifyError(test_case, @() controller_step(controller, 0.0, zeros(1, 4), 0.0), ...
    "twsbr:controller:invalid_state");
verifyError(test_case, @() controller_step(controller, 0.0, [0; 0; NaN; 0], 0.0), ...
    "twsbr:controller:invalid_state");
end

function test_controller_step_rejects_unsupported_name(test_case)
controller = make_attitude_controller();
controller.name = "UNKNOWN";

verifyError(test_case, @() controller_step(controller, 0.0, zeros(4, 1), 0.0), ...
    "twsbr:controller:unsupported_name");
end

function test_after_actuation_rejects_saturation_mismatch_without_committing(test_case)
controller = make_attitude_controller();
[control, controller] = controller_step(controller, 0.0, [0; 0; 1; 0], 0.0);
state_before = controller.state;

verifyGreaterThan(test_case, abs(control.u_raw), controller.params.u_max);
verifyError(test_case, @() controller_after_actuation( ...
    controller, control.u_raw, 0.0), ...
    "twsbr:controller:legacy_saturation_mismatch");
verifyEqual(test_case, controller.state, state_before);
end

function test_after_actuation_rejects_nonfinite_scalar_inputs(test_case)
controller = make_attitude_controller();
[control, controller] = controller_step(controller, 0.0, [0; 0; 0.1; 0], 0.0);

verifyError(test_case, @() controller_after_actuation( ...
    controller, NaN, control.diagnostics.u), "twsbr:controller:invalid_input");
verifyError(test_case, @() controller_after_actuation( ...
    controller, control.u_raw, Inf), "twsbr:controller:invalid_input");
end

function controller = make_attitude_controller()
controller = reset_controller(create_controller("ATTITUDE_PID", ...
    log10([1.9, 0.2, 0.18]), twsbr_params(), experiment_config("quick")));
end

function vector = cascade_vector()
vector = log10([0.24100028146267993, 0.0003962067755988572, ...
    0.1930824033173246, 9.254929149177556, 1.0113335430173094]);
end
