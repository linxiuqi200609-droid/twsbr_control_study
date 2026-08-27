function tests = test_all_controller_factory
%TEST_ALL_CONTROLLER_FACTORY Verifies the unified five-controller contract.
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

function test_config_and_starter_vectors_cover_all_controllers(test_case)
[names, vectors] = controller_cases();
config = experiment_config("quick");

verifyEqual(test_case, config.controller_names, names);
verifyEqual(test_case, numel(vectors), numel(names));
for index = 1:numel(names)
    space = controller_parameter_space(names(index));
    verifySize(test_case, vectors{index}, [1, space.dimension]);
    verifyTrue(test_case, all(isfinite(vectors{index})));
end
end

function test_all_controllers_share_the_lifecycle_contract(test_case)
[names, vectors] = controller_cases();
plant = twsbr_params();
config = experiment_config("quick");

for index = 1:numel(names)
    controller = create_controller(names(index), vectors{index}, plant, config);
    controller = reset_controller(controller);
    [control, controller] = controller_step(controller, 0.0, ...
        zeros(4, 1), 0.0);
    applied_u = min(max(control.u_raw, -plant.u_max), plant.u_max);
    controller = controller_after_actuation(controller, control.u_raw, applied_u);

    verifyEqual(test_case, controller.name, names(index));
    verifyTrue(test_case, isscalar(control.u_raw) && isfinite(control.u_raw));
    verifyTrue(test_case, isscalar(control.theta_reference) && ...
        isfinite(control.theta_reference));
    verifyTrue(test_case, isscalar(control.diagnostics) && ...
        isstruct(control.diagnostics));
    verify_finite_struct(test_case, controller.state);
end
end

function test_all_controllers_share_the_simulation_result_contract(test_case)
[names, vectors] = controller_cases();
plant = twsbr_params();
config = experiment_config("quick");
scenario = short_t1_scenario();
expected_fields = { ...
    'controller_name'; 'scenario_name'; 'seed'; 'time'; 'state'; ...
    'position_reference'; 'theta_reference'; 'u_raw'; 'u'; ...
    'saturated'; 'force_disturbance'; 'torque_disturbance'; ...
    'diagnostics'; 'success'; 'failure_reason'; 'survived_time'; ...
    'runtime_seconds'};

for index = 1:numel(names)
    result = simulate_control_system(names(index), vectors{index}, ...
        plant, config, scenario, 42);
    sample_count = numel(result.time);

    verifyEqual(test_case, fieldnames(result), expected_fields);
    verifyEqual(test_case, result.controller_name, names(index));
    verifyGreaterThan(test_case, sample_count, 0);
    verifySize(test_case, result.state, [sample_count, 4]);
    verify_finite_nonempty(test_case, result.time);
    verify_finite_nonempty(test_case, result.state);
    verify_finite_nonempty(test_case, result.u_raw);
    verify_finite_nonempty(test_case, result.u);
    verify_finite_nonempty(test_case, result.theta_reference);
    verifyEqual(test_case, numel(result.u_raw), sample_count);
    verifyEqual(test_case, numel(result.u), sample_count);
    verifyEqual(test_case, numel(result.theta_reference), sample_count);
    verifyEqual(test_case, result.u, ...
        min(max(result.u_raw, -plant.u_max), plant.u_max));
    verifyTrue(test_case, isstruct(result.diagnostics));
    verifyEqual(test_case, numel(result.diagnostics), sample_count);
    for sample_index = 1:sample_count
        verifyTrue(test_case, isscalar(result.diagnostics(sample_index)));
    end
end
end

function test_same_seed_is_deterministic_for_all_controllers(test_case)
[names, vectors] = controller_cases();
plant = twsbr_params();
config = experiment_config("quick");
scenario = short_t1_scenario();

for index = 1:numel(names)
    first = simulate_control_system(names(index), vectors{index}, ...
        plant, config, scenario, 42);
    second = simulate_control_system(names(index), vectors{index}, ...
        plant, config, scenario, 42);
    first = rmfield(first, "runtime_seconds");
    second = rmfield(second, "runtime_seconds");
    verifyEqual(test_case, second, first);
end
end

function test_cascade_nonfinite_output_uses_unified_controller_error(test_case)
plant = twsbr_params();
config = experiment_config("quick");
[~, vectors] = controller_cases();
controller = create_controller("CASCADE_PID", vectors{2}, plant, config);
controller = reset_controller(controller);
controller.params.kp_x = realmax;

verifyError(test_case, @() controller_step(controller, 0.0, ...
    zeros(4, 1), realmax), "twsbr:controller:nonfinite_output");
end

function test_simulator_returns_truncated_failure_for_unified_nonfinite_output(test_case)
plant = twsbr_params();
config = experiment_config("quick");
scenario = short_t1_scenario();
scenario.x_reference = @(~) realmax;
[~, vectors] = controller_cases();
cascade_vector = vectors{2};
cascade_vector(1) = 0.5;

result = simulate_control_system("CASCADE_PID", cascade_vector, ...
    plant, config, scenario, 42);

verifyFalse(test_case, result.success);
verifyEqual(test_case, result.failure_reason, "nonfinite_control");
verifyEmpty(test_case, result.time);
verifySize(test_case, result.state, [0, 4]);
verifyEmpty(test_case, result.u_raw);
verifyEmpty(test_case, result.u);
verifyEmpty(test_case, result.theta_reference);
verifyEmpty(test_case, result.diagnostics);
verifyEqual(test_case, result.survived_time, 0.0);
end

function scenario = short_t1_scenario()
scenarios = training_scenarios(3.2);
scenario = scenarios.T1_initial_tilt_5deg;
scenario.duration = 0.1;
end

function [names, vectors] = controller_cases()
names = ["ATTITUDE_PID"; "CASCADE_PID"; "FUZZY_PID"; "LQR"; "LQI"];
vectors = { ...
    log10([1.9, 0.2, 0.18]), ...
    log10([0.24100028146267993, 0.0003962067755988572, ...
        0.1930824033173246, 9.254929149177556, 1.0113335430173094]), ...
    [log10([0.24, 0.0004, 0.19, 9.25, 0.05, 1.01]), 0.2, 0.2, 0.2], ...
    log10([10, 1, 200, 10, 0.1]), ...
    log10([10, 1, 200, 10, 100, 0.1])};
end

function verify_finite_nonempty(test_case, value)
verifyFalse(test_case, isempty(value));
verifyTrue(test_case, all(isfinite(value), "all"));
end

function verify_finite_struct(test_case, value)
verifyTrue(test_case, isscalar(value) && isstruct(value));
fields = fieldnames(value);
for index = 1:numel(fields)
    field_value = value.(fields{index});
    verifyTrue(test_case, isnumeric(field_value));
    verifyTrue(test_case, all(isfinite(field_value), "all"));
end
end
