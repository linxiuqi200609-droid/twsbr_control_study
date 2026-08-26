function tests = test_training_scenarios
%TEST_TRAINING_SCENARIOS Tests for the unified training scenario factory.
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

function test_training_scenarios_are_exact_and_marked_train(test_case)
scenarios = training_scenarios(8.0);

verifyEqual(test_case, fieldnames(scenarios), ...
    {'T1_initial_tilt_5deg'; 'T2_position_step_0p5m'; ...
    'T3_impulse_disturbance'});
verifyEqual(test_case, scenarios.T1_initial_tilt_5deg.initial_state, ...
    [0; 0; deg2rad(5); 0], "AbsTol", 1e-15);
verifyEqual(test_case, scenarios.T2_position_step_0p5m.x_reference(0.9), 0);
verifyEqual(test_case, scenarios.T2_position_step_0p5m.x_reference(1.0), 0.5);
verifyEqual(test_case, scenarios.T3_impulse_disturbance.force_disturbance(3.1), 5);
verifyEqual(test_case, scenarios.T3_impulse_disturbance.force_disturbance(3.2), 0);
verifyTrue(test_case, all(structfun(@(s) s.split == "train", scenarios)));
end

function test_training_scenarios_preserve_serializable_metadata(test_case)
scenarios = training_scenarios(8.0);
required_fields = ["name"; "split"; "duration"; "initial_state"; ...
    "x_reference"; "force_disturbance"; "torque_disturbance"; ...
    "reference_start"; "disturbance_end"; "measurement_noise_std"; ...
    "reference_amplitude"; "force_amplitude"; "force_start"; ...
    "force_duration"; "torque_amplitude"; "torque_start"; ...
    "torque_duration"; "constant_force"];

scenario_names = fieldnames(scenarios);
for index = 1:numel(scenario_names)
    scenario = scenarios.(scenario_names{index});
    verifyTrue(test_case, all(isfield(scenario, required_fields)));
    verifyEqual(test_case, scenario.measurement_noise_std, zeros(4, 1));
end

verifyEqual(test_case, scenarios.T2_position_step_0p5m.reference_amplitude, 0.5);
verifyEqual(test_case, scenarios.T3_impulse_disturbance.force_duration, 0.2);
verifyEqual(test_case, scenarios.T3_impulse_disturbance.force_disturbance( ...
    [2.999, 3.0, 3.199, 3.2]), [0, 5, 5, 0]);
end

function test_training_scenarios_reject_duration_that_cannot_contain_events(test_case)
verifyError(test_case, @() training_scenarios(3.199), ...
    "twsbr:scenario:invalid_duration");
end
