function tests = test_heldout_scenarios
%TEST_HELDOUT_SCENARIOS Tests for the deterministic held-out scenarios.
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

function test_names_schema_split_and_training_disjointness_are_exact(test_case)
heldout = heldout_scenarios(8.0);
training = training_scenarios(8.0);

expected_names = { ...
    'S1_initial_tilt_3deg'; ...
    'S1_initial_tilt_6deg'; ...
    'S1_initial_tilt_9deg'; ...
    'S1_initial_tilt_12deg'; ...
    'S2_position_step_0p25m'; ...
    'S2_position_step_0p75m'; ...
    'S2_position_step_1p0m'; ...
    'S2_saturation_stress'; ...
    'S3_positive_impulse'; ...
    'S3_negative_impulse'; ...
    'S3_torque_impulse'; ...
    'S5_constant_bias'};
verifyEqual(test_case, fieldnames(heldout), expected_names);
verifyEmpty(test_case, intersect(fieldnames(heldout), fieldnames(training)));

training_schema = fieldnames(training.T1_initial_tilt_5deg);
for index = 1:numel(expected_names)
    scenario = heldout.(expected_names{index});
    verifyEqual(test_case, fieldnames(scenario), training_schema);
    verifyEqual(test_case, scenario.name, string(expected_names{index}));
    verifyEqual(test_case, scenario.split, "test");
    verifyEqual(test_case, scenario.duration, 8.0);
    verifyClass(test_case, scenario.x_reference, "function_handle");
    verifyClass(test_case, scenario.force_disturbance, "function_handle");
    verifyClass(test_case, scenario.torque_disturbance, "function_handle");
end
end

function test_initial_tilt_and_position_step_values_are_exact(test_case)
scenarios = heldout_scenarios(8.0);

tilt_names = { ...
    'S1_initial_tilt_3deg'; 'S1_initial_tilt_6deg'; ...
    'S1_initial_tilt_9deg'; 'S1_initial_tilt_12deg'};
tilt_degrees = [3, 6, 9, 12];
for index = 1:numel(tilt_names)
    scenario = scenarios.(tilt_names{index});
    verifyEqual(test_case, scenario.initial_state, ...
        [0; 0; deg2rad(tilt_degrees(index)); 0], "AbsTol", 1e-15);
    verifyEqual(test_case, scenario.reference_amplitude, 0);
    verifyEqual(test_case, scenario.reference_start, 0);
    verifyEqual(test_case, scenario.disturbance_end, 0);
    verifyEqual(test_case, scenario.x_reference([0, 1, 8]), [0, 0, 0]);
end

step_names = { ...
    'S2_position_step_0p25m'; 'S2_position_step_0p75m'; ...
    'S2_position_step_1p0m'};
step_amplitudes = [0.25, 0.75, 1.0];
for index = 1:numel(step_names)
    scenario = scenarios.(step_names{index});
    verifyEqual(test_case, scenario.initial_state, zeros(4, 1));
    verifyEqual(test_case, scenario.reference_amplitude, ...
        step_amplitudes(index));
    verifyEqual(test_case, scenario.reference_start, 1.0);
    verifyEqual(test_case, scenario.x_reference([0.999, 1.0, 8.0]), ...
        [0, step_amplitudes(index), step_amplitudes(index)]);
    verifyEqual(test_case, scenario.disturbance_end, 0);
end
end

function test_force_scenarios_use_exact_metadata_and_half_open_timing(test_case)
scenarios = heldout_scenarios(8.0);

saturation = scenarios.S2_saturation_stress;
verifyEqual(test_case, saturation.initial_state, ...
    [0; 0; deg2rad(8); 0], "AbsTol", 1e-15);
verifyEqual(test_case, saturation.reference_amplitude, 1.0);
verifyEqual(test_case, saturation.reference_start, 0.5);
verifyEqual(test_case, saturation.force_amplitude, 10.0);
verifyEqual(test_case, saturation.force_start, 1.2);
verifyEqual(test_case, saturation.force_duration, 0.2);
verifyEqual(test_case, saturation.disturbance_end, 1.4, "AbsTol", 1e-15);
verifyEqual(test_case, saturation.x_reference([0.499, 0.5]), [0, 1]);
verifyEqual(test_case, saturation.force_disturbance( ...
    [1.199, 1.2, 1.399, 1.4]), [0, 10, 10, 0]);

positive = scenarios.S3_positive_impulse;
negative = scenarios.S3_negative_impulse;
verifyEqual(test_case, positive.reference_amplitude, 0.5);
verifyEqual(test_case, positive.reference_start, 1.0);
verifyEqual(test_case, positive.force_amplitude, 8.0);
verifyEqual(test_case, negative.force_amplitude, -8.0);
verifyEqual(test_case, positive.force_start, 4.0);
verifyEqual(test_case, positive.force_duration, 0.2);
verifyEqual(test_case, positive.disturbance_end, 4.2, "AbsTol", 1e-15);
verifyEqual(test_case, positive.force_disturbance( ...
    [3.999, 4.0, 4.199, 4.2]), [0, 8, 8, 0]);
verifyEqual(test_case, negative.force_disturbance( ...
    [3.999, 4.0, 4.199, 4.2]), [0, -8, -8, 0]);
end

function test_torque_and_constant_bias_scenarios_are_exact(test_case)
scenarios = heldout_scenarios(8.0);

torque = scenarios.S3_torque_impulse;
verifyEqual(test_case, torque.reference_amplitude, 0.5);
verifyEqual(test_case, torque.reference_start, 1.0);
verifyEqual(test_case, torque.torque_amplitude, 0.9);
verifyEqual(test_case, torque.torque_start, 4.0);
verifyEqual(test_case, torque.torque_duration, 0.15);
verifyEqual(test_case, torque.disturbance_end, 4.15, "AbsTol", 1e-15);
verifyEqual(test_case, torque.torque_disturbance( ...
    [3.999, 4.0, 4.149, 4.15]), [0, 0.9, 0.9, 0]);
verifyEqual(test_case, torque.force_disturbance([0, 4, 8]), [0, 0, 0]);

bias = scenarios.S5_constant_bias;
verifyEqual(test_case, bias.reference_amplitude, 0.5);
verifyEqual(test_case, bias.reference_start, 1.0);
verifyEqual(test_case, bias.constant_force, 0.8);
verifyEqual(test_case, bias.disturbance_end, 0);
verifyEqual(test_case, bias.force_disturbance([0, 4, 8]), [0.8, 0.8, 0.8]);
verifyEqual(test_case, bias.torque_disturbance([0, 4, 8]), [0, 0, 0]);
end

function test_duration_validation_is_stable_and_accepts_boundary(test_case)
boundary = heldout_scenarios(4.2);
verifyEqual(test_case, boundary.S3_positive_impulse.duration, 4.2);

invalid_values = {4.199, -1, NaN, Inf, 4.2 + 1i, [4.2, 5.0], "8"};
for index = 1:numel(invalid_values)
    verifyError(test_case, @() heldout_scenarios(invalid_values{index}), ...
        "twsbr:scenario:invalid_duration");
end
end
