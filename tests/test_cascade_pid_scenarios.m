function tests = test_cascade_pid_scenarios
%TEST_CASCADE_PID_SCENARIOS Tests for the frozen cascade PID scenarios.
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

function test_exact_scenario_names_and_fields_are_frozen(test_case)
scenarios = cascade_pid_scenarios();

verifyEqual(test_case, string(fieldnames(scenarios)), [ ...
    "zero_state"; "positive_position_step"; "negative_position_step"; ...
    "initial_tilt"; "force_impulse"]);

expected_fields = sort(cellstr(["disturbance_end"; "duration"; ...
    "force_disturbance"; "initial_state"; "name"; ...
    "reference_start"; "torque_disturbance"; "x_reference"]));
scenario_names = fieldnames(scenarios);
for index = 1:numel(scenario_names)
    scenario = scenarios.(scenario_names{index});
    verifyEqual(test_case, sort(fieldnames(scenario)), expected_fields);
    verifyEqual(test_case, string(scenario.name), string(scenario_names{index}));
end
end

function test_durations_and_initial_conditions_are_exact(test_case)
scenarios = cascade_pid_scenarios();

verifyEqual(test_case, scenarios.zero_state.duration, 2.0);
verifyEqual(test_case, scenarios.positive_position_step.duration, 8.0);
verifyEqual(test_case, scenarios.negative_position_step.duration, 8.0);
verifyEqual(test_case, scenarios.initial_tilt.duration, 5.0);
verifyEqual(test_case, scenarios.force_impulse.duration, 8.0);

verifyEqual(test_case, scenarios.zero_state.initial_state, zeros(4, 1));
verifyEqual(test_case, scenarios.positive_position_step.initial_state, zeros(4, 1));
verifyEqual(test_case, scenarios.negative_position_step.initial_state, zeros(4, 1));
verifyEqual(test_case, scenarios.initial_tilt.initial_state, ...
    [0.0; 0.0; deg2rad(5.0); 0.0], "AbsTol", 1e-15);
verifyEqual(test_case, scenarios.force_impulse.initial_state, zeros(4, 1));
end

function test_position_references_use_exact_step_boundaries(test_case)
scenarios = cascade_pid_scenarios();
times = [0.0, 0.999, 1.0, 1.001, 8.0];

verifyEqual(test_case, scenarios.zero_state.x_reference(times), zeros(size(times)));
verifyEqual(test_case, scenarios.positive_position_step.x_reference(times), ...
    [0.0, 0.0, 0.5, 0.5, 0.5]);
verifyEqual(test_case, scenarios.negative_position_step.x_reference(times), ...
    [0.0, 0.0, -0.5, -0.5, -0.5]);
verifyEqual(test_case, scenarios.initial_tilt.x_reference(times), zeros(size(times)));
verifyEqual(test_case, scenarios.force_impulse.x_reference(times), ...
    [0.0, 0.0, 0.5, 0.5, 0.5]);
end

function test_force_impulse_is_half_open_and_vector_safe(test_case)
scenarios = cascade_pid_scenarios();
times = [0.0, 3.999, 4.0, 4.199, 4.2, 8.0];

verifyEqual(test_case, scenarios.force_impulse.force_disturbance(times), ...
    [0.0, 0.0, 5.0, 5.0, 0.0, 0.0]);
verifyEqual(test_case, scenarios.force_impulse.force_disturbance(times.'), ...
    [0.0; 0.0; 5.0; 5.0; 0.0; 0.0]);

scenario_names = fieldnames(scenarios);
for index = 1:numel(scenario_names)
    scenario = scenarios.(scenario_names{index});
    verifySize(test_case, scenario.x_reference(times), size(times));
    verifySize(test_case, scenario.force_disturbance(times), size(times));
    verifyEqual(test_case, scenario.torque_disturbance(times), zeros(size(times)));
end
end

function test_event_metadata_is_finite_and_exact(test_case)
scenarios = cascade_pid_scenarios();
expected_reference_starts = [0.0, 1.0, 1.0, 0.0, 1.0];
expected_disturbance_ends = [0.0, 0.0, 0.0, 0.0, 4.2];
scenario_names = fieldnames(scenarios);

for index = 1:numel(scenario_names)
    scenario = scenarios.(scenario_names{index});
    verifyTrue(test_case, isnumeric(scenario.reference_start) && ...
        isscalar(scenario.reference_start) && isfinite(scenario.reference_start));
    verifyTrue(test_case, isnumeric(scenario.disturbance_end) && ...
        isscalar(scenario.disturbance_end) && isfinite(scenario.disturbance_end));
    verifyEqual(test_case, scenario.reference_start, expected_reference_starts(index));
    verifyEqual(test_case, scenario.disturbance_end, expected_disturbance_ends(index));
end
end
