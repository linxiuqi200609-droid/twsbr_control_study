function tests = test_deterministic_batch
%TEST_DETERMINISTIC_BATCH Tests for complete deterministic and benchmark batches.
local_functions = localfunctions;
function_names = string(cellfun(@func2str, local_functions, ...
    "UniformOutput", false));
tests = functiontests(local_functions(~endsWith( ...
    function_names, "starter_vectors_for_test")));
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

function test_deterministic_batch_keeps_every_controller_scenario_pair(test_case)
[vectors, config] = starter_vectors_for_test();
scenario_struct = heldout_scenarios(10);
names = fieldnames(scenario_struct);
scenario_struct = rmfield(scenario_struct, names(3:end));

[metrics, raw] = run_deterministic_batch(vectors, twsbr_params(), ...
    config, scenario_struct, 5);

verifyEqual(test_case, height(metrics), 5 * 2);
verifyEqual(test_case, numel(fieldnames(raw)), 5 * 2);
verifyEqual(test_case, sort(unique(metrics.controller)), ...
    sort(config.controller_names));
verifyEqual(test_case, metrics.controller, ...
    repelem(config.controller_names, 2));
verifyEqual(test_case, metrics.scenario, repmat([ ...
    "S1_initial_tilt_3deg"; "S1_initial_tilt_6deg"], 5, 1));
end

function test_deterministic_batch_uses_stable_raw_lookup_keys(test_case)
[vectors, config] = starter_vectors_for_test();
scenario_struct = heldout_scenarios(4.2);
names = fieldnames(scenario_struct);
scenario_struct = rmfield(scenario_struct, names(2:end));

[metrics, raw] = run_deterministic_batch(vectors, twsbr_params(), ...
    config, scenario_struct, 17);

expected_keys = config.controller_names + "__S1_initial_tilt_3deg";
verifyEqual(test_case, string(fieldnames(raw)), expected_keys);
verifyEqual(test_case, metrics.seed, repmat(17, 5, 1));
for index = 1:numel(expected_keys)
    simulation = raw.(char(expected_keys(index)));
    verifyEqual(test_case, simulation.controller_name, ...
        config.controller_names(index));
    verifyEqual(test_case, simulation.scenario_name, ...
        "S1_initial_tilt_3deg");
end
end

function test_benchmark_reports_all_controllers_and_dimensions(test_case)
[vectors, config] = starter_vectors_for_test();
config.benchmark_repeats = 2;

complexity = benchmark_controllers(vectors, twsbr_params(), config);

verifyEqual(test_case, complexity.Properties.VariableNames, { ...
    'controller', 'calls', 'total_seconds', ...
    'mean_step_runtime_us', 'parameter_dimension'});
verifyEqual(test_case, complexity.controller, config.controller_names);
verifyEqual(test_case, complexity.calls, repmat(2, 5, 1));
verifyEqual(test_case, complexity.parameter_dimension, [3; 5; 9; 5; 6]);
verifyTrue(test_case, all(isfinite(complexity.total_seconds)));
verifyTrue(test_case, all(complexity.total_seconds >= 0));
verifyTrue(test_case, all(isfinite(complexity.mean_step_runtime_us)));
verifyTrue(test_case, all(complexity.mean_step_runtime_us >= 0));
end

function [vectors, config] = starter_vectors_for_test()
config = experiment_config("quick");
vectors = struct();
vectors.ATTITUDE_PID = log10([1.9,0.2,0.18]);
vectors.CASCADE_PID = log10([0.241,0.000396,0.193,9.255,1.011]);
vectors.FUZZY_PID = [log10([0.241,0.000396,0.193, ...
    9.255,0.05,1.011]),0.2,0.2,0.2];
vectors.LQR = log10([10,1,200,10,0.1]);
vectors.LQI = log10([10,1,200,10,100,0.1]);
end
