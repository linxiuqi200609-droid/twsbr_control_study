function tests = test_tuning_pipeline
%TEST_TUNING_PIPELINE Tests for fair multi-seed controller tuning.
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

function test_selection_uses_only_training_cost(test_case)
runs = table(["LQR"; "LQR"; "LQI"; "LQI"], [0; 1; 0; 1], ...
    [2.0; 1.0; 4.0; 3.0], {[1, 2]; [3, 4]; [5, 6]; [7, 8]}, ...
    'VariableNames', {'controller', 'seed', 'training_cost', 'vector'});
runs.test_cost = [0.0; 100.0; 0.0; 100.0];

frozen = select_frozen_parameters(runs);

verifyEqual(test_case, frozen.LQR, [3, 4]);
verifyEqual(test_case, frozen.LQI, [7, 8]);
end

function test_selection_breaks_equal_cost_ties_by_seed(test_case)
runs = table(["LQR"; "LQR"; "LQR"], [7; 2; 5], [1; 1; 2], ...
    {7; 2; 5}, 'VariableNames', ...
    {'controller', 'seed', 'training_cost', 'vector'});

frozen = select_frozen_parameters(runs);

verifyEqual(test_case, frozen.LQR, 2);
end

function test_tune_controller_uses_same_exact_budget_per_seed(test_case)
config = compact_config([2, 3]);
runs = tune_controller("ATTITUDE_PID", twsbr_params(), config, ...
    objective_config(), training_scenarios(3.2), ...
    log10([1.9, 0.2, 0.18]));

verifyEqual(test_case, runs.evaluation_count, [4; 4]);
verifyEqual(test_case, runs.seed, [2; 3]);
verifyEqual(test_case, runs.controller, ...
    ["ATTITUDE_PID"; "ATTITUDE_PID"]);
verifyEqual(test_case, runs.Properties.VariableNames, ...
    {'controller', 'seed', 'training_cost', 'vector', ...
    'evaluation_count', 'elapsed_seconds'});
verifyTrue(test_case, all(isfinite(runs.training_cost)));
verifyTrue(test_case, all(runs.elapsed_seconds >= 0));
verifyTrue(test_case, all(cellfun(@(value) isequal(size(value), [1, 3]), ...
    runs.vector)));
end

function test_tune_all_controllers_preserves_order_and_equal_budget(test_case)
config = compact_config(4);
starter_vectors = controller_starters();
runs = tune_all_controllers(twsbr_params(), config, objective_config(), ...
    training_scenarios(3.2), starter_vectors);

verifyEqual(test_case, runs.controller, config.controller_names);
verifyEqual(test_case, runs.seed, repmat(4, 5, 1));
verifyEqual(test_case, runs.evaluation_count, repmat(4, 5, 1));
for index = 1:height(runs)
    space = controller_parameter_space(runs.controller(index));
    verifyEqual(test_case, size(runs.vector{index}), [1, space.dimension]);
end
end
function test_tune_all_controllers_accepts_column_starters(test_case)
config = compact_config(5);
starter_vectors = controller_starters();
names = config.controller_names;
for index = 1:numel(names)
    name = char(names(index));
    starter = starter_vectors.(name);
    starter_vectors.(name) = starter(:);
end

runs = tune_all_controllers(twsbr_params(), config, objective_config(), ...
    training_scenarios(3.2), starter_vectors);

verifyEqual(test_case, runs.controller, names);
verifyEqual(test_case, runs.evaluation_count, repmat(4, 5, 1));
verifyTrue(test_case, all(cellfun(@isrow, runs.vector)));
end


function test_starter_vectors_are_optional(test_case)
config = compact_config(6);
scenarios = training_scenarios(3.2);
without_starter = tune_controller("ATTITUDE_PID", twsbr_params(), ...
    config, objective_config(), scenarios);
with_starter = tune_controller("ATTITUDE_PID", twsbr_params(), ...
    config, objective_config(), scenarios, log10([1.9, 0.2, 0.18]));

verifyEqual(test_case, without_starter.evaluation_count, 4);
verifyEqual(test_case, with_starter.evaluation_count, 4);
verifyEqual(test_case, without_starter.seed, 6);

verifyEqual(test_case, with_starter.seed, 6);
end

function test_starter_vector_is_normalized_to_a_row(test_case)
config = compact_config(8);
runs = tune_controller("ATTITUDE_PID", twsbr_params(), config, ...
    objective_config(), training_scenarios(3.2), ...
    log10([1.9; 0.2; 0.18]));

verifyEqual(test_case, runs.evaluation_count, 4);
verifyEqual(test_case, size(runs.vector{1}), [1, 3]);
end


function test_tune_controller_checks_required_arguments_first(test_case)
verifyError(test_case, @() tune_controller("ATTITUDE_PID"), ...
    "twsbr:tuning:invalid_input");
end

function test_mixed_integer_config_uses_exact_budget(test_case)
config = compact_config(uint32([2, 3]));
config.population_size = uint8(4);
config.evaluation_budget = uint16(8);
runs = tune_controller("ATTITUDE_PID", twsbr_params(), config, ...
    objective_config(), training_scenarios(3.2), ...
    log10([1.9, 0.2, 0.18]));

verifyEqual(test_case, runs.seed, [2; 3]);
verifyEqual(test_case, runs.evaluation_count, [8; 8]);
verifyTrue(test_case, all(isfinite(runs.elapsed_seconds)));
verifyTrue(test_case, all(runs.elapsed_seconds >= 0));
end

function test_equal_cost_and_seed_tie_preserves_first_row(test_case)
runs = table(["LQR"; "LQR"; "LQR"], [3; 3; 4], [1; 1; 0], ...
    {11; 22; 33}, 'VariableNames', ...
    {'controller', 'seed', 'training_cost', 'vector'});

frozen = select_frozen_parameters(runs);

verifyEqual(test_case, frozen.LQR, 33);
same_minimum = runs(1:2, :);
same_minimum.training_cost(:) = 1;
frozen = select_frozen_parameters(same_minimum);
verifyEqual(test_case, frozen.LQR, 11);
end

function test_tune_all_controllers_without_starters(test_case)
config = compact_config(9);
runs = tune_all_controllers(twsbr_params(), config, objective_config(), ...
    training_scenarios(3.2));

verifyEqual(test_case, runs.controller, config.controller_names);
verifyEqual(test_case, runs.seed, repmat(9, 5, 1));
verifyEqual(test_case, runs.evaluation_count, repmat(4, 5, 1));
verifyTrue(test_case, all(isfinite(runs.elapsed_seconds)));
verifyTrue(test_case, all(runs.elapsed_seconds >= 0));
end

function test_invalid_pipeline_inputs_use_stable_identifiers(test_case)
config = compact_config(0);
plant = twsbr_params();
objective = objective_config();
scenarios = training_scenarios(3.2);
verifyError(test_case, @() tune_controller("ATTITUDE_PID", plant, ...
    rmfield(config, "evaluation_budget"), objective, scenarios), ...
    "twsbr:tuning:invalid_config");
verifyError(test_case, @() tune_controller("ATTITUDE_PID", plant, ...
    config, objective, scenarios, [100, 100, 100]), ...
    "twsbr:de:invalid_starter_vector");
verifyError(test_case, @() tune_all_controllers(plant, config, ...
    objective, scenarios, struct("ATTITUDE_PID", [0, 0])), ...
    "twsbr:tuning:invalid_starters");

bad_runs = table("LQR", 0, NaN, {[1, 2]}, 'VariableNames', ...
    {'controller', 'seed', 'training_cost', 'vector'});
verifyError(test_case, @() select_frozen_parameters(bad_runs), ...
    "twsbr:tuning:invalid_runs");
end

function config = compact_config(seeds)
config = experiment_config("quick");
config.population_size = 4;
config.evaluation_budget = 4;
config.tuning_seeds = seeds;
end

function starters = controller_starters()
starters = struct( ...
    "ATTITUDE_PID", log10([1.9, 0.2, 0.18]), ...
    "CASCADE_PID", log10([0.24100028146267993, ...
        0.0003962067755988572, 0.1930824033173246, ...
        9.254929149177556, 1.0113335430173094]), ...
    "FUZZY_PID", [log10([0.24, 0.0004, 0.19, 9.25, 0.05, 1.01]), ...
        0.2, 0.2, 0.2], ...
    "LQR", log10([10, 1, 200, 10, 0.1]), ...
    "LQI", log10([10, 1, 200, 10, 100, 0.1]));
end
