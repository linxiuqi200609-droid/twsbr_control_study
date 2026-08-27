function tests = test_differential_evolution
%TEST_DIFFERENTIAL_EVOLUTION Tests for exact-budget DE/rand/1/bin.
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

function test_de_uses_exact_budget_bounds_and_starter(test_case)
space = make_space(3);
options = make_options(8, 31);
options.starter_vector = [0.5, -0.5, 0.25];
[best, value, history] = differential_evolution( ...
    @(candidate) sum(candidate .^ 2), space, options);

verifyEqual(test_case, height(history), 31);
verifyEqual(test_case, history.evaluation, (1:31).');
verifyEqual(test_case, history.candidate{1}, options.starter_vector);
verifyEqual(test_case, string(history.Properties.VariableNames), ...
    ["evaluation", "candidate", "value", "best_value", "best_candidate"]);
verifyGreaterThanOrEqual(test_case, best, space.lower_bounds);
verifyLessThanOrEqual(test_case, best, space.upper_bounds);
verifyLessThanOrEqual(test_case, value, sum(options.starter_vector .^ 2));
verifyEqual(test_case, value, history.best_value(end));
verifyEqual(test_case, best, history.best_candidate{end});
verifyLessThanOrEqual(test_case, diff(history.best_value), zeros(30, 1));
for index = 1:height(history)
    verifySize(test_case, history.candidate{index}, [1, space.dimension]);
    verifyGreaterThanOrEqual(test_case, ...
        history.candidate{index}, space.lower_bounds);
    verifyLessThanOrEqual(test_case, ...
        history.candidate{index}, space.upper_bounds);
end
end

function test_de_is_deterministic_without_global_rng_changes(test_case)
original_rng = rng;
rng_cleanup = onCleanup(@() rng(original_rng));
rng(77, "twister");
before = rng;
[first, first_value, first_history] = run_small_de();
middle = rng;
[second, second_value, second_history] = run_small_de();
after = rng;

verifyEqual(test_case, first, second);
verifyEqual(test_case, first_value, second_value);
verifyEqual(test_case, first_history, second_history);
verifyEqual(test_case, middle, before);
verifyEqual(test_case, after, before);
end

function test_de_supports_absent_starter_and_mid_generation_stop(test_case)
space = make_space(2);
options = make_options(5, 12);
call_count = 0;
[best, value, history] = differential_evolution( ...
    @counted_objective, space, options);

verifyEqual(test_case, call_count, options.evaluation_budget);
verifyEqual(test_case, height(history), options.evaluation_budget);
verifyEqual(test_case, history.evaluation, (1:12).');
verifyTrue(test_case, all(isfinite(history.value)));
verifyTrue(test_case, isfinite(value));
verifySize(test_case, best, [1, 2]);

    function objective_value = counted_objective(candidate)
        call_count = call_count + 1;
        objective_value = sum((candidate - 0.2) .^ 2);
    end
end

function test_integer_inputs_are_normalized_to_double(test_case)
space = struct("dimension", uint8(2), ...
    "lower_bounds", int8([-1, -1]), ...
    "upper_bounds", int8([1, 1]));
options = struct("population_size", uint8(4), ...
    "evaluation_budget", uint8(12), "seed", uint32(7), ...
    "mutation_factor", uint8(1), "crossover_rate", uint8(1), ...
    "starter_vector", int8([0, 0]));
[best, value, history] = differential_evolution( ...
    @(candidate) single(sum(candidate .^ 2)), space, options);

verifyClass(test_case, best, "double");
verifyClass(test_case, value, "double");
verifyClass(test_case, history.value, "double");
verifyEqual(test_case, history.candidate{1}, [0, 0]);
for index = 1:height(history)
    verifyClass(test_case, history.candidate{index}, "double");
    verifyClass(test_case, history.best_candidate{index}, "double");
end
end

function test_extreme_bounds_match_scaled_reference_in_both_directions(test_case)
space = struct("dimension", 3, ...
    "lower_bounds", -realmax * ones(1, 3), ...
    "upper_bounds", realmax * ones(1, 3));
options = make_options(4, 20);
options.seed = 0;
options.mutation_factor = 2.0;
options.crossover_rate = 1.0;
[best, value, history] = differential_evolution(@(~) 1.0, space, options);
reference = scaled_extreme_reference(space, options);
actual_candidates = cell2mat(history.candidate);
expected_candidates = cell2mat(reference.candidate);

verifyEqual(test_case, value, 1.0);
verifyTrue(test_case, all(isfinite(best)));
verifyGreaterThanOrEqual(test_case, best, space.lower_bounds);
verifyLessThanOrEqual(test_case, best, space.upper_bounds);
verifyTrue(test_case, any(actual_candidates(:) == realmax));
verifyTrue(test_case, any(actual_candidates(:) == -realmax));
verifyEqual(test_case, actual_candidates, expected_candidates, ...
    "RelTol", 16 * eps);
for index = 1:height(history)
    verifyTrue(test_case, all(isfinite(history.candidate{index})));
    verifyTrue(test_case, all(isfinite(history.best_candidate{index})));
    verifyGreaterThanOrEqual(test_case, ...
        history.candidate{index}, space.lower_bounds);
    verifyLessThanOrEqual(test_case, ...
        history.candidate{index}, space.upper_bounds);
end
end

function test_candidate_history_matches_independent_generational_reference(test_case)
space = make_space(3);
options = make_options(5, 12);
options.seed = 23;
[best, value, history] = differential_evolution( ...
    @(candidate) sum(candidate .^ 2), space, options);
reference = classic_generational_reference(space, options);

verifyEqual(test_case, history.candidate, reference.candidate);
verifyEqual(test_case, history.value, reference.value);
verifyEqual(test_case, history.best_value, reference.best_value);
verifyEqual(test_case, history.best_candidate, reference.best_candidate);
verifyEqual(test_case, best, reference.best_candidate{end});
verifyEqual(test_case, value, reference.best_value(end));
end

function test_population_only_budget_and_history_are_exactly_auditable(test_case)
space = make_space(2);
options = make_options(4, 4);
call_count = 0;
[best, value, history] = differential_evolution(@counted_objective, ...
    space, options);

verifyEqual(test_case, call_count, options.evaluation_budget);
verifyEqual(test_case, height(history), options.evaluation_budget);
verifyEqual(test_case, history.evaluation, (1:4).');
for index = 1:height(history)
    verifyEqual(test_case, history.value(index), ...
        sum(history.candidate{index} .^ 2));
    verifyEqual(test_case, history.best_value(index), ...
        min(history.value(1:index)));
    verifyEqual(test_case, sum(history.best_candidate{index} .^ 2), ...
        history.best_value(index));
end
verifyEqual(test_case, value, min(history.value));
verifyEqual(test_case, sum(best .^ 2), value);

    function objective_value = counted_objective(candidate)
        call_count = call_count + 1;
        objective_value = sum(candidate .^ 2);
    end
end

function test_invalid_objective_has_stable_error(test_case)
verifyError(test_case, @() differential_evolution( ...
    3, make_space(2), make_options(4, 8)), ...
    "twsbr:de:invalid_objective");
end

function test_invalid_spaces_have_stable_error(test_case)
options = make_options(4, 8);
bad_dimension = make_space(2);
bad_dimension.dimension = 1.5;
bad_shape = make_space(2);
bad_shape.lower_bounds = [-1; -1];
bad_order = make_space(2);
bad_order.lower_bounds(1) = bad_order.upper_bounds(1);
bad_nonfinite = make_space(2);
bad_nonfinite.upper_bounds(2) = Inf;
invalid_spaces = {struct(), bad_dimension, bad_shape, bad_order, bad_nonfinite};
for index = 1:numel(invalid_spaces)
    verifyError(test_case, @() differential_evolution( ...
        @(candidate) sum(candidate .^ 2), invalid_spaces{index}, options), ...
        "twsbr:de:invalid_space");
end
end

function test_invalid_options_have_specific_error_ids(test_case)
space = make_space(2);
objective = @(candidate) sum(candidate .^ 2);
invalid_cases = { ...
    rmfield(make_options(4, 8), "seed"), "twsbr:de:invalid_options"; ...
    set_option(make_options(4, 8), "population_size", 3), ...
        "twsbr:de:invalid_population_size"; ...
    set_option(make_options(4, 8), "evaluation_budget", 3), ...
        "twsbr:de:invalid_budget"; ...
    set_option(make_options(4, 8), "seed", 0.5), ...
        "twsbr:de:invalid_seed"; ...
    set_option(make_options(4, 8), "mutation_factor", 0), ...
        "twsbr:de:invalid_mutation_factor"; ...
    set_option(make_options(4, 8), "crossover_rate", 1.1), ...
        "twsbr:de:invalid_crossover_rate"};
for index = 1:size(invalid_cases, 1)
    verifyError(test_case, @() differential_evolution( ...
        objective, space, invalid_cases{index, 1}), invalid_cases{index, 2});
end
end

function test_invalid_starter_has_stable_error(test_case)
space = make_space(2);
invalid_starters = {0, [0; 0], [0, Inf], [0, 2]};
for index = 1:numel(invalid_starters)
    options = make_options(4, 8);
    options.starter_vector = invalid_starters{index};
    verifyError(test_case, @() differential_evolution( ...
        @(candidate) sum(candidate .^ 2), space, options), ...
        "twsbr:de:invalid_starter_vector");
end
end

function test_invalid_objective_outputs_have_stable_error_ids(test_case)
space = make_space(2);
options = make_options(4, 8);
verifyError(test_case, @() differential_evolution( ...
    @(~) NaN, space, options), "twsbr:de:nonfinite_objective");
verifyError(test_case, @() differential_evolution( ...
    @(~) Inf, space, options), "twsbr:de:nonfinite_objective");
verifyError(test_case, @() differential_evolution( ...
    @(~) [1, 2], space, options), "twsbr:de:invalid_objective_output");
verifyError(test_case, @() differential_evolution( ...
    @(~) 1i, space, options), "twsbr:de:invalid_objective_output");
end

function [best, value, history] = run_small_de()
space = make_space(2);
options = make_options(4, 12);
options.seed = 5;
options.starter_vector = [0.5, -0.5];
[best, value, history] = differential_evolution( ...
    @(candidate) sum(candidate .^ 2), space, options);
end

function history = classic_generational_reference(space, options)
stream = RandStream("mt19937ar", "Seed", options.seed);
random_weights = rand(stream, options.population_size, space.dimension);
population = (1 - random_weights) .* space.lower_bounds + ...
    random_weights .* space.upper_bounds;
values = nan(options.population_size, 1);
candidate = cell(options.evaluation_budget, 1);
objective_value = nan(options.evaluation_budget, 1);
best_value = nan(options.evaluation_budget, 1);
best_candidate = cell(options.evaluation_budget, 1);
current_best_value = Inf;
current_best = population(1, :);
evaluation_count = 0;

for index = 1:options.population_size
    values(index) = sum(population(index, :) .^ 2);
    if values(index) <= current_best_value
        current_best_value = values(index);
        current_best = population(index, :);
    end
    evaluation_count = evaluation_count + 1;
    candidate{evaluation_count} = population(index, :);
    objective_value(evaluation_count) = values(index);
    best_value(evaluation_count) = current_best_value;
    best_candidate{evaluation_count} = current_best;
end

while evaluation_count < options.evaluation_budget
    frozen_population = population;
    frozen_values = values;
    next_population = frozen_population;
    next_values = frozen_values;
    for target = 1:options.population_size
        other_indices = [1:(target - 1), ...
            (target + 1):options.population_size];
        order = randperm(stream, options.population_size - 1, 3);
        donors = other_indices(order);
        mutant = frozen_population(donors(1), :) + ...
            options.mutation_factor * (frozen_population(donors(2), :) - ...
                frozen_population(donors(3), :));
        mutant = min(max(mutant, space.lower_bounds), space.upper_bounds);
        mask = rand(stream, 1, space.dimension) < options.crossover_rate;
        mask(randi(stream, space.dimension)) = true;
        trial = frozen_population(target, :);
        trial(mask) = mutant(mask);
        trial_value = sum(trial .^ 2);
        if trial_value <= frozen_values(target)
            next_population(target, :) = trial;
            next_values(target) = trial_value;
        end
        if trial_value <= current_best_value
            current_best_value = trial_value;
            current_best = trial;
        end
        evaluation_count = evaluation_count + 1;
        candidate{evaluation_count} = trial;
        objective_value(evaluation_count) = trial_value;
        best_value(evaluation_count) = current_best_value;
        best_candidate{evaluation_count} = current_best;
        if evaluation_count == options.evaluation_budget
            break
        end
    end
    population = next_population;
    values = next_values;
end
history = struct("candidate", {candidate}, "value", objective_value, ...
    "best_value", best_value, "best_candidate", {best_candidate});
end

function history = scaled_extreme_reference(space, options)
stream = RandStream("mt19937ar", "Seed", options.seed);
random_weights = rand(stream, options.population_size, space.dimension);
population = (1 - random_weights) .* space.lower_bounds + ...
    random_weights .* space.upper_bounds;
candidate = cell(options.evaluation_budget, 1);
evaluation_count = 0;

for index = 1:options.population_size
    evaluation_count = evaluation_count + 1;
    candidate{evaluation_count} = population(index, :);
end

while evaluation_count < options.evaluation_budget
    frozen_population = population;
    next_population = frozen_population;
    for target = 1:options.population_size
        other_indices = [1:(target - 1), ...
            (target + 1):options.population_size];
        order = randperm(stream, options.population_size - 1, 3);
        donors = other_indices(order);
        mutant = scaled_mutant_reference( ...
            frozen_population(donors(1), :), ...
            frozen_population(donors(2), :), ...
            frozen_population(donors(3), :), options.mutation_factor, ...
            space.lower_bounds, space.upper_bounds);
        mask = rand(stream, 1, space.dimension) < options.crossover_rate;
        mask(randi(stream, space.dimension)) = true;
        trial = frozen_population(target, :);
        trial(mask) = mutant(mask);
        next_population(target, :) = trial;
        evaluation_count = evaluation_count + 1;
        candidate{evaluation_count} = trial;
        if evaluation_count == options.evaluation_budget
            break
        end
    end
    population = next_population;
end
history = struct("candidate", {candidate});
end

function mutant = scaled_mutant_reference(base, second, third, ...
        mutation_factor, lower_bounds, upper_bounds)
mutant = zeros(size(base));
for index = 1:numel(base)
    scale = max(abs([base(index), second(index), third(index)]));
    if scale == 0
        normalized = 0;
    else
        normalized = base(index) / scale + mutation_factor * ...
            (second(index) / scale - third(index) / scale);
    end
    if scale == 0
        mutant(index) = 0;
    elseif normalized > upper_bounds(index) / scale
        mutant(index) = upper_bounds(index);
    elseif normalized < lower_bounds(index) / scale
        mutant(index) = lower_bounds(index);
    else
        mutant(index) = scale * normalized;
    end
end
end

function space = make_space(dimension)
space = struct("dimension", dimension, ...
    "lower_bounds", -ones(1, dimension), ...
    "upper_bounds", ones(1, dimension));
end

function options = make_options(population_size, evaluation_budget)
options = struct("population_size", population_size, ...
    "evaluation_budget", evaluation_budget, "seed", 9, ...
    "mutation_factor", 0.8, "crossover_rate", 0.9);
end

function options = set_option(options, name, value)
options.(name) = value;
end
