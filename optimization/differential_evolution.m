function [best_vector, best_value, history] = differential_evolution( ...
        objective, space, options)
%DIFFERENTIAL_EVOLUTION Run exact-budget classic DE/rand/1/bin.

validate_objective(objective);
[dimension, lower_bounds, upper_bounds] = validate_space(space);
validated = validate_options(options, dimension, lower_bounds, upper_bounds);

population_size = validated.population_size;
evaluation_budget = validated.evaluation_budget;
stream = RandStream("mt19937ar", "Seed", validated.seed);
random_weights = rand(stream, population_size, dimension);
population = (1 - random_weights) .* lower_bounds + ...
    random_weights .* upper_bounds;
population = min(max(population, lower_bounds), upper_bounds);
if validated.has_starter
    population(1, :) = validated.starter_vector;
end

population_values = nan(population_size, 1);
evaluation = (1:evaluation_budget).';
candidate = cell(evaluation_budget, 1);
value = nan(evaluation_budget, 1);
best_history = nan(evaluation_budget, 1);
best_candidate = cell(evaluation_budget, 1);
best_value = Inf;
best_vector = population(1, :);
evaluation_count = 0;

for index = 1:population_size
    assert_candidate(population(index, :), lower_bounds, upper_bounds);
    objective_value = evaluate_objective(objective, population(index, :));
    population_values(index) = objective_value;
    if objective_value <= best_value
        best_value = objective_value;
        best_vector = population(index, :);
    end
    evaluation_count = evaluation_count + 1;
    [candidate, value, best_history, best_candidate] = record_evaluation( ...
        candidate, value, best_history, best_candidate, evaluation_count, ...
        population(index, :), objective_value, best_vector, best_value);
end

while evaluation_count < evaluation_budget
    frozen_population = population;
    frozen_values = population_values;
    next_population = frozen_population;
    next_values = frozen_values;
    for target = 1:population_size
        other_indices = [1:(target - 1), (target + 1):population_size];
        order = randperm(stream, population_size - 1, 3);
        donors = other_indices(order);
        mutant = stable_mutant(frozen_population(donors(1), :), ...
            frozen_population(donors(2), :), ...
            frozen_population(donors(3), :), validated.mutation_factor, ...
            lower_bounds, upper_bounds);

        mask = rand(stream, 1, dimension) < validated.crossover_rate;
        mask(randi(stream, dimension)) = true;
        trial = frozen_population(target, :);
        trial(mask) = mutant(mask);
        assert_candidate(trial, lower_bounds, upper_bounds);
        objective_value = evaluate_objective(objective, trial);

        if objective_value <= frozen_values(target)
            next_population(target, :) = trial;
            next_values(target) = objective_value;
        end
        if objective_value <= best_value
            best_value = objective_value;
            best_vector = trial;
        end
        evaluation_count = evaluation_count + 1;
        [candidate, value, best_history, best_candidate] = ...
            record_evaluation(candidate, value, best_history, ...
                best_candidate, evaluation_count, trial, objective_value, ...
                best_vector, best_value);
        if evaluation_count == evaluation_budget
            break
        end
    end
    population = next_population;
    population_values = next_values;
end

history = table(evaluation, candidate, value, best_history, best_candidate, ...
    'VariableNames', {'evaluation', 'candidate', 'value', ...
        'best_value', 'best_candidate'});
end

function [candidate, value, best_history, best_candidate] = ...
        record_evaluation(candidate, value, best_history, best_candidate, ...
            index, evaluated_candidate, objective_value, best_vector, best_value)
candidate{index} = evaluated_candidate;
value(index) = objective_value;
best_history(index) = best_value;
best_candidate{index} = best_vector;
end

function mutant = stable_mutant(base, second, third, ...
        mutation_factor, lower_bounds, upper_bounds)
mutant = base + mutation_factor * (second - third);
nonfinite = ~isfinite(mutant);
for dimension_index = find(nonfinite)
    scale = max(abs([base(dimension_index), second(dimension_index), ...
        third(dimension_index)]));
    if scale == 0
        repaired = 0.0;
    else
        normalized = base(dimension_index) / scale + mutation_factor * ...
            (second(dimension_index) / scale - ...
                third(dimension_index) / scale);
        if abs(normalized) > realmax / scale
            repaired = sign(normalized) * realmax;
        else
            repaired = scale * normalized;
        end
    end
    if ~isfinite(repaired)
        if normalized >= 0
            repaired = upper_bounds(dimension_index);
        else
            repaired = lower_bounds(dimension_index);
        end
    end
    mutant(dimension_index) = repaired;
end
mutant = min(max(mutant, lower_bounds), upper_bounds);
end

function assert_candidate(candidate, lower_bounds, upper_bounds)
if ~isnumeric(candidate) || ~isreal(candidate) || ...
        ~isequal(size(candidate), size(lower_bounds)) || ...
        any(~isfinite(candidate)) || any(candidate < lower_bounds) || ...
        any(candidate > upper_bounds)
    error("twsbr:de:invalid_candidate", ...
        "Generated candidates must remain finite and within bounds.");
end
end

function objective_value = evaluate_objective(objective, candidate)
objective_value = objective(candidate);
if ~isnumeric(objective_value) || ~isreal(objective_value) || ...
        ~isscalar(objective_value)
    error("twsbr:de:invalid_objective_output", ...
        "Objective values must be real numeric scalars.");
end
if ~isfinite(objective_value)
    error("twsbr:de:nonfinite_objective", ...
        "Objective values must be finite.");
end
objective_value = double(objective_value);
end

function validate_objective(objective)
if ~isa(objective, "function_handle") || ~isscalar(objective)
    error("twsbr:de:invalid_objective", ...
        "Objective must be a scalar function handle.");
end
end

function [dimension, lower_bounds, upper_bounds] = validate_space(space)
required_fields = ["dimension", "lower_bounds", "upper_bounds"];
if ~isstruct(space) || ~isscalar(space) || ...
        ~all(isfield(space, required_fields)) || ...
        ~is_integer_scalar(space.dimension) || space.dimension < 1
    invalid_space();
end
dimension = double(space.dimension);
raw_lower_bounds = space.lower_bounds;
raw_upper_bounds = space.upper_bounds;
if ~isnumeric(raw_lower_bounds) || ~isreal(raw_lower_bounds) || ...
        ~isequal(size(raw_lower_bounds), [1, dimension]) || ...
        ~isnumeric(raw_upper_bounds) || ~isreal(raw_upper_bounds) || ...
        ~isequal(size(raw_upper_bounds), [1, dimension])
    invalid_space();
end
lower_bounds = double(raw_lower_bounds);
upper_bounds = double(raw_upper_bounds);
if any(~isfinite(lower_bounds)) || ...
        any(~isfinite(upper_bounds)) || ...
        any(lower_bounds >= upper_bounds)
    invalid_space();
end
end

function validated = validate_options( ...
        options, dimension, lower_bounds, upper_bounds)
required_fields = ["population_size", "evaluation_budget", "seed", ...
    "mutation_factor", "crossover_rate"];
if ~isstruct(options) || ~isscalar(options) || ...
        ~all(isfield(options, required_fields))
    error("twsbr:de:invalid_options", ...
        "Options must contain every required optimizer setting.");
end
if ~is_integer_scalar(options.population_size) || ...
        double(options.population_size) < 4
    error("twsbr:de:invalid_population_size", ...
        "Population size must be an integer of at least four.");
end
if ~is_integer_scalar(options.evaluation_budget) || ...
        double(options.evaluation_budget) < double(options.population_size)
    error("twsbr:de:invalid_budget", ...
        "Evaluation budget must be an integer no smaller than the population.");
end
if ~is_integer_scalar(options.seed) || double(options.seed) < 0 || ...
        double(options.seed) > 2^32 - 1
    error("twsbr:de:invalid_seed", ...
        "Seed must be an integer from zero through 2^32-1.");
end
if ~is_real_finite_scalar(options.mutation_factor) || ...
        double(options.mutation_factor) <= 0 || double(options.mutation_factor) > 2
    error("twsbr:de:invalid_mutation_factor", ...
        "Mutation factor must be in the interval (0,2].");
end
if ~is_real_finite_scalar(options.crossover_rate) || ...
        double(options.crossover_rate) < 0 || double(options.crossover_rate) > 1
    error("twsbr:de:invalid_crossover_rate", ...
        "Crossover rate must be in the interval [0,1].");
end

validated = options;
validated.population_size = double(options.population_size);
validated.evaluation_budget = double(options.evaluation_budget);
validated.seed = double(options.seed);
validated.mutation_factor = double(options.mutation_factor);
validated.crossover_rate = double(options.crossover_rate);
validated.has_starter = isfield(options, "starter_vector");
if validated.has_starter
    raw_starter = options.starter_vector;
    if ~isnumeric(raw_starter) || ~isreal(raw_starter) || ...
            ~isequal(size(raw_starter), [1, dimension]) || ...
            any(~isfinite(raw_starter))
        error("twsbr:de:invalid_starter_vector", ...
            "Starter vector must be a finite bounded row vector.");
    end
    starter = double(raw_starter);
    if any(starter < lower_bounds) || any(starter > upper_bounds)
        error("twsbr:de:invalid_starter_vector", ...
            "Starter vector must be a finite bounded row vector.");
    end
    validated.starter_vector = starter;
end
end

function valid = is_integer_scalar(value)
valid = is_real_finite_scalar(value) && fix(value) == value;
end

function valid = is_real_finite_scalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value);
end

function invalid_space()
error("twsbr:de:invalid_space", ...
    "Space must contain finite ordered row-vector bounds.");
end
