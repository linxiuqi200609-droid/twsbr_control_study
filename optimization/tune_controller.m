function runs = tune_controller(controller_name, plant_params, config, ...
        objective, scenarios, starter_vector)
%TUNE_CONTROLLER Tune one controller independently for every study seed.

if nargin < 5
    error("twsbr:tuning:invalid_input", ...
        "Tuning requires controller, plant, config, objective, and scenarios.");
end
space = controller_parameter_space(controller_name);
[population_size, evaluation_budget, seeds] = ...
    validate_tuning_config(config);
has_starter = nargin == 6;
if has_starter
    starter_vector = normalize_starter(starter_vector, space);
end

run_count = numel(seeds);
controller = repmat(space.name, run_count, 1);
seed = seeds(:);
training_cost = zeros(run_count, 1);
vector = cell(run_count, 1);
evaluation_count = zeros(run_count, 1);
elapsed_seconds = zeros(run_count, 1);

for index = 1:run_count
    current_seed = seed(index);
    objective_handle = @(candidate) control_objective(space.name, ...
        candidate, plant_params, config, objective, scenarios, current_seed);
    options = struct( ...
        "population_size", population_size, ...
        "evaluation_budget", evaluation_budget, ...
        "seed", current_seed, ...
        "mutation_factor", 0.8, ...
        "crossover_rate", 0.9);
    if has_starter
        options.starter_vector = starter_vector;
    end

    run_start = tic;
    [best_vector, best_value, history] = differential_evolution( ...
        objective_handle, space, options);
    elapsed_seconds(index) = toc(run_start);
    training_cost(index) = best_value;
    vector{index} = best_vector;
    evaluation_count(index) = height(history);
end

runs = table(controller, seed, training_cost, vector, evaluation_count, ...
    elapsed_seconds);
end

function [population_size, evaluation_budget, seeds] = validate_tuning_config(config)
required_fields = ["population_size", "evaluation_budget", "tuning_seeds"];
if ~isstruct(config) || ~isscalar(config) || ...
        ~all(isfield(config, required_fields))
    invalid_config();
end
raw_population_size = config.population_size;
raw_evaluation_budget = config.evaluation_budget;
raw_seeds = config.tuning_seeds;
if ~is_integer_scalar(raw_population_size) || ...
        ~is_integer_scalar(raw_evaluation_budget) || ...
        ~isnumeric(raw_seeds) || ~isreal(raw_seeds) || ...
        ~isvector(raw_seeds) || isempty(raw_seeds) || ...
        any(~isfinite(raw_seeds)) || any(fix(raw_seeds) ~= raw_seeds)
    invalid_config();
end
population_size = double(raw_population_size);
evaluation_budget = double(raw_evaluation_budget);
seeds = double(raw_seeds(:));
if population_size < 4 || evaluation_budget < population_size || ...
        any(seeds < 0) || any(seeds > 2^32 - 3)
    invalid_config();
end
end

function valid = is_integer_scalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value) && fix(value) == value;
end

function starter = normalize_starter(value, space)
if ~isnumeric(value) || ~isreal(value) || ~isvector(value) || ...
        numel(value) ~= space.dimension || any(~isfinite(value))
    invalid_starter();
end
starter = double(value(:).');
if any(starter < space.lower_bounds) || any(starter > space.upper_bounds)
    invalid_starter();
end
end

function invalid_config()
error("twsbr:tuning:invalid_config", ...
    "Tuning config must define one shared valid budget and seed list.");
end

function invalid_starter()
error("twsbr:de:invalid_starter_vector", ...
    "Starter vector must be finite, bounded, and dimension-compatible.");
end
