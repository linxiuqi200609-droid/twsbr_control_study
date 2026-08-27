function frozen_vectors = select_frozen_parameters(runs)
%SELECT_FROZEN_PARAMETERS Select each controller using training cost only.

[controllers, seeds, costs, vectors] = validate_runs(runs);
controller_order = unique(controllers, "stable");
frozen_vectors = struct();
for index = 1:numel(controller_order)
    controller = controller_order(index);
    matching = find(controllers == controller);
    ranking = table(costs(matching), seeds(matching), matching, ...
        'VariableNames', {'training_cost', 'seed', 'row'});
    ranking = sortrows(ranking, {'training_cost', 'seed', 'row'}, ...
        {'ascend', 'ascend', 'ascend'});
    selected_row = ranking.row(1);
    frozen_vectors.(char(controller)) = vectors{selected_row};
end
end

function [controllers, seeds, costs, vectors] = validate_runs(runs)
required = {'controller', 'seed', 'training_cost', 'vector'};
if ~istable(runs) || height(runs) == 0 || ...
        ~all(ismember(required, runs.Properties.VariableNames))
    invalid_runs();
end
try
    controllers = string(runs.controller);
catch
    invalid_runs();
end
if ~isequal(size(controllers), [height(runs), 1]) || ...
        any(ismissing(controllers)) || any(strlength(controllers) == 0)
    invalid_runs();
end
for index = 1:numel(controllers)
    if ~isvarname(char(controllers(index)))
        invalid_runs();
    end
end
seeds = runs.seed;
costs = runs.training_cost;
vectors = runs.vector;
if ~isnumeric(seeds) || ~isreal(seeds) || ...
        ~isequal(size(seeds), [height(runs), 1]) || ...
        any(~isfinite(seeds)) || any(seeds < 0) || ...
        any(fix(seeds) ~= seeds) || ...
        ~isnumeric(costs) || ~isreal(costs) || ...
        ~isequal(size(costs), [height(runs), 1]) || ...
        any(~isfinite(costs)) || ...
        ~iscell(vectors) || ~isequal(size(vectors), [height(runs), 1])
    invalid_runs();
end
seeds = double(seeds);
costs = double(costs);
for index = 1:numel(vectors)
    value = vectors{index};
    if ~isnumeric(value) || ~isreal(value) || ~isrow(value) || ...
            isempty(value) || any(~isfinite(value))
        invalid_runs();
    end
    vectors{index} = double(value);
end
end

function invalid_runs()
error("twsbr:tuning:invalid_runs", ...
    "Tuning runs must contain finite training-only selection data.");
end
