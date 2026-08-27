function runs = tune_all_controllers(plant_params, config, objective, ...
        scenarios, starter_vectors)
%TUNE_ALL_CONTROLLERS Tune all configured controllers in fixed order.

if nargin < 4
    error("twsbr:tuning:invalid_input", ...
        "All-controller tuning requires plant, config, objective, and scenarios.");
end
names = validate_controller_names(config);
has_starters = nargin == 5;
if has_starters
    validate_starters(starter_vectors, names);
end

runs = table(strings(0, 1), zeros(0, 1), zeros(0, 1), cell(0, 1), ...
    zeros(0, 1), zeros(0, 1), 'VariableNames', ...
    {'controller', 'seed', 'training_cost', 'vector', ...
    'evaluation_count', 'elapsed_seconds'});
for index = 1:numel(names)
    name = names(index);
    if has_starters
        controller_runs = tune_controller(name, plant_params, config, ...
            objective, scenarios, starter_vectors.(char(name)));
    else
        controller_runs = tune_controller(name, plant_params, config, ...
            objective, scenarios);
    end
    runs = [runs; controller_runs]; %#ok<AGROW>
end
end

function names = validate_controller_names(config)
expected = ["ATTITUDE_PID"; "CASCADE_PID"; "FUZZY_PID"; "LQR"; "LQI"];
if ~isstruct(config) || ~isscalar(config) || ...
        ~isfield(config, "controller_names") || ...
        ~isstring(config.controller_names) || ...
        ~isequal(config.controller_names, expected)
    error("twsbr:tuning:invalid_config", ...
        "Controller names must use the fixed five-controller study order.");
end
names = expected;
end

function validate_starters(starters, names)
if ~isstruct(starters) || ~isscalar(starters) || ...
        numel(fieldnames(starters)) ~= numel(names) || ...
        ~all(isfield(starters, cellstr(names)))
    error("twsbr:tuning:invalid_starters", ...
        "Starter vectors must cover every configured controller.");
end
for index = 1:numel(names)
    space = controller_parameter_space(names(index));
    value = starters.(char(names(index)));
    if ~isnumeric(value) || ~isreal(value) || ~isvector(value) || ...
            numel(value) ~= space.dimension || any(~isfinite(value))
        error("twsbr:tuning:invalid_starters", ...
            "Starter vectors must be finite vectors with matching dimensions.");
    end
    normalized = double(value(:).');
    if any(normalized < space.lower_bounds) || ...
            any(normalized > space.upper_bounds)
        error("twsbr:tuning:invalid_starters", ...
            "Starter vectors must remain within parameter bounds.");
    end
end
end
