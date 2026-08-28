function [metrics_table, raw_results] = run_deterministic_batch( ...
        vectors, plant_params, config, scenarios, seed)
%RUN_DETERMINISTIC_BATCH Evaluate every controller on every held-out case.

controller_names = validate_common_inputs(vectors, config, seed);
scenario_fields = validate_scenarios(scenarios);
row_count = numel(controller_names) * numel(scenario_fields);
metric_rows = cell(row_count, 1);
raw_results = struct();
row_index = 0;

for controller_index = 1:numel(controller_names)
    controller_name = controller_names(controller_index);
    vector = vectors.(char(controller_name));
    for scenario_index = 1:numel(scenario_fields)
        scenario = scenarios.(scenario_fields{scenario_index});
        simulation = simulate_control_system(controller_name, vector, ...
            plant_params, config, scenario, seed);
        metrics = calculate_control_metrics( ...
            simulation, scenario, plant_params);

        row_index = row_index + 1;
        metric_rows{row_index} = metrics;
        raw_key = matlab.lang.makeValidName(char( ...
            controller_name + "__" + string(scenario.name)));
        if isfield(raw_results, raw_key)
            error("twsbr:batch:duplicate_raw_key", ...
                "Controller and scenario names must form unique raw keys.");
        end
        raw_results.(raw_key) = simulation;
    end
end

metrics_table = struct2table(vertcat(metric_rows{:}), "AsArray", true);
end

function controller_names = validate_common_inputs(vectors, config, seed)
if ~isstruct(vectors) || ~isscalar(vectors) || ...
        ~isstruct(config) || ~isscalar(config) || ...
        ~isfield(config, "controller_names") || ...
        ~isstring(config.controller_names) || ...
        isempty(config.controller_names) || ...
        ~isnumeric(seed) || ~isreal(seed) || ~isscalar(seed) || ...
        ~isfinite(seed) || seed < 0 || seed > 2^32 - 1 || ...
        seed ~= floor(seed)
    invalid_input();
end
controller_names = config.controller_names(:);
if numel(unique(controller_names)) ~= numel(controller_names) || ...
        ~all(isfield(vectors, cellstr(controller_names)))
    invalid_input();
end
end

function scenario_fields = validate_scenarios(scenarios)
if ~isstruct(scenarios) || ~isscalar(scenarios)
    invalid_input();
end
scenario_fields = fieldnames(scenarios);
if isempty(scenario_fields)
    invalid_input();
end
for index = 1:numel(scenario_fields)
    scenario = scenarios.(scenario_fields{index});
    if ~isstruct(scenario) || ~isscalar(scenario) || ...
            ~isfield(scenario, "name") || ~isfield(scenario, "split") || ...
            string(scenario.split) ~= "test"
        invalid_input();
    end
end
end

function invalid_input()
error("twsbr:batch:invalid_input", ...
    "Batch inputs must define frozen controllers and held-out scenarios.");
end
