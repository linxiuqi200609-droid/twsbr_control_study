function validation = run_simulink_validation_batch(vectors, plant_params, config)
%RUN_SIMULINK_VALIDATION_BATCH Validate the five frozen controller models.

validate_inputs(vectors, plant_params, config);
training = training_scenarios(3.2);
controller_names = config.controller_names(:);
row_count = numel(controller_names);
controller = strings(row_count, 1);
scenario_name = strings(row_count, 1);
max_tilt_difference_deg = nan(row_count, 1);
max_position_difference_m = nan(row_count, 1);
max_applied_input_difference = nan(row_count, 1);
tilt_tolerance_deg = nan(row_count, 1);
position_tolerance_m = nan(row_count, 1);
input_tolerance = nan(row_count, 1);
max_fuzzy_gain_relative_error = nan(row_count, 1);
fuzzy_gain_accepted = true(row_count, 1);
accepted = false(row_count, 1);

for index = 1:row_count
    name = controller_names(index);
    scenario = select_scenario(name, training);
    vector = vectors.(char(name));
    matlab_result = simulate_control_system( ...
        name, vector, plant_params, config, scenario, config.global_seed);
    simulink_result = run_controller_simulink( ...
        name, vector, plant_params, config, scenario);
    comparison = compare_matlab_simulink( ...
        matlab_result, simulink_result, config.plant_step);

    controller(index) = name;
    scenario_name(index) = string(scenario.name);
    max_tilt_difference_deg(index) = comparison.max_tilt_difference_deg;
    max_position_difference_m(index) = comparison.max_position_difference_m;
    max_applied_input_difference(index) = ...
        comparison.max_applied_input_difference;
    tilt_tolerance_deg(index) = comparison.tilt_tolerance_deg;
    position_tolerance_m(index) = comparison.position_tolerance_m;
    input_tolerance(index) = comparison.input_tolerance;
    max_fuzzy_gain_relative_error(index) = ...
        comparison.max_fuzzy_gain_relative_error;
    fuzzy_gain_accepted(index) = comparison.fuzzy_gain_accepted;
    accepted(index) = comparison.accepted;
end

validation = table(controller, scenario_name, max_tilt_difference_deg, ...
    max_position_difference_m, max_applied_input_difference, ...
    tilt_tolerance_deg, position_tolerance_m, input_tolerance, ...
    max_fuzzy_gain_relative_error, fuzzy_gain_accepted, accepted, ...
    'VariableNames', {'controller', 'scenario', ...
    'max_tilt_difference_deg', 'max_position_difference_m', ...
    'max_applied_input_difference', 'tilt_tolerance_deg', ...
    'position_tolerance_m', 'input_tolerance', ...
    'max_fuzzy_gain_relative_error', 'fuzzy_gain_accepted', 'accepted'});
end

function validate_inputs(vectors, plant_params, config)
required_names = ["ATTITUDE_PID"; "CASCADE_PID"; "FUZZY_PID"; "LQR"; "LQI"];
if ~isstruct(vectors) || ~isscalar(vectors) || ...
        ~all(isfield(vectors, required_names))
    error("twsbr:simulink_batch:invalid_vectors", ...
        "Frozen vectors must contain all five configured controller fields.");
end
if ~isstruct(plant_params) || ~isscalar(plant_params)
    error("twsbr:simulink_batch:invalid_plant", ...
        "Plant parameters must be a scalar structure.");
end
if ~isstruct(config) || ~isscalar(config) || ...
        ~isfield(config, "controller_names") || ...
        ~isequal(string(config.controller_names(:)), required_names) || ...
        ~isfield(config, "sample_time") || ~isfield(config, "plant_step") || ...
        ~isfield(config, "global_seed")
    error("twsbr:simulink_batch:invalid_config", ...
        "Configuration must contain the five names in their frozen order.");
end
end

function scenario = select_scenario(controller_name, training)
if controller_name == "ATTITUDE_PID"
    scenario = training.T1_initial_tilt_5deg;
else
    scenario = training.T2_position_step_0p5m;
end
end
