function simulation = run_controller_simulink( ...
        controller_name, vector, plant_params, config, scenario)
%RUN_CONTROLLER_SIMULINK Run one frozen controller in its Simulink model.

controller_name = validate_inputs( ...
    controller_name, vector, plant_params, config, scenario);
params = decode_controller_vector(controller_name, vector, plant_params, config);

switch controller_name
    case "ATTITUDE_PID"
        simulation = normalize_legacy(run_attitude_pid_simulink( ...
            plant_params, params, adapt_attitude_scenario(scenario)), ...
            controller_name, scenario, config);
    case "CASCADE_PID"
        simulation = normalize_legacy(run_cascade_pid_simulink( ...
            plant_params, params, scenario), controller_name, scenario, config);
    case {"FUZZY_PID", "LQR", "LQI"}
        simulation = run_generated_model( ...
            controller_name, plant_params, params, config, scenario);
    otherwise
        error("twsbr:simulink_runner:unsupported_controller", ...
            "Unsupported controller: %s", controller_name);
end
end

function controller_name = validate_inputs( ...
        controller_name, vector, plant_params, config, scenario)
if ~(ischar(controller_name) && isrow(controller_name)) && ...
        ~(isstring(controller_name) && isscalar(controller_name) && ...
        ~ismissing(controller_name))
    error("twsbr:simulink_runner:invalid_controller", ...
        "Controller name must be a character row or string scalar.");
end
controller_name = string(controller_name);
supported_names = ["ATTITUDE_PID"; "CASCADE_PID"; "FUZZY_PID"; "LQR"; "LQI"];
if ~any(controller_name == supported_names)
    error("twsbr:simulink_runner:unsupported_controller", ...
        "Unsupported controller: %s", controller_name);
end
if ~isnumeric(vector) || ~isreal(vector) || ~isvector(vector) || ...
        any(~isfinite(vector))
    error("twsbr:simulink_runner:invalid_vector", ...
        "Controller vector must contain finite real numeric values.");
end
if ~isstruct(plant_params) || ~isscalar(plant_params)
    error("twsbr:simulink_runner:invalid_plant", ...
        "Plant parameters must be a scalar structure.");
end
required_config = ["sample_time"; "plant_step"; "controller_names"];
if ~isstruct(config) || ~isscalar(config) || ...
        ~all(isfield(config, required_config)) || ...
        ~is_positive_finite_scalar(config.sample_time) || ...
        ~is_positive_finite_scalar(config.plant_step) || ...
        abs(config.sample_time - 0.01) > 1e-15 || ...
        abs(config.plant_step - 0.001) > 1e-15
    error("twsbr:simulink_runner:invalid_config", ...
        "Configuration must preserve the 0.01 s and 0.001 s timing contract.");
end
required_scenario = ["name"; "initial_state"; "duration"; "x_reference"; ...
    "force_disturbance"; "torque_disturbance"; "measurement_noise_std"];
if ~isstruct(scenario) || ~isscalar(scenario) || ...
        ~all(isfield(scenario, required_scenario)) || ...
        ~isnumeric(scenario.initial_state) || ~isreal(scenario.initial_state) || ...
        ~isequal(size(scenario.initial_state), [4, 1]) || ...
        any(~isfinite(scenario.initial_state)) || ...
        ~is_positive_finite_scalar(scenario.duration) || ...
        ~isa(scenario.x_reference, "function_handle") || ...
        ~isa(scenario.force_disturbance, "function_handle") || ...
        ~isa(scenario.torque_disturbance, "function_handle")
    error("twsbr:simulink_runner:invalid_scenario", ...
        "Scenario must satisfy the common simulation contract.");
end
noise = scenario.measurement_noise_std;
if ~isnumeric(noise) || ~isreal(noise) || ...
        ~isequal(size(noise), [4, 1]) || any(~isfinite(noise)) || ...
        any(noise < 0) || any(noise ~= 0)
    error("twsbr:simulink_runner:invalid_noise", ...
        "Simulink validation requires zero finite 4-by-1 measurement noise.");
end
step_count = round(scenario.duration / config.plant_step);
if abs(step_count * config.plant_step - scenario.duration) > 1e-12
    error("twsbr:simulink_runner:invalid_duration", ...
        "Scenario duration must be an integer multiple of the plant step.");
end
validate_scenario_signals(scenario, config.plant_step);
end

function scenario = adapt_attitude_scenario(generic_scenario)
scenario = struct( ...
    "name", generic_scenario.name, ...
    "initial_state", generic_scenario.initial_state, ...
    "duration", generic_scenario.duration, ...
    "theta_reference", @(~) 0.0, ...
    "force_disturbance", generic_scenario.force_disturbance, ...
    "torque_disturbance", generic_scenario.torque_disturbance);
end

function simulation = normalize_legacy(simulation, controller_name, scenario, config)
validate_trajectory_time(simulation.time, scenario.duration, "legacy result");
plant_time = (0:config.plant_step:scenario.duration).';
simulation.state = interpolate_pchip( ...
    simulation.time, simulation.state, plant_time, "legacy state");
simulation.u_raw = resample_held( ...
    simulation.time, simulation.u_raw, plant_time, "legacy raw control");
simulation.u = resample_held( ...
    simulation.time, simulation.u, plant_time, "legacy applied control");
simulation.time = plant_time;
simulation.controller_name = controller_name;
simulation.scenario_name = string(scenario.name);
simulation.diagnostics = repmat(struct(), numel(simulation.time), 1);
simulation.timing = make_timing_metadata( ...
    controller_schedule(scenario.duration, config.sample_time), config.sample_time, []);
simulation.survived_time = simulation.time(end);
if simulation.success
    simulation.survived_time = scenario.duration;
end
end

function simulation = run_generated_model( ...
        controller_name, plant_params, params, config, scenario)
model_name = model_for_controller(controller_name);
model_path = build_model(controller_name, plant_params, params);
load_system(model_path);
cleanup = onCleanup(@() close_model_if_loaded(model_name));

set_param(model_name + "/nonlinear_plant/state_integrator", ...
    "InitialCondition", mat2str(scenario.initial_state(:), 17));
source_time = (0:config.plant_step:scenario.duration).';
[position_reference_data, disturbance_force_data, disturbance_torque_data] = ...
    make_source_data(source_time, scenario);
workspace = get_param(model_name, "ModelWorkspace");
assignin(workspace, "position_reference_data", position_reference_data);
assignin(workspace, "disturbance_force_data", disturbance_force_data);
assignin(workspace, "disturbance_torque_data", disturbance_torque_data);
if controller_name == "FUZZY_PID" || controller_name == "LQI"
    reset_data = [source_time, zeros(size(source_time))];
    reset_data(1, 2) = 1.0;
    assignin(workspace, "controller_reset_data", reset_data);
end

set_param(model_name, "SimulationCommand", "update");
simulation_input = Simulink.SimulationInput(model_name);
simulation_input = simulation_input.setModelParameter( ...
    "StopTime", sprintf("%.17g", scenario.duration));
simulation_output = sim(simulation_input);
[state_time, state] = unpack_log( ...
    simulation_output.get("state_log"), 4, "state_log");
[position_reference_time, position_reference] = unpack_log( ...
    simulation_output.get("position_reference_log"), 1, "position_reference_log");
[u_raw_time, u_raw] = unpack_log( ...
    simulation_output.get("u_raw_log"), 1, "u_raw_log");
[u_time, u] = unpack_log(simulation_output.get("u_log"), 1, "u_log");
[force_time, force] = unpack_log( ...
    simulation_output.get("disturbance_force_log"), 1, "disturbance_force_log");
validate_trajectory_time(state_time, scenario.duration, "state log");
validate_controller_log_time(u_raw_time, scenario.duration, config.sample_time, ...
    "raw control log");
validate_controller_log_time(u_time, scenario.duration, config.sample_time, ...
    "applied control log");

simulation = struct();
simulation.controller_name = controller_name;
simulation.scenario_name = string(scenario.name);
simulation.time = source_time;
simulation.state = interpolate_pchip( ...
    state_time, state, source_time, "state log");
simulation.position_reference = resample_held( ...
    position_reference_time, position_reference, source_time, "position reference log");
simulation.u_raw = resample_held( ...
    u_raw_time, u_raw, source_time, "raw control log");
simulation.u = resample_held(u_time, u, source_time, "applied control log");
simulation.force_disturbance = resample_held( ...
    force_time, force, source_time, "force disturbance log");
simulation.torque_disturbance = evaluate_signal( ...
    scenario.torque_disturbance, source_time, "torque_disturbance");
simulation.saturated = abs(simulation.u_raw) > plant_params.u_max + 1e-12;
[simulation.diagnostics, fuzzy_gain_time] = diagnostics_for_controller( ...
    controller_name, simulation_output, source_time, scenario.duration, ...
    config.sample_time);
simulation.timing = make_timing_metadata( ...
    u_raw_time, config.sample_time, fuzzy_gain_time);
[simulation.success, simulation.failure_reason] = evaluate_success( ...
    simulation.state, plant_params);
simulation.survived_time = source_time(end);
if simulation.success
    simulation.survived_time = scenario.duration;
end

clear cleanup;
close_system(model_name, 0);
end

function model_path = build_model(controller_name, plant_params, params)
switch controller_name
    case "FUZZY_PID"
        model_path = build_fuzzy_pid_simulink(plant_params, params);
    case "LQR"
        model_path = build_lqr_simulink(plant_params, params);
    case "LQI"
        model_path = build_lqi_simulink(plant_params, params);
    otherwise
        error("twsbr:simulink_runner:unsupported_controller", ...
            "Unsupported generated controller: %s", controller_name);
end
end

function model_name = model_for_controller(controller_name)
switch controller_name
    case "FUZZY_PID"
        model_name = "twsbr_fuzzy_pid";
    case "LQR"
        model_name = "twsbr_lqr";
    case "LQI"
        model_name = "twsbr_lqi";
    otherwise
        error("twsbr:simulink_runner:unsupported_controller", ...
            "Unsupported generated controller: %s", controller_name);
end
end

function [reference_data, force_data, torque_data] = make_source_data(time, scenario)
reference_values = evaluate_signal( ...
    scenario.x_reference, time, "x_reference");
force_values = evaluate_signal( ...
    scenario.force_disturbance, time, "force_disturbance");
torque_values = evaluate_signal( ...
    scenario.torque_disturbance, time, "torque_disturbance");
reference_data = [time, reference_values(:)];
force_data = [time, force_values(:)];
torque_data = [time, torque_values(:)];
end

function [diagnostics, fuzzy_gain_time] = diagnostics_for_controller( ...
        controller_name, simulation_output, time, duration, sample_time)
if controller_name ~= "FUZZY_PID"
    diagnostics = repmat(struct(), numel(time), 1);
    fuzzy_gain_time = [];
    return
end
[kp_time, kp] = unpack_log( ...
    simulation_output.get("kp_theta_log"), 1, "kp_theta_log");
[ki_time, ki] = unpack_log( ...
    simulation_output.get("ki_theta_log"), 1, "ki_theta_log");
[kd_time, kd] = unpack_log( ...
    simulation_output.get("kd_theta_log"), 1, "kd_theta_log");
validate_controller_log_time(kp_time, duration, sample_time, "kp_theta log");
validate_controller_log_time(ki_time, duration, sample_time, "ki_theta log");
validate_controller_log_time(kd_time, duration, sample_time, "kd_theta log");
kp = resample_held(kp_time, kp, time, "kp_theta log");
ki = resample_held(ki_time, ki, time, "ki_theta log");
kd = resample_held(kd_time, kd, time, "kd_theta log");
fuzzy_gain_time = kp_time;
diagnostics = repmat(struct("kp_theta", 0.0, "ki_theta", 0.0, ...
    "kd_theta", 0.0), numel(time), 1);
for index = 1:numel(time)
    diagnostics(index).kp_theta = kp(index);
    diagnostics(index).ki_theta = ki(index);
    diagnostics(index).kd_theta = kd(index);
end
end

function [time, values] = unpack_log(log_data, width, log_name)
if ~isstruct(log_data) || ~isfield(log_data, "time") || ...
        ~isfield(log_data, "signals") || ~isfield(log_data.signals, "values")
    error("twsbr:simulink_runner:invalid_log", ...
        "%s must be a Simulink structure-with-time log.", log_name);
end
time = log_data.time(:);
values = squeeze(log_data.signals.values);
if width == 1
    values = values(:);
elseif size(values, 1) == width && size(values, 2) == numel(time)
    values = values.';
elseif ~isequal(size(values), [numel(time), width])
    error("twsbr:simulink_runner:invalid_log", ...
        "Logged signal has an unexpected shape.");
end
if ~isnumeric(time) || ~isreal(time) || isempty(time) || ...
        any(~isfinite(time)) || ~isnumeric(values) || ~isreal(values) || ...
        any(~isfinite(values), "all") || any(diff(time) < 0)
    error("twsbr:simulink_runner:invalid_log", ...
        "%s must have finite monotone time and values.", log_name);
end
if any(diff(time) == 0)
    [time, keep_indices] = unique(time, "last");
    values = values(keep_indices, :);
end
if numel(time) < 2 || any(diff(time) <= 0)
    error("twsbr:simulink_runner:invalid_log", ...
        "%s must retain at least two strictly increasing samples.", log_name);
end
end

function values = resample_held(source_time, values, target_time, log_name)
if target_time(1) < source_time(1) - 1e-12 || ...
        target_time(end) > source_time(end) + 1e-12
    error("twsbr:simulink_runner:invalid_log", ...
        "%s does not cover the requested plant-step grid.", log_name);
end
values = interp1(source_time, values, target_time, "previous");
end

function values = interpolate_pchip(source_time, values, target_time, log_name)
if target_time(1) < source_time(1) - 1e-12 || ...
        target_time(end) > source_time(end) + 1e-12
    error("twsbr:simulink_runner:invalid_log", ...
        "%s does not cover the requested plant-step grid.", log_name);
end
values = interp1(source_time, values, target_time, "pchip");
end

function validate_scenario_signals(scenario, plant_step)
time = (0:plant_step:scenario.duration).';
evaluate_signal(scenario.x_reference, time, "x_reference");
evaluate_signal(scenario.force_disturbance, time, "force_disturbance");
evaluate_signal(scenario.torque_disturbance, time, "torque_disturbance");
end

function values = evaluate_signal(signal_function, time, signal_name)
values = zeros(numel(time), 1);
for index = 1:numel(time)
    try
        value = signal_function(time(index));
    catch
        error("twsbr:simulink_runner:invalid_signal", ...
            "%s must return finite real numeric scalars.", signal_name);
    end
    if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ...
            ~isfinite(value)
        error("twsbr:simulink_runner:invalid_signal", ...
            "%s must return finite real numeric scalars.", signal_name);
    end
    values(index) = value;
end
end

function validate_trajectory_time(time, duration, log_name)
if ~isnumeric(time) || ~isreal(time) || ~iscolumn(time) || ...
        numel(time) < 2 || any(~isfinite(time)) || any(diff(time) <= 0) || ...
        abs(time(1)) > 1e-12 || time(end) < duration - 1e-12
    error("twsbr:simulink_runner:invalid_log", ...
        "%s must cover the simulation interval with monotone time.", log_name);
end
end

function validate_controller_log_time(time, duration, sample_time, log_name)
validate_trajectory_time(time, duration, log_name);
if any(abs(diff(time) - sample_time) > 1e-12)
    error("twsbr:simulink_runner:invalid_timing", ...
        "%s must use the common 0.01 second controller schedule.", log_name);
end
end

function timing = make_timing_metadata(update_time, sample_time, fuzzy_gain_time)
timing = struct( ...
    "u_raw_time", update_time, ...
    "u_time", update_time, ...
    "controller_update_time", update_time, ...
    "fuzzy_gain_time", fuzzy_gain_time, ...
    "sample_time", sample_time, ...
    "validated", true);
end

function time = controller_schedule(duration, sample_time)
time = (0:sample_time:duration).';
end

function [success, failure_reason] = evaluate_success(state, plant_params)
success = true;
failure_reason = "";
if any(~isfinite(state), "all")
    success = false;
    failure_reason = "nonfinite_state";
elseif any(abs(state(:, 3)) > deg2rad(plant_params.theta_fail_deg))
    success = false;
    failure_reason = "tilt_limit";
elseif any(abs(state(:, 1)) > plant_params.x_limit)
    success = false;
    failure_reason = "position_limit";
end
end

function close_model_if_loaded(model_name)
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
end

function valid = is_positive_finite_scalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value) && value > 0;
end
