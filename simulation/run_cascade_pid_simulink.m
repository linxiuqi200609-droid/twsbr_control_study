function simulation = run_cascade_pid_simulink( ...
    plant_params, params, scenario, include_timing)
%RUN_CASCADE_PID_SIMULINK Run the generated cascade PID model.

if nargin < 1
    plant_params = twsbr_params();
else
    plant_params = twsbr_params(plant_params);
end
if nargin < 2
    params = cascade_pid_params(struct(), plant_params);
else
    params = cascade_pid_params(params, plant_params);
end
validate_cascade_pid_timing(params);
if nargin < 3
    scenarios = cascade_pid_scenarios();
    scenario = scenarios.positive_position_step;
end
if nargin < 4
    include_timing = false;
end
if ~islogical(include_timing) || ~isscalar(include_timing)
    error("twsbr:simulink:invalid_timing_option", ...
        "include_timing must be a logical scalar.");
end
validate_scenario(scenario, params);

source_time = (0:params.plant_step:scenario.duration).';
[initial_signals, initial_failure_reason] = inspect_initial_sample(scenario);
if initial_failure_reason ~= ""
    simulation = make_initial_failure( ...
        scenario, initial_failure_reason, params.plant_step);
    return
end
[position_reference_data, disturbance_force_data, ...
    disturbance_torque_data, first_nonfinite_signal_index] = ...
    make_source_data(source_time, scenario, initial_signals);
source_time = position_reference_data(:, 1);
simulation_duration = scenario.duration;
has_delayed_nonfinite_signal = first_nonfinite_signal_index > 0;
if has_delayed_nonfinite_signal
    simulation_duration = source_time(end);
end

project_paths = setup_project();
model_path = fullfile(project_paths.model_directory, "twsbr_cascade_pid.slx");
build_cascade_pid_simulink(plant_params, params);
model_name = "twsbr_cascade_pid";
load_system(model_path);
cleanup = onCleanup(@() close_model_if_loaded(model_name));

set_param(model_name + "/nonlinear_plant/state_integrator", ...
    "InitialCondition", mat2str(scenario.initial_state(:), 17));
set_param(model_name, "StopTime", sprintf("%.17g", simulation_duration));

controller_reset_data = [source_time, zeros(size(source_time))];
controller_reset_data(1, 2) = 1.0;
model_workspace = get_param(model_name, "ModelWorkspace");
assignin(model_workspace, "position_reference_data", position_reference_data);
assignin(model_workspace, "controller_reset_data", controller_reset_data);
assignin(model_workspace, "disturbance_force_data", disturbance_force_data);
assignin(model_workspace, "disturbance_torque_data", disturbance_torque_data);

set_param(model_name, "SimulationCommand", "update");
simulation_output = sim(model_name, "ReturnWorkspaceOutputs", "on");
[time, state] = unpack_log(simulation_output.get("state_log"), 4);
[position_reference_time, position_reference_values] = unpack_log( ...
    simulation_output.get("position_reference_log"), 1);
[position_error_time, position_error_values] = unpack_log( ...
    simulation_output.get("position_error_log"), 1);
[theta_reference_raw_time, theta_reference_raw_values] = unpack_log( ...
    simulation_output.get("theta_reference_raw_log"), 1);
[theta_reference_time, theta_reference_values] = unpack_log( ...
    simulation_output.get("theta_reference_log"), 1);
[theta_error_time, theta_error_values] = unpack_log( ...
    simulation_output.get("theta_error_log"), 1);
[u_raw_time, u_raw_values] = unpack_log( ...
    simulation_output.get("u_raw_log"), 1);
[u_time, u_values] = unpack_log(simulation_output.get("u_log"), 1);
[position_integral_time, position_integral_values] = unpack_log( ...
    simulation_output.get("position_integral_log"), 1);
[disturbance_force_time, disturbance_force_values] = unpack_log( ...
    simulation_output.get("disturbance_force_log"), 1);

position_reference = resample_held( ...
    position_reference_time, position_reference_values, time);
position_error = resample_held(position_error_time, position_error_values, time);
theta_reference_raw = resample_held( ...
    theta_reference_raw_time, theta_reference_raw_values, time);
theta_reference = resample_held( ...
    theta_reference_time, theta_reference_values, time);
theta_error = resample_held(theta_error_time, theta_error_values, time);
u_raw = resample_held(u_raw_time, u_raw_values, time);
u = resample_held(u_time, u_values, time);
position_integral = resample_held( ...
    position_integral_time, position_integral_values, time);
disturbance_force = resample_held( ...
    disturbance_force_time, disturbance_force_values, time);
disturbance_torque = resample_held( ...
    disturbance_torque_data(:, 1), disturbance_torque_data(:, 2), time);
saturated = abs(u_raw) > params.u_max;

[success, failure_reason, valid_count] = evaluate_trajectory( ...
    time, state, position_reference, position_error, ...
    theta_reference_raw, theta_reference, theta_error, u_raw, u, ...
    disturbance_force, disturbance_torque, position_integral, ...
    plant_params, params);
if has_delayed_nonfinite_signal && success
    success = false;
    failure_reason = "nonfinite_signal";
end
if valid_count == 0
    time = 0.0;
    state = zeros(1, 4);
    position_reference = 0.0;
    position_error = 0.0;
    theta_reference_raw = 0.0;
    theta_reference = 0.0;
    theta_error = 0.0;
    u_raw = 0.0;
    u = 0.0;
    disturbance_force = 0.0;
    position_integral = 0.0;
    saturated = false;
else
    time = time(1:valid_count);
    state = state(1:valid_count, :);
    position_reference = position_reference(1:valid_count);
    position_error = position_error(1:valid_count);
    theta_reference_raw = theta_reference_raw(1:valid_count);
    theta_reference = theta_reference(1:valid_count);
    theta_error = theta_error(1:valid_count);
    u_raw = u_raw(1:valid_count);
    u = u(1:valid_count);
    disturbance_force = disturbance_force(1:valid_count);
    position_integral = position_integral(1:valid_count);
    saturated = saturated(1:valid_count);
end

simulation = struct();
simulation.scenario_name = string(scenario.name);
simulation.time = time;
simulation.state = state;
simulation.position_reference = position_reference;
simulation.position_error = position_error;
simulation.theta_reference = theta_reference;
simulation.theta_reference_raw = theta_reference_raw;
simulation.theta_error = theta_error;
simulation.u_raw = u_raw;
simulation.u = u;
simulation.disturbance_force = disturbance_force;
simulation.position_integral = position_integral;
simulation.saturated = saturated;
simulation.success = success;
simulation.failure_reason = failure_reason;
simulation.metrics = calculate_metrics(simulation, scenario, params.plant_step);
if include_timing
    simulation.timing = struct( ...
        "u_raw_time", u_raw_time, "u_raw", u_raw_values, ...
        "u_time", u_time, "u", u_values);
end

clear cleanup;
close_system(model_name, 0);
end

function validate_scenario(scenario, params)
required_fields = ["name", "initial_state", "duration", "x_reference", ...
    "force_disturbance", "torque_disturbance", "reference_start", ...
    "disturbance_end"];
if ~isstruct(scenario) || ~isscalar(scenario) || ...
        ~all(isfield(scenario, required_fields))
    error("twsbr:simulink:invalid_scenario", ...
        "Scenario must be a scalar structure with all required fields.");
end
if ~isnumeric(scenario.initial_state) || ~isreal(scenario.initial_state) || ...
        ~isvector(scenario.initial_state) || numel(scenario.initial_state) ~= 4
    error("twsbr:simulink:invalid_scenario", ...
        "Initial state must contain four real numeric values.");
end
if ~is_finite_real_scalar(scenario.duration) || scenario.duration <= 0
    error("twsbr:simulink:invalid_scenario", ...
        "Scenario duration must be a positive finite scalar.");
end
step_count = round(scenario.duration / params.plant_step);
if abs(step_count * params.plant_step - scenario.duration) > 1e-12
    error("twsbr:simulink:invalid_duration", ...
        "Scenario duration must be an integer multiple of the plant step.");
end
event_times = [scenario.reference_start, scenario.disturbance_end];
if ~isnumeric(event_times) || ~isreal(event_times) || ...
        any(~isfinite(event_times)) || any(event_times < 0) || ...
        any(event_times > scenario.duration)
    error("twsbr:simulink:invalid_scenario", ...
        "Scenario event metadata must be finite and within its duration.");
end
end

function [values, failure_reason] = inspect_initial_sample(scenario)
values = [scalar_signal(scenario.x_reference, 0.0); ...
    scalar_signal(scenario.force_disturbance, 0.0); ...
    scalar_signal(scenario.torque_disturbance, 0.0)];
failure_reason = "";
if any(~isfinite(scenario.initial_state))
    failure_reason = "nonfinite_state";
elseif any(~isfinite(values))
    failure_reason = "nonfinite_signal";
end
end

function value = scalar_signal(signal_function, time)
if ~isa(signal_function, "function_handle")
    error("twsbr:simulink:invalid_scenario", ...
        "Scenario signals must be function handles.");
end
value = signal_function(time);
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value)
    error("twsbr:simulink:invalid_signal", ...
        "Scenario signals must return real numeric scalars.");
end
end

function [position_reference_data, disturbance_force_data, ...
    disturbance_torque_data, first_nonfinite_index] = ...
    make_source_data(time, scenario, initial_values)
values = zeros(numel(time), 3);
values(1, :) = initial_values(:).';
first_nonfinite_index = 0;
valid_count = numel(time);
for index = 2:numel(time)
    values(index, :) = [ ...
        scalar_signal(scenario.x_reference, time(index)); ...
        scalar_signal(scenario.force_disturbance, time(index)); ...
        scalar_signal(scenario.torque_disturbance, time(index))].';
    if any(~isfinite(values(index, :)))
        first_nonfinite_index = index;
        valid_count = index - 1;
        break
    end
end
time = time(1:valid_count);
values = values(1:valid_count, :);
position_reference_data = [time, values(:, 1)];
disturbance_force_data = [time, values(:, 2)];
disturbance_torque_data = [time, values(:, 3)];
end

function simulation = make_initial_failure(scenario, failure_reason, plant_step)
simulation = struct();
simulation.scenario_name = string(scenario.name);
simulation.time = 0.0;
simulation.state = zeros(1, 4);
simulation.position_reference = 0.0;
simulation.position_error = 0.0;
simulation.theta_reference = 0.0;
simulation.theta_reference_raw = 0.0;
simulation.theta_error = 0.0;
simulation.u_raw = 0.0;
simulation.u = 0.0;
simulation.disturbance_force = 0.0;
simulation.position_integral = 0.0;
simulation.saturated = false;
simulation.success = false;
simulation.failure_reason = failure_reason;
simulation.metrics = calculate_metrics(simulation, scenario, plant_step);
end

function [time, values] = unpack_log(log_data, signal_width)
time = log_data.time(:);
values = squeeze(log_data.signals.values);
if signal_width == 1
    values = values(:);
elseif size(values, 1) == signal_width && size(values, 2) == numel(time)
    values = values.';
elseif size(values, 2) ~= signal_width || size(values, 1) ~= numel(time)
    error("twsbr:simulink:invalid_log_shape", ...
        "Logged signal has an unexpected shape.");
end
end

function aligned_values = resample_held(source_time, source_values, target_time)
if isscalar(source_time)
    aligned_values = repmat(source_values(1, :), numel(target_time), 1);
else
    aligned_values = interp1(source_time, source_values, ...
        target_time, "previous", "extrap");
end
end

function [success, reason, valid_count] = evaluate_trajectory( ...
    time, state, position_reference, position_error, ...
    theta_reference_raw, theta_reference, theta_error, u_raw, u, ...
    disturbance_force, disturbance_torque, position_integral, ...
    plant_params, params)
success = true;
reason = "";
valid_count = numel(time);
for index = 1:valid_count
    control_values = [position_error(index), theta_reference_raw(index), ...
        theta_reference(index), theta_error(index), u_raw(index), u(index), ...
        position_integral(index)];
    if ~isfinite(time(index)) || any(~isfinite(state(index, :)))
        success = false;
        reason = "nonfinite_state";
        valid_count = index - 1;
        return
    elseif any(~isfinite([position_reference(index), ...
            disturbance_force(index), disturbance_torque(index)]))
        success = false;
        reason = "nonfinite_signal";
        valid_count = index - 1;
        return
    elseif any(~isfinite(control_values))
        success = false;
        reason = "nonfinite_control";
        valid_count = index - 1;
        return
    elseif abs(state(index, 3)) > deg2rad(30.0)
        success = false;
        reason = "tilt_limit";
    elseif abs(state(index, 1)) > plant_params.x_limit
        success = false;
        reason = "position_limit";
    elseif abs(theta_reference(index)) > ...
            min(params.theta_reference_limit, deg2rad(12.0))
        success = false;
        reason = "theta_reference_limit";
    elseif abs(u(index)) > min(params.u_max, plant_params.u_max)
        success = false;
        reason = "actuator_limit";
    else
        continue
    end
    valid_count = index;
    return
end
end

function metrics = calculate_metrics(simulation, scenario, plant_step)
actual_position_error = simulation.position_reference - simulation.state(:, 1);
actual_tilt_error = simulation.state(:, 3) - simulation.theta_reference;

metrics = struct();
metrics.max_abs_tilt_deg = finite_rad2deg(max(abs(simulation.state(:, 3))));
metrics.final_tilt_deg = finite_rad2deg(simulation.state(end, 3));
metrics.final_abs_tilt_deg = abs(metrics.final_tilt_deg);
metrics.max_abs_position = max(abs(simulation.state(:, 1)));
metrics.max_abs_position_error = max(abs(actual_position_error));
metrics.final_position_error = actual_position_error(end);
metrics.final_abs_position_error = abs(metrics.final_position_error);
metrics.position_settling_time = event_settling_time( ...
    simulation.time, actual_position_error, scenario.reference_start, ...
    0.05, scenario.duration, plant_step);
metrics.disturbance_recovery_time = event_settling_time( ...
    simulation.time, actual_position_error, scenario.disturbance_end, ...
    0.10, scenario.duration, plant_step);
if numel(simulation.time) < 2
    metrics.position_iae = 0.0;
    metrics.tilt_iae = 0.0;
else
    metrics.position_iae = trapz(simulation.time, abs(actual_position_error));
    metrics.tilt_iae = trapz(simulation.time, abs(actual_tilt_error));
end
metrics.max_abs_u_raw = max(abs(simulation.u_raw));
metrics.max_abs_u = max(abs(simulation.u));
metrics.max_abs_theta_reference_deg = ...
    finite_rad2deg(max(abs(simulation.theta_reference)));
metrics.max_abs_position_integral = max(abs(simulation.position_integral));
metrics.saturation_duration = sum(diff(simulation.time) .* ...
    double(simulation.saturated(1:end - 1)));
end

function degrees = finite_rad2deg(radians)
degrees = rad2deg(radians);
overflow = ~isfinite(degrees) & isfinite(radians);
degrees(overflow & radians >= 0) = realmax;
degrees(overflow & radians < 0) = -realmax;
end

function elapsed_time = event_settling_time( ...
    time, error, event_time, tolerance, scenario_duration, plant_step)
if event_time == 0.0
    elapsed_time = 0.0;
    return
end
indices = find(time >= event_time - 1e-12);
elapsed_time = max(0.0, scenario_duration - event_time) + plant_step;
remains_settled = true;
for offset = numel(indices):-1:1
    index = indices(offset);
    remains_settled = remains_settled && isfinite(error(index)) && ...
        abs(error(index)) <= tolerance;
    if remains_settled
        elapsed_time = time(index) - event_time;
    end
end
end

function valid = is_finite_real_scalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end

function close_model_if_loaded(model_name)
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
end
