function simulation = run_attitude_pid_simulink( ...
    plant_params, pid_params, scenario)
%RUN_ATTITUDE_PID_SIMULINK Run the generated attitude PID model.

if nargin < 1
    plant_params = twsbr_params();
else
    plant_params = twsbr_params(plant_params);
end
if nargin < 2
    pid_params = attitude_pid_params(struct(), plant_params);
else
    pid_params = attitude_pid_params(pid_params, plant_params);
end
if nargin < 3
    scenarios = attitude_pid_scenarios();
    scenario = scenarios.positive_tilt;
end
validate_scenario(scenario);

model_path = build_attitude_pid_simulink(plant_params, pid_params);
model_name = "twsbr_attitude_pid";
load_system(model_path);
cleanup = onCleanup(@() close_model_if_loaded(model_name));

set_param(model_name + "/nonlinear_plant/state_integrator", ...
    "InitialCondition", mat2str(scenario.initial_state(:), 17));
set_param(model_name, "StopTime", sprintf("%.17g", scenario.duration));

source_time = (0:pid_params.plant_step:scenario.duration).';
theta_reference_data = make_source_data( ...
    source_time, scenario.theta_reference);
force_disturbance_data = make_source_data( ...
    source_time, scenario.force_disturbance);
torque_disturbance_data = make_source_data( ...
    source_time, scenario.torque_disturbance);
model_workspace = get_param(model_name, "ModelWorkspace");
assignin(model_workspace, "theta_reference_data", theta_reference_data);
assignin(model_workspace, "force_disturbance_data", force_disturbance_data);
assignin(model_workspace, "torque_disturbance_data", torque_disturbance_data);

set_param(model_name, "SimulationCommand", "update");
simulation_output = sim(model_name, "ReturnWorkspaceOutputs", "on");
[time, state] = unpack_log(simulation_output.get("state_log"), 4);
[u_raw_time, u_raw_values] = unpack_log( ...
    simulation_output.get("u_raw_log"), 1);
[u_time, u_values] = unpack_log(simulation_output.get("u_log"), 1);
[reference_time, reference_values] = unpack_log( ...
    simulation_output.get("theta_reference_log"), 1);
[integral_time, integral_values] = unpack_log( ...
    simulation_output.get("integral_error_log"), 1);

u_raw = resample_held(u_raw_time, u_raw_values, time);
u = resample_held(u_time, u_values, time);
theta_reference = resample_held( ...
    reference_time, reference_values, time);
integral_error = resample_held( ...
    integral_time, integral_values, time);
force_disturbance = arrayfun(scenario.force_disturbance, time);
torque_disturbance = arrayfun(scenario.torque_disturbance, time);
saturated = abs(u_raw) > pid_params.u_max + 1e-12;

simulation = struct();
simulation.scenario_name = string(scenario.name);
simulation.time = time;
simulation.state = state;
simulation.theta_reference = theta_reference;
simulation.u_raw = u_raw;
simulation.u = u;
simulation.saturated = saturated;
simulation.integral_error = integral_error;
simulation.force_disturbance = force_disturbance;
simulation.torque_disturbance = torque_disturbance;
[simulation.success, simulation.failure_reason] = ...
    evaluate_success(state, plant_params);
simulation.metrics = calculate_metrics(simulation);

clear cleanup;
close_system(model_name, 0);
end

function validate_scenario(scenario)
required_fields = {"name", "initial_state", "duration", ...
    "theta_reference", "force_disturbance", "torque_disturbance"};
if ~isstruct(scenario) || ~isscalar(scenario)
    error("twsbr:simulink:invalid_scenario", ...
        "Scenario must be a scalar structure.");
end
for index = 1:numel(required_fields)
    if ~isfield(scenario, required_fields{index})
        error("twsbr:simulink:invalid_scenario", ...
            "Scenario is missing field: %s", required_fields{index});
    end
end
if ~isnumeric(scenario.initial_state) || numel(scenario.initial_state) ~= 4 || ...
        any(~isfinite(scenario.initial_state), "all")
    error("twsbr:simulink:invalid_scenario", ...
        "Initial state must contain four finite numeric values.");
end
end

function data = make_source_data(time, signal_function)
if ~isa(signal_function, "function_handle")
    error("twsbr:simulink:invalid_scenario", ...
        "Scenario signals must be function handles.");
end
values = arrayfun(signal_function, time);
if any(~isfinite(values))
    error("twsbr:simulink:invalid_signal", ...
        "Scenario signals must return finite numeric scalars.");
end
data = [time, values(:)];
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

function [success, reason] = evaluate_success(state, plant_params)
success = true;
reason = "";
if any(~isfinite(state), "all")
    success = false;
    reason = "nonfinite_state";
elseif any(abs(state(:, 3)) > deg2rad(plant_params.theta_fail_deg))
    success = false;
    reason = "tilt_limit";
end
end

function metrics = calculate_metrics(simulation)
tilt_error = simulation.state(:, 3) - simulation.theta_reference;
absolute_tilt_error = abs(tilt_error);
settling_tolerance = deg2rad(0.5);
settling_time = inf;
remains_settled = true;
for index = numel(absolute_tilt_error):-1:1
    remains_settled = remains_settled && ...
        absolute_tilt_error(index) <= settling_tolerance;
    if remains_settled
        settling_time = simulation.time(index);
    end
end
metrics = struct();
metrics.max_abs_tilt_deg = rad2deg(max(absolute_tilt_error));
metrics.final_abs_tilt_deg = rad2deg(absolute_tilt_error(end));
metrics.settling_time = settling_time;
metrics.integral_abs_tilt_error = trapz(simulation.time, absolute_tilt_error);
metrics.max_abs_u_raw = max(abs(simulation.u_raw));
metrics.saturation_duration = sum( ...
    diff(simulation.time) .* double(simulation.saturated(1:end - 1)));
metrics.max_abs_position = max(abs(simulation.state(:, 1)));
end

function close_model_if_loaded(model_name)
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
end
