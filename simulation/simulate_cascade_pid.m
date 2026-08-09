function simulation = simulate_cascade_pid(plant_params, params, scenario)
%SIMULATE_CASCADE_PID Simulate the nonlinear plant with a cascade PID.

validate_inputs(plant_params, params, scenario);

step_size = params.plant_step;
step_ratio = round(params.sample_time / step_size);
step_count = round(scenario.duration / step_size);
if abs(step_count * step_size - scenario.duration) > 1e-12
    error("twsbr:cascade_simulation:invalid_duration", ...
        "Scenario duration must be an integer multiple of the plant step.");
end

time = (0:step_count).' * step_size;
sample_count = numel(time);
state = nan(sample_count, 4);
state(1, :) = scenario.initial_state(:).';
position_reference = nan(sample_count, 1);
position_error = nan(sample_count, 1);
theta_reference_raw = nan(sample_count, 1);
theta_reference = nan(sample_count, 1);
theta_error = nan(sample_count, 1);
u_raw = nan(sample_count, 1);
u = nan(sample_count, 1);
disturbance_force = nan(sample_count, 1);
position_integral = nan(sample_count, 1);
saturated = false(sample_count, 1);

controller_state = struct("position_integral", 0.0);
control = zero_control();
success = true;
failure_reason = "";
final_index = sample_count;

for index = 1:sample_count
    current_time = time(index);
    reference = scalar_signal(scenario.x_reference, current_time, ...
        "x_reference");
    force = scalar_signal(scenario.force_disturbance, current_time, ...
        "force_disturbance");
    torque = scalar_signal(scenario.torque_disturbance, current_time, ...
        "torque_disturbance");

    position_reference(index) = reference;
    disturbance_force(index) = force;

    if any(~isfinite(state(index, :)))
        [position_error(index), theta_reference_raw(index), ...
            theta_reference(index), theta_error(index), u_raw(index), ...
            u(index), position_integral(index), saturated(index)] = ...
            control_values(control);
        success = false;
        failure_reason = "nonfinite_state";
        final_index = index;
        break
    end
    if any(~isfinite([reference, force, torque]))
        [position_error(index), theta_reference_raw(index), ...
            theta_reference(index), theta_error(index), u_raw(index), ...
            u(index), position_integral(index), saturated(index)] = ...
            control_values(control);
        success = false;
        failure_reason = "nonfinite_signal";
        final_index = index;
        break
    end

    if mod(index - 1, step_ratio) == 0
        try
            [control, controller_state] = cascade_pid_step( ...
                controller_state, state(index, :).', reference, params);
        catch exception
            if exception.identifier == "twsbr:cascade_pid:nonfinite_output"
                success = false;
                failure_reason = "nonfinite_control";
                final_index = index;
                break
            end
            rethrow(exception);
        end
    end

    position_error(index) = control.position_error;
    theta_reference_raw(index) = control.theta_reference_raw;
    theta_reference(index) = control.theta_reference;
    theta_error(index) = control.theta_error;
    u_raw(index) = control.u_raw;
    u(index) = control.u;
    position_integral(index) = control.position_integral;
    saturated(index) = control.saturated;

    [failed, reason] = check_failure(state(index, :), reference, ...
        force, torque, control, plant_params, params);
    if failed
        success = false;
        failure_reason = reason;
        final_index = index;
        break
    end
    if index == sample_count
        break
    end

    next_state = twsbr_rk4_step(state(index, :).', control.u, ...
        step_size, plant_params, force, torque);
    state(index + 1, :) = next_state.';
end

time = time(1:final_index);
state = state(1:final_index, :);
position_reference = position_reference(1:final_index);
position_error = position_error(1:final_index);
theta_reference_raw = theta_reference_raw(1:final_index);
theta_reference = theta_reference(1:final_index);
theta_error = theta_error(1:final_index);
u_raw = u_raw(1:final_index);
u = u(1:final_index);
disturbance_force = disturbance_force(1:final_index);
position_integral = position_integral(1:final_index);
saturated = saturated(1:final_index);

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
simulation.metrics = calculate_metrics(simulation, scenario);
end

function validate_inputs(plant_params, params, scenario)
if ~isstruct(plant_params) || ~isscalar(plant_params) || ...
        ~all(isfield(plant_params, ["u_max", "theta_fail_deg", "x_limit"]))
    error("twsbr:cascade_simulation:invalid_plant_params", ...
        "Plant parameters must contain u_max, theta_fail_deg, and x_limit.");
end
plant_limits = [plant_params.u_max, plant_params.theta_fail_deg, ...
    plant_params.x_limit];
if ~isnumeric(plant_limits) || ~isreal(plant_limits) || ...
        any(~isfinite(plant_limits)) || any(plant_limits <= 0)
    error("twsbr:cascade_simulation:invalid_plant_params", ...
        "Plant limits must be positive finite real scalars.");
end

if ~isstruct(params) || ~isscalar(params) || ...
        ~all(isfield(params, ["plant_step", "sample_time", ...
        "theta_reference_limit", "u_max"]))
    error("twsbr:cascade_simulation:invalid_params", ...
        "Cascade parameters are missing required fields.");
end
if abs(params.plant_step - 0.001) > 1e-15 || ...
        abs(params.sample_time - 0.01) > 1e-15
    error("twsbr:cascade_simulation:invalid_timing", ...
        "Cascade simulation requires plant_step=0.001 and sample_time=0.01.");
end

required_fields = ["name", "initial_state", "duration", "x_reference", ...
    "force_disturbance", "torque_disturbance", "reference_start", ...
    "disturbance_end"];
if ~isstruct(scenario) || ~isscalar(scenario) || ...
        ~all(isfield(scenario, required_fields))
    error("twsbr:cascade_simulation:invalid_scenario", ...
        "Scenario must be a scalar structure with all required fields.");
end
if ~isnumeric(scenario.initial_state) || ~isreal(scenario.initial_state) || ...
        ~isvector(scenario.initial_state) || numel(scenario.initial_state) ~= 4
    error("twsbr:cascade_simulation:invalid_scenario", ...
        "Initial state must contain four real numeric values.");
end
if ~is_finite_real_scalar(scenario.duration) || scenario.duration <= 0
    error("twsbr:cascade_simulation:invalid_scenario", ...
        "Scenario duration must be a positive finite scalar.");
end
event_times = [scenario.reference_start, scenario.disturbance_end];
if ~isnumeric(event_times) || ~isreal(event_times) || ...
        any(~isfinite(event_times)) || any(event_times < 0) || ...
        any(event_times > scenario.duration)
    error("twsbr:cascade_simulation:invalid_scenario", ...
        "Scenario event metadata must be finite and within its duration.");
end
end

function value = scalar_signal(signal_function, time, field_name)
if ~isa(signal_function, "function_handle")
    error("twsbr:cascade_simulation:invalid_scenario", ...
        "Scenario field %s must be a function handle.", field_name);
end
value = signal_function(time);
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value)
    error("twsbr:cascade_simulation:invalid_signal", ...
        "Scenario signal %s must return a real numeric scalar.", field_name);
end
end

function control = zero_control()
control = struct();
control.position_error = 0.0;
control.theta_reference_raw = 0.0;
control.theta_reference = 0.0;
control.theta_error = 0.0;
control.u_raw = 0.0;
control.u = 0.0;
control.position_integral = 0.0;
control.saturated = false;
end

function [position_error, theta_reference_raw, theta_reference, ...
    theta_error, u_raw, u, position_integral, saturated] = ...
    control_values(control)
position_error = control.position_error;
theta_reference_raw = control.theta_reference_raw;
theta_reference = control.theta_reference;
theta_error = control.theta_error;
u_raw = control.u_raw;
u = control.u;
position_integral = control.position_integral;
saturated = control.saturated;
end

function [failed, reason] = check_failure(state, reference, force, torque, ...
    control, plant_params, params)
failed = false;
reason = "";
control_values_vector = [control.position_error, ...
    control.theta_reference_raw, control.theta_reference, ...
    control.theta_error, control.u_raw, control.u, ...
    control.position_integral];
if any(~isfinite(state))
    failed = true;
    reason = "nonfinite_state";
elseif any(~isfinite([reference, force, torque, control_values_vector]))
    failed = true;
    reason = "nonfinite_control";
elseif abs(state(3)) > deg2rad(30.0)
    failed = true;
    reason = "tilt_limit";
elseif abs(state(1)) > plant_params.x_limit
    failed = true;
    reason = "position_limit";
elseif abs(control.theta_reference) > ...
        min(params.theta_reference_limit, deg2rad(12.0))
    failed = true;
    reason = "theta_reference_limit";
elseif abs(control.u) > min(params.u_max, plant_params.u_max)
    failed = true;
    reason = "actuator_limit";
end
end

function metrics = calculate_metrics(simulation, scenario)
actual_position_error = simulation.position_reference - simulation.state(:, 1);
actual_tilt_error = simulation.state(:, 3) - simulation.theta_reference;

metrics = struct();
metrics.max_abs_tilt_deg = rad2deg(max(abs(simulation.state(:, 3))));
metrics.final_tilt_deg = rad2deg(simulation.state(end, 3));
metrics.final_abs_tilt_deg = abs(metrics.final_tilt_deg);
metrics.max_abs_position = max(abs(simulation.state(:, 1)));
metrics.final_position_error = actual_position_error(end);
metrics.final_abs_position_error = abs(metrics.final_position_error);
metrics.position_settling_time = event_settling_time( ...
    simulation.time, actual_position_error, scenario.reference_start, 0.05);
metrics.disturbance_recovery_time = event_settling_time( ...
    simulation.time, actual_position_error, scenario.disturbance_end, 0.10);
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
    rad2deg(max(abs(simulation.theta_reference)));
metrics.max_abs_position_integral = max(abs(simulation.position_integral));
metrics.saturation_duration = sum(diff(simulation.time) .* ...
    double(simulation.saturated(1:end - 1)));
end

function elapsed_time = event_settling_time(time, error, event_time, tolerance)
if event_time == 0.0
    elapsed_time = 0.0;
    return
end
indices = find(time >= event_time - 1e-12);
elapsed_time = inf;
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
