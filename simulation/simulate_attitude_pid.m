function simulation = simulate_attitude_pid(plant_params, pid_params, scenario)
%SIMULATE_ATTITUDE_PID Simulate upright attitude control at fixed steps.

validate_scenario(scenario);

step_size = pid_params.plant_step;
step_ratio = round(pid_params.sample_time / step_size);
step_count = round(scenario.duration / step_size);
if abs(step_count * step_size - scenario.duration) > 1e-12
    error("twsbr:simulation:invalid_duration", ...
        "Scenario duration must be an integer multiple of the plant step.");
end

time = (0:step_count).' * step_size;
sample_count = numel(time);
state = nan(sample_count, 4);
state(1, :) = scenario.initial_state(:).';
theta_reference = nan(sample_count, 1);
u_raw = nan(sample_count, 1);
u = nan(sample_count, 1);
saturated = false(sample_count, 1);
integral_error = nan(sample_count, 1);
force_disturbance = nan(sample_count, 1);
torque_disturbance = nan(sample_count, 1);

controller_state = struct("integral_error", 0.0);
control = struct("u_raw", 0.0, "u", 0.0, "saturated", false);
success = true;
failure_reason = "";
final_index = sample_count;

for index = 1:sample_count
    current_time = time(index);
    reference = scalar_signal(scenario.theta_reference, current_time, ...
        "theta_reference");
    force = scalar_signal(scenario.force_disturbance, current_time, ...
        "force_disturbance");
    torque = scalar_signal(scenario.torque_disturbance, current_time, ...
        "torque_disturbance");

    if mod(index - 1, step_ratio) == 0
        [control, controller_state] = attitude_pid_step( ...
            controller_state, state(index, 3), state(index, 4), ...
            reference, pid_params);
    end

    theta_reference(index) = reference;
    u_raw(index) = control.u_raw;
    u(index) = control.u;
    saturated(index) = control.saturated;
    integral_error(index) = controller_state.integral_error;
    force_disturbance(index) = force;
    torque_disturbance(index) = torque;

    if index == sample_count
        break
    end

    next_state = twsbr_rk4_step(state(index, :).', control.u, ...
        step_size, plant_params, force, torque);
    state(index + 1, :) = next_state.';

    [failed, reason] = check_failure(next_state, plant_params);
    if failed
        success = false;
        failure_reason = reason;
        final_index = index + 1;
        theta_reference(final_index) = scalar_signal( ...
            scenario.theta_reference, time(final_index), "theta_reference");
        u_raw(final_index) = control.u_raw;
        u(final_index) = control.u;
        saturated(final_index) = control.saturated;
        integral_error(final_index) = controller_state.integral_error;
        force_disturbance(final_index) = scalar_signal( ...
            scenario.force_disturbance, time(final_index), ...
            "force_disturbance");
        torque_disturbance(final_index) = scalar_signal( ...
            scenario.torque_disturbance, time(final_index), ...
            "torque_disturbance");
        break
    end
end

time = time(1:final_index);
state = state(1:final_index, :);
theta_reference = theta_reference(1:final_index);
u_raw = u_raw(1:final_index);
u = u(1:final_index);
saturated = saturated(1:final_index);
integral_error = integral_error(1:final_index);
force_disturbance = force_disturbance(1:final_index);
torque_disturbance = torque_disturbance(1:final_index);

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
simulation.success = success;
simulation.failure_reason = failure_reason;
simulation.metrics = calculate_metrics(simulation);
end

function validate_scenario(scenario)
required_fields = {"name", "initial_state", "duration", ...
    "theta_reference", "force_disturbance", "torque_disturbance"};
if ~isstruct(scenario) || ~isscalar(scenario)
    error("twsbr:simulation:invalid_scenario", ...
        "Scenario must be a scalar structure.");
end
for index = 1:numel(required_fields)
    if ~isfield(scenario, required_fields{index})
        error("twsbr:simulation:invalid_scenario", ...
            "Scenario is missing field: %s", required_fields{index});
    end
end
if ~isnumeric(scenario.initial_state) || numel(scenario.initial_state) ~= 4 || ...
        any(~isfinite(scenario.initial_state), "all")
    error("twsbr:simulation:invalid_scenario", ...
        "Initial state must contain four finite numeric values.");
end
if ~isnumeric(scenario.duration) || ~isscalar(scenario.duration) || ...
        ~isfinite(scenario.duration) || scenario.duration <= 0
    error("twsbr:simulation:invalid_scenario", ...
        "Scenario duration must be a positive finite scalar.");
end
end

function value = scalar_signal(signal_function, time, field_name)
if ~isa(signal_function, "function_handle")
    error("twsbr:simulation:invalid_scenario", ...
        "Scenario field %s must be a function handle.", field_name);
end
value = signal_function(time);
if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value)
    error("twsbr:simulation:invalid_signal", ...
        "Scenario signal %s must return a finite numeric scalar.", field_name);
end
end

function [failed, reason] = check_failure(state, plant_params)
failed = false;
reason = "";
if any(~isfinite(state))
    failed = true;
    reason = "nonfinite_state";
elseif abs(state(3)) > deg2rad(plant_params.theta_fail_deg)
    failed = true;
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
