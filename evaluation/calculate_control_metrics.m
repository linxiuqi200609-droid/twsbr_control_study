function metrics = calculate_control_metrics(simulation, scenario, plant_params)
%CALCULATE_CONTROL_METRICS Convert one common simulation to scalar metrics.

validate_inputs(simulation, scenario, plant_params);
sample_count = numel(simulation.time);
simulation_success = logical(simulation.success);

if sample_count == 0
    values = empty_log_metrics(simulation, scenario);
    task_success = false;
else
    values = logged_metrics(simulation, scenario);
    task_success = final_window_success(simulation);
end

success = simulation_success && task_success;
if ~simulation_success
    failure_reason = string(simulation.failure_reason);
elseif ~task_success
    failure_reason = "task_not_settled";
else
    failure_reason = "";
end

metrics = struct( ...
    "controller", string(simulation.controller_name), ...
    "scenario", string(simulation.scenario_name), ...
    "split", string(scenario.split), ...
    "seed", simulation.seed, ...
    "success", success, ...
    "simulation_success", simulation_success, ...
    "failure_reason", failure_reason, ...
    "survived_time", simulation.survived_time, ...
    "theta_rms_deg", values.theta_rms_deg, ...
    "max_abs_theta_deg", values.max_abs_theta_deg, ...
    "max_abs_theta_rad", values.max_abs_theta_rad, ...
    "theta_itae", values.theta_itae, ...
    "attitude_settling_time", values.attitude_settling_time, ...
    "position_itae", values.position_itae, ...
    "final_abs_position_error", values.final_abs_position_error, ...
    "position_settling_time", values.position_settling_time, ...
    "position_overshoot", values.position_overshoot, ...
    "position_drift", values.position_drift, ...
    "control_energy", values.control_energy, ...
    "saturation_time", values.saturation_time, ...
    "saturation_ratio", values.saturation_ratio, ...
    "disturbance_recovery_time", values.disturbance_recovery_time, ...
    "controller_runtime_seconds", simulation.runtime_seconds, ...
    "mean_step_runtime_us", finite_product( ...
        simulation.runtime_seconds / max(sample_count, 1), 1e6));
end

function values = logged_metrics(simulation, scenario)
time = simulation.time;
position = simulation.state(:, 1);
speed = simulation.state(:, 2); %#ok<NASGU>
theta = simulation.state(:, 3);
position_error = finite_abs_difference( ...
    simulation.position_reference, position);
final_reference = simulation.position_reference(end);
max_abs_theta = max(abs(theta));

values = struct( ...
    "theta_rms_deg", finite_product(stable_rms(theta), 180 / pi), ...
    "max_abs_theta_deg", finite_product(max_abs_theta, 180 / pi), ...
    "max_abs_theta_rad", max_abs_theta, ...
    "theta_itae", finite_trapz(time, finite_product(time, abs(theta))), ...
    "attitude_settling_time", permanent_settling_time( ...
        time, theta, deg2rad(2), 0.0, scenario.duration), ...
    "position_itae", finite_trapz(time, finite_product(time, position_error)), ...
    "final_abs_position_error", position_error(end), ...
    "position_settling_time", permanent_settling_time( ...
        time, position_error, 0.05, scenario.reference_start, ...
        scenario.duration), ...
    "position_overshoot", position_overshoot(position, final_reference), ...
    "position_drift", finite_abs_difference(position(end), position(1)), ...
    "control_energy", finite_trapz(time, finite_square(simulation.u)), ...
    "saturation_time", trapz(time, double(simulation.saturated)), ...
    "saturation_ratio", mean(simulation.saturated), ...
    "disturbance_recovery_time", disturbance_recovery_time( ...
        time, position_error, scenario.disturbance_end, scenario.duration));
end

function values = empty_log_metrics(simulation, scenario)
final_reference = available_final_reference(simulation, scenario);
if scenario.disturbance_end == 0
    recovery_time = 0.0;
else
    recovery_time = scenario.duration;
end
values = struct( ...
    "theta_rms_deg", 0.0, ...
    "max_abs_theta_deg", 0.0, ...
    "max_abs_theta_rad", 0.0, ...
    "theta_itae", 0.0, ...
    "attitude_settling_time", scenario.duration, ...
    "position_itae", 0.0, ...
    "final_abs_position_error", abs(final_reference), ...
    "position_settling_time", scenario.duration, ...
    "position_overshoot", 0.0, ...
    "position_drift", 0.0, ...
    "control_energy", 0.0, ...
    "saturation_time", 0.0, ...
    "saturation_ratio", 0.0, ...
    "disturbance_recovery_time", recovery_time);
end

function success = final_window_success(simulation)
final_time = simulation.time(end);
window = simulation.time >= final_time - 0.5;
final_reference = simulation.position_reference(end);
if final_reference == 0
    position_tolerance = 0.30;
else
    position_tolerance = max(0.10, 0.20 * abs(final_reference));
end
success = mean(abs(simulation.state(window, 3))) <= deg2rad(3) && ...
    mean(abs(simulation.state(window, 2))) <= 0.25 && ...
    mean(finite_abs_difference(simulation.position_reference(window), ...
        simulation.state(window, 1))) <= position_tolerance;
end

function settling_time = permanent_settling_time( ...
        time, error_signal, tolerance, start_time, duration)
has_eligible_sample = any(time >= start_time);
settling_index = 0;
permanently_settled = true;
for index = numel(time):-1:1
    if time(index) < start_time
        break
    end
    permanently_settled = permanently_settled && ...
        abs(error_signal(index)) <= tolerance;
    if permanently_settled
        settling_index = index;
    end
end
if ~has_eligible_sample || settling_index == 0
    settling_time = duration;
else
    settling_time = max(0.0, time(settling_index) - start_time);
end
end

function recovery_time = disturbance_recovery_time( ...
        time, position_error, disturbance_end, duration)
if disturbance_end == 0
    recovery_time = 0.0;
else
    recovery_time = permanent_settling_time(time, position_error, ...
        0.10, disturbance_end, duration);
end
end

function overshoot = position_overshoot(position, final_reference)
if final_reference == 0
    overshoot = 0.0;
elseif final_reference > 0
    beyond_target = position(position > final_reference);
    if isempty(beyond_target)
        overshoot = 0.0;
    else
        overshoot = max(beyond_target - final_reference);
    end
else
    beyond_target = position(position < final_reference);
    if isempty(beyond_target)
        overshoot = 0.0;
    else
        overshoot = max(final_reference - beyond_target);
    end
end
end

function final_reference = available_final_reference(simulation, scenario)
if ~isempty(simulation.position_reference)
    final_reference = simulation.position_reference(end);
elseif isfield(scenario, "x_reference") && ...
        isa(scenario.x_reference, "function_handle")
    final_reference = scenario.x_reference(scenario.duration);
    if ~is_valid_scalar(final_reference)
        invalid_input();
    end
else
    final_reference = 0.0;
end
end

function validate_inputs(simulation, scenario, plant_params)
simulation_fields = ["controller_name", "scenario_name", "seed", ...
    "time", "state", "position_reference", "u_raw", "u", ...
    "saturated", "success", "failure_reason", "survived_time", ...
    "runtime_seconds"];
scenario_fields = ["split", "reference_start", "disturbance_end", ...
    "duration"];
if ~is_scalar_struct_with_fields(simulation, simulation_fields) || ...
        ~is_scalar_struct_with_fields(scenario, scenario_fields) || ...
        ~is_scalar_struct_with_fields(plant_params, "u_max")
    invalid_input();
end
if ~is_text_scalar(simulation.controller_name) || ...
        ~is_text_scalar(simulation.scenario_name) || ...
        ~is_text_scalar(simulation.failure_reason) || ...
        ~is_text_scalar(scenario.split) || ...
        ~is_valid_scalar(simulation.seed) || simulation.seed < 0 || ...
        fix(simulation.seed) ~= simulation.seed || ...
        ~(islogical(simulation.success) && isscalar(simulation.success)) || ...
        ~is_valid_scalar(simulation.survived_time) || ...
        simulation.survived_time < 0 || ...
        ~is_valid_scalar(simulation.runtime_seconds) || ...
        simulation.runtime_seconds < 0
    invalid_input();
end
if ~is_valid_scalar(scenario.reference_start) || ...
        ~is_valid_scalar(scenario.disturbance_end) || ...
        ~is_valid_scalar(scenario.duration) || scenario.duration <= 0 || ...
        scenario.reference_start < 0 || ...
        scenario.reference_start > scenario.duration || ...
        scenario.disturbance_end < 0 || ...
        scenario.disturbance_end > scenario.duration
    invalid_input();
end
validate_plant(plant_params);

time = simulation.time;
sample_count = numel(time);
if ~isnumeric(time) || ~isreal(time) || ...
        ~isequal(size(time), [sample_count, 1]) || ...
        any(~isfinite(time)) || any(diff(time) <= 0) || ...
        (~isempty(time) && (time(1) < 0 || time(end) > scenario.duration))
    invalid_input();
end

time_tolerance = 16 * eps(max(1.0, abs(scenario.duration)));
if simulation.survived_time > scenario.duration
    invalid_input();
end
if isempty(time)
    if simulation.survived_time > time_tolerance
        invalid_input();
    end
elseif abs(simulation.survived_time - time(end)) > time_tolerance
    invalid_input();
end
if simulation.success
    if isempty(time) || ...
            abs(simulation.survived_time - scenario.duration) > ...
                time_tolerance || ...
            abs(time(end) - scenario.duration) > time_tolerance
        invalid_input();
    end
end
if ~isnumeric(simulation.state) || ~isreal(simulation.state) || ...
        ~isequal(size(simulation.state), [sample_count, 4]) || ...
        any(~isfinite(simulation.state), "all")
    invalid_input();
end
vector_fields = ["position_reference", "u_raw", "u"];
for index = 1:numel(vector_fields)
    value = simulation.(vector_fields(index));
    if ~isnumeric(value) || ~isreal(value) || ...
            ~isequal(size(value), [sample_count, 1]) || ...
            any(~isfinite(value))
        invalid_input();
    end
end
if ~islogical(simulation.saturated) || ...
        ~isequal(size(simulation.saturated), [sample_count, 1])
    invalid_input();
end
if sample_count == 0 && simulation.success
    invalid_input();
end
end

function validate_plant(plant_params)
fields = fieldnames(plant_params);
for index = 1:numel(fields)
    value = plant_params.(fields{index});
    if ~isnumeric(value) || ~isreal(value) || isempty(value) || ...
            any(~isfinite(value), "all")
        invalid_input();
    end
end
if ~is_valid_scalar(plant_params.u_max) || plant_params.u_max <= 0
    invalid_input();
end
end

function valid = is_scalar_struct_with_fields(value, fields)
valid = isstruct(value) && isscalar(value) && all(isfield(value, fields));
end

function valid = is_valid_scalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value);
end

function valid = is_text_scalar(value)
valid = (isstring(value) && isscalar(value) && ~ismissing(value)) || ...
    (ischar(value) && isrow(value));
end

function result = finite_abs_difference(first, second)
first_abs = abs(first);
second_abs = abs(second);
same_sign = sign(first) == sign(second);
result = zeros(size(first));
result(same_sign) = abs(first(same_sign) - second(same_sign));
opposite = ~same_sign;
result(opposite) = finite_sum(first_abs(opposite), second_abs(opposite));
end

function result = finite_square(value)
result = finite_product(abs(value), abs(value));
end

function value = stable_rms(signal)
scale = max(abs(signal));
if scale == 0
    value = 0.0;
else
    value = scale * sqrt(mean((signal / scale) .^ 2));
end
end

function integral = finite_trapz(time, signal)
integral = trapz(time, signal);
if isfinite(integral)
    return
end
integral = 0.0;
for index = 1:(numel(time) - 1)
    average = signal(index) / 2 + signal(index + 1) / 2;
    contribution = finite_product(time(index + 1) - time(index), average);
    integral = finite_sum(integral, contribution);
end
end

function result = finite_product(first, second)
result = first .* second;
overflow = ~isfinite(result);
if any(overflow, "all")
    result(overflow) = realmax;
end
end

function result = finite_sum(first, second)
result = first + second;
overflow = ~isfinite(result);
if any(overflow, "all")
    result(overflow) = realmax;
end
end

function invalid_input()
error("twsbr:metrics:invalid_input", ...
    "Metric inputs must satisfy the common simulation contract.");
end
