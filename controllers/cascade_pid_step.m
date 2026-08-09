function [control, next_state] = cascade_pid_step( ...
    controller_state, plant_state, x_reference, params)
%CASCADE_PID_STEP Advance the discrete position-attitude cascade by one sample.

if ~isstruct(controller_state) || ~isscalar(controller_state) || ...
        ~isequal(fieldnames(controller_state), {"position_integral"})
    error("twsbr:cascade_pid:invalid_state", ...
        "Controller state must contain only position_integral.");
end

position_integral = controller_state.position_integral;
if ~is_finite_real_scalar(position_integral) || ...
        ~isnumeric(plant_state) || ~isreal(plant_state) || ...
        ~isvector(plant_state) || numel(plant_state) ~= 4 || ...
        any(~isfinite(plant_state)) || ~is_finite_real_scalar(x_reference)
    error("twsbr:cascade_pid:invalid_input", ...
        "Controller state and inputs must be finite real numeric values.");
end

plant_state = plant_state(:);
x = plant_state(1);
x_dot = plant_state(2);
theta = plant_state(3);
theta_dot = plant_state(4);

position_error = x_reference - x;
ensure_finite_output(position_error);
theta_reference_raw = params.kp_x * position_error ...
    + params.ki_x * position_integral - params.kd_x * x_dot;
ensure_finite_output(theta_reference_raw);
theta_reference = min(max(theta_reference_raw, -params.theta_reference_limit), ...
    params.theta_reference_limit);
ensure_finite_output(theta_reference);
theta_error = theta - theta_reference;
ensure_finite_output(theta_error);
u_raw = params.kp_theta * theta_error + params.kd_theta * theta_dot;
ensure_finite_output(u_raw);
u = min(max(u_raw, -params.u_max), params.u_max);
ensure_finite_output(u);

next_position_integral = position_integral + ...
    params.sample_time * position_error;
ensure_finite_output(next_position_integral);
next_position_integral = min(max(next_position_integral, ...
    -params.position_integral_limit), params.position_integral_limit);
ensure_finite_output(next_position_integral);

control = struct();
control.position_integral = position_integral;
control.position_error = position_error;
control.theta_reference_raw = theta_reference_raw;
control.theta_reference = theta_reference;
control.theta_error = theta_error;
control.u_raw = u_raw;
control.u = u;
control.saturated = abs(u_raw) > params.u_max;

next_state = struct("position_integral", next_position_integral);
end

function ensure_finite_output(value)
if ~isnumeric(value) || ~isreal(value) || any(~isfinite(value(:)))
    error("twsbr:cascade_pid:nonfinite_output", ...
        "Cascade PID calculation produced a nonfinite output.");
end
end

function valid = is_finite_real_scalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end
