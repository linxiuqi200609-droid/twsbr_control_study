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
theta_reference_raw = params.kp_x * position_error ...
    + params.ki_x * position_integral - params.kd_x * x_dot;
theta_reference = min(max(theta_reference_raw, -params.theta_reference_limit), ...
    params.theta_reference_limit);
theta_error = theta - theta_reference;
u_raw = params.kp_theta * theta_error + params.kd_theta * theta_dot;
u = min(max(u_raw, -params.u_max), params.u_max);

next_position_integral = position_integral + ...
    params.sample_time * position_error;
next_position_integral = min(max(next_position_integral, ...
    -params.position_integral_limit), params.position_integral_limit);

control = struct();
control.position_error = position_error;
control.theta_reference_raw = theta_reference_raw;
control.theta_reference = theta_reference;
control.theta_error = theta_error;
control.u_raw = u_raw;
control.u = u;
control.saturated = abs(u_raw) > params.u_max;

next_state = struct("position_integral", next_position_integral);
end

function valid = is_finite_real_scalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end
