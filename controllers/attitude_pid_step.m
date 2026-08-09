function [control, next_state] = attitude_pid_step( ...
    controller_state, theta, theta_dot, theta_reference, pid_params)
%ATTITUDE_PID_STEP Advance the discrete attitude PID by one sample.

if ~isstruct(controller_state) || ~isscalar(controller_state) || ...
        ~isfield(controller_state, "integral_error")
    error("twsbr:attitude_pid:invalid_state", ...
        "Controller state must contain integral_error.");
end

integral_error = controller_state.integral_error;
values = [integral_error, theta, theta_dot, theta_reference];
if ~isnumeric(values) || numel(values) ~= 4 || any(~isfinite(values))
    error("twsbr:attitude_pid:invalid_input", ...
        "Controller state and inputs must be finite numeric scalars.");
end

theta_error = theta - theta_reference;
u_raw = pid_params.kp * theta_error + ...
    pid_params.ki * integral_error + ...
    pid_params.kd * theta_dot;
u = min(max(u_raw, -pid_params.u_max), pid_params.u_max);
saturated = abs(u_raw) > pid_params.u_max + 1e-12;

next_integral = integral_error;
if ~saturated || theta_error * u_raw < 0
    next_integral = integral_error + pid_params.sample_time * theta_error;
end
next_integral = min(max(next_integral, -pid_params.integral_limit), ...
    pid_params.integral_limit);

control = struct();
control.theta_error = theta_error;
control.integral_error = integral_error;
control.u_raw = u_raw;
control.u = u;
control.saturated = saturated;

next_state = struct("integral_error", next_integral);
end
