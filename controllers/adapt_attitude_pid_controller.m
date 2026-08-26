function [control, controller] = adapt_attitude_pid_controller(controller, measured_state)
%ADAPT_ATTITUDE_PID_CONTROLLER Adapt the legacy attitude PID step.

[legacy, next_state] = attitude_pid_step(controller.state, ...
    measured_state(3), measured_state(4), 0.0, controller.params);
control = struct("u_raw", legacy.u_raw, "theta_reference", 0.0, ...
    "diagnostics", legacy);
controller.pending_state = next_state;
controller.pending_legacy_u = legacy.u;
end
