function [control, controller] = adapt_cascade_pid_controller( ...
        controller, measured_state, x_reference)
%ADAPT_CASCADE_PID_CONTROLLER Adapt the legacy cascade PID step.

[legacy, next_state] = cascade_pid_step(controller.state, ...
    measured_state, x_reference, controller.params);
control = struct("u_raw", legacy.u_raw, ...
    "theta_reference", legacy.theta_reference, "diagnostics", legacy);
controller.pending_state = next_state;
controller.pending_legacy_u = legacy.u;
end
