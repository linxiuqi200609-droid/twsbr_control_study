function [control, controller] = adapt_cascade_pid_controller( ...
        controller, measured_state, x_reference)
%ADAPT_CASCADE_PID_CONTROLLER Adapt the legacy cascade PID step.

try
    [legacy, next_state] = cascade_pid_step(controller.state, ...
        measured_state, x_reference, controller.params);
catch exception
    if strcmp(exception.identifier, "twsbr:cascade_pid:nonfinite_output")
        unified_exception = MException( ...
            "twsbr:controller:nonfinite_output", ...
            "Controller calculation produced a nonfinite output.");
        unified_exception = addCause(unified_exception, exception);
        throw(unified_exception);
    end
    rethrow(exception);
end
control = struct("u_raw", legacy.u_raw, ...
    "theta_reference", legacy.theta_reference, "diagnostics", legacy);
controller.pending_state = next_state;
controller.pending_legacy_u = legacy.u;
end
