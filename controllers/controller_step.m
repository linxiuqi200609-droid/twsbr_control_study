function [control, controller] = controller_step( ...
        controller, time, measured_state, x_reference)
%CONTROLLER_STEP Run one controller sample without committing controller state.

validate_finite_scalar(time, "time");
validate_finite_scalar(x_reference, "x_reference");
if ~isnumeric(measured_state) || ~isreal(measured_state) || ...
        ~isequal(size(measured_state), [4, 1]) || any(~isfinite(measured_state))
    error("twsbr:controller:invalid_state", ...
        "Measured state must be a finite 4-by-1 real numeric column vector.");
end

switch controller.name
    case "ATTITUDE_PID"
        [control, controller] = adapt_attitude_pid_controller( ...
            controller, measured_state);
    case "CASCADE_PID"
        [control, controller] = adapt_cascade_pid_controller( ...
            controller, measured_state, x_reference);
    case "LQR"
        control = lqr_step(measured_state, x_reference, controller.params);
        controller.pending_state = struct();
        controller.pending_legacy_u = [];
    otherwise
        error("twsbr:controller:unsupported_name", ...
            "Controller is not implemented in this phase: %s", controller.name);
end
end

function validate_finite_scalar(value, value_name)
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ~isfinite(value)
    error("twsbr:controller:invalid_input", ...
        "%s must be a finite real numeric scalar.", value_name);
end
end
