function controller = reset_controller(controller)
%RESET_CONTROLLER Reset a common controller to its initial state shape.

switch controller.name
    case "ATTITUDE_PID"
        controller.state = struct("integral_error", 0.0);
    case "CASCADE_PID"
        controller.state = struct("position_integral", 0.0);
    case "LQR"
        controller.state = struct();
    otherwise
        error("twsbr:controller:unsupported_name", ...
            "Controller is not implemented in this phase: %s", controller.name);
end

controller.pending_state = [];
controller.pending_legacy_u = [];
end
