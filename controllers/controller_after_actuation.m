function controller = controller_after_actuation(controller, u_raw, applied_u)
%CONTROLLER_AFTER_ACTUATION Commit state after actuator saturation is verified.

validate_finite_scalar(u_raw, "Raw control");
validate_finite_scalar(applied_u, "Applied control");
validate_finite_scalar(controller.pending_legacy_u, "Pending legacy control");
if isempty(controller.pending_state)
    error("twsbr:controller:invalid_state", ...
        "Controller has no pending state to commit.");
end
if abs(applied_u - controller.pending_legacy_u) > 1e-12
    error("twsbr:controller:legacy_saturation_mismatch", ...
        "Common and legacy actuator saturation differ.");
end

controller.state = controller.pending_state;
controller.pending_state = [];
controller.pending_legacy_u = [];
end

function validate_finite_scalar(value, value_name)
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ~isfinite(value)
    error("twsbr:controller:invalid_input", ...
        "%s must be a finite real numeric scalar.", value_name);
end
end
