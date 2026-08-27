function space = controller_parameter_space(controller_name)
%CONTROLLER_PARAMETER_SPACE Return frozen log10 gain bounds by controller.

name = upper(string(controller_name));
if ~isscalar(name)
    unsupported_name(controller_name);
end

switch name
    case "ATTITUDE_PID"
        parameter_names = ["kp"; "ki"; "kd"];
        lower_bounds = [-1.0, -4.0, -2.0];
        upper_bounds = [2.0, 1.0, 1.5];
    case "CASCADE_PID"
        parameter_names = ["kp_x"; "ki_x"; "kd_x"; "kp_theta"; "kd_theta"];
        lower_bounds = [-2.0, -4.0, -2.0, 0.0, -1.5];
        upper_bounds = [0.5, 0.0, 1.0, 2.0, 1.5];
    case "LQR"
        parameter_names = ["q_x"; "q_x_dot"; "q_theta"; ...
            "q_theta_dot"; "r"];
        lower_bounds = [-2.0, -2.0, -2.0, -2.0, -3.0];
        upper_bounds = [4.0, 4.0, 5.0, 4.0, 2.0];
    case "LQI"
        parameter_names = ["q_x"; "q_x_dot"; "q_theta"; ...
            "q_theta_dot"; "q_integral"; "r"];
        lower_bounds = [-2.0, -2.0, -2.0, -2.0, -2.0, -3.0];
        upper_bounds = [4.0, 4.0, 5.0, 4.0, 5.0, 2.0];
    otherwise
        unsupported_name(controller_name);
end

space = struct( ...
    "name", name, ...
    "parameter_names", parameter_names, ...
    "lower_bounds", lower_bounds, ...
    "upper_bounds", upper_bounds, ...
    "dimension", numel(parameter_names));
end

function unsupported_name(controller_name)
error("twsbr:controller:unsupported_name", ...
    "Controller is not implemented in this phase: %s", string(controller_name));
end
