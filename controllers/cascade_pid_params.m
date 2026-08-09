function params = cascade_pid_params(overrides, plant_params)
%CASCADE_PID_PARAMS Return validated parameters for the cascade PID.

if nargin < 2
    plant_params = twsbr_params();
end

if isnumeric(plant_params) && isscalar(plant_params)
    plant_u_max = plant_params;
elseif isstruct(plant_params) && isscalar(plant_params) && ...
        isfield(plant_params, "u_max")
    plant_u_max = plant_params.u_max;
else
    error("twsbr:cascade_pid:invalid_plant_params", ...
        "Plant parameters must contain a positive finite scalar u_max.");
end
if ~is_positive_finite_scalar(plant_u_max)
    error("twsbr:cascade_pid:invalid_plant_params", ...
        "Plant parameters must contain a positive finite scalar u_max.");
end

params = struct( ...
    "kp_x", 0.24100028146267993, ...
    "ki_x", 0.0003962067755988572, ...
    "kd_x", 0.1930824033173246, ...
    "kp_theta", 9.254929149177556, ...
    "kd_theta", 1.0113335430173094, ...
    "sample_time", 0.01, ...
    "theta_reference_limit", deg2rad(12), ...
    "position_integral_limit", 1e6, ...
    "u_max", plant_u_max);

if nargin < 1
    overrides = struct();
end
if ~isstruct(overrides) || ~isscalar(overrides)
    error("twsbr:cascade_pid:invalid_overrides", ...
        "Cascade PID overrides must be a scalar structure.");
end

override_fields = fieldnames(overrides);
valid_fields = fieldnames(params);
for index = 1:numel(override_fields)
    field_name = override_fields{index};
    if ~ismember(field_name, valid_fields)
        error("twsbr:cascade_pid:unknown_field", ...
            "Unknown cascade PID parameter field: %s", field_name);
    end
    params.(field_name) = overrides.(field_name);
end

gain_fields = {"kp_x", "ki_x", "kd_x", "kp_theta", "kd_theta"};
for index = 1:numel(gain_fields)
    field_name = gain_fields{index};
    if ~is_nonnegative_finite_scalar(params.(field_name))
        error("twsbr:cascade_pid:invalid_value", ...
            "Cascade PID gain %s must be a nonnegative finite scalar.", ...
            field_name);
    end
end

positive_fields = {"sample_time", "theta_reference_limit", ...
    "position_integral_limit", "u_max"};
for index = 1:numel(positive_fields)
    field_name = positive_fields{index};
    if ~is_positive_finite_scalar(params.(field_name))
        error("twsbr:cascade_pid:invalid_value", ...
            "Cascade PID parameter %s must be a positive finite scalar.", ...
            field_name);
    end
end
end

function valid = is_nonnegative_finite_scalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value) && value >= 0;
end

function valid = is_positive_finite_scalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value) && value > 0;
end
