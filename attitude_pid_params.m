function pid_params = attitude_pid_params(overrides, plant_params)
%ATTITUDE_PID_PARAMS Return validated basic attitude PID parameters.

if nargin < 2
    plant_params = twsbr_params();
end

pid_params = struct( ...
    "kp", 1.90, ...
    "ki", 0.20, ...
    "kd", 0.18, ...
    "sample_time", 0.01, ...
    "plant_step", 0.001, ...
    "integral_limit", 0.50, ...
    "u_max", plant_params.u_max);

if nargin < 1
    overrides = struct();
end
if ~isstruct(overrides) || ~isscalar(overrides)
    error("twsbr:attitude_pid:invalid_overrides", ...
        "PID overrides must be a scalar structure.");
end

override_fields = fieldnames(overrides);
valid_fields = fieldnames(pid_params);
for index = 1:numel(override_fields)
    field_name = override_fields{index};
    if ~ismember(field_name, valid_fields)
        error("twsbr:attitude_pid:unknown_field", ...
            "Unknown PID parameter field: %s", field_name);
    end
    pid_params.(field_name) = overrides.(field_name);
end

gain_fields = {"kp", "ki", "kd"};
for index = 1:numel(gain_fields)
    field_name = gain_fields{index};
    value = pid_params.(field_name);
    if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || value < 0
        error("twsbr:attitude_pid:invalid_value", ...
            "PID gain %s must be a nonnegative finite scalar.", field_name);
    end
end

positive_fields = {"sample_time", "plant_step", "integral_limit", "u_max"};
for index = 1:numel(positive_fields)
    field_name = positive_fields{index};
    value = pid_params.(field_name);
    if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || value <= 0
        error("twsbr:attitude_pid:invalid_value", ...
            "PID parameter %s must be a positive finite scalar.", field_name);
    end
end

step_ratio = pid_params.sample_time / pid_params.plant_step;
if abs(step_ratio - round(step_ratio)) > 1e-12
    error("twsbr:attitude_pid:invalid_time_ratio", ...
        "Sample time must be an integer multiple of the plant step.");
end
end
