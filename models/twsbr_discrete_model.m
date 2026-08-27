function [ad, bd, cd, dd] = twsbr_discrete_model( ...
        plant_params, sample_time)
%TWSBR_DISCRETE_MODEL Discretize the analytical plant with exact ZOH.

[a_matrix, b_matrix, cd, dd] = twsbr_linear_model(plant_params);

if ~isnumeric(sample_time) || ~isscalar(sample_time) || ...
        ~isreal(sample_time) || ~isfinite(sample_time) || sample_time <= 0
    error("twsbr:linear:invalid_sample_time", ...
        "sample_time must be a positive finite real numeric scalar.");
end

zoh = expm([a_matrix, b_matrix; zeros(1, 5)] * sample_time);
if ~isreal(zoh) || any(~isfinite(zoh), "all")
    error("twsbr:linear:discretization_failed", ...
        "Exact ZOH discretization must produce a finite real model.");
end
ad = zoh(1:4, 1:4);
bd = zoh(1:4, 5);
end
