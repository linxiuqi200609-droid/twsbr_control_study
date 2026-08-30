function [low, high] = wilson_interval(success_count, total_count, z_value)
%WILSON_INTERVAL Return a Wilson score interval for a binomial proportion.

if ~is_valid_count(success_count) || ~is_valid_count(total_count) || ...
        total_count <= 0 || success_count > total_count
    error("twsbr:statistics:InvalidCounts", ...
        "Counts must be integers with 0 <= successes <= total and total > 0.");
end
if ~is_valid_z_value(z_value)
    error("twsbr:statistics:InvalidZValue", ...
        "The z value must be a positive finite real scalar.");
end

proportion = success_count / total_count;
z_squared = z_value ^ 2;
denominator = 1 + z_squared / total_count;
center = (proportion + z_squared / (2 * total_count)) / denominator;
half_width = z_value * sqrt((proportion * (1 - proportion) + ...
    z_squared / (4 * total_count)) / total_count) / denominator;
low = clamp_probability(center - half_width);
high = clamp_probability(center + half_width);
end

function valid = is_valid_count(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value) && fix(value) == value && value >= 0;
end

function valid = is_valid_z_value(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value) && value > 0;
end

function value = clamp_probability(value)
value = max(0, min(1, value));
end
