function [low, high] = bootstrap_mean_ci(values, resample_count, seed)
%BOOTSTRAP_MEAN_CI Return a seeded percentile bootstrap interval for a mean.

if ~is_valid_vector(values)
    error("twsbr:statistics:InvalidValues", ...
        "Values must be a nonempty finite real numeric vector.");
end
if ~is_valid_positive_integer(resample_count)
    error("twsbr:statistics:InvalidResampleCount", ...
        "Resample count must be a positive integer.");
end
if ~is_valid_seed(seed)
    error("twsbr:statistics:InvalidSeed", ...
        "Seed must be a nonnegative integer in the MATLAB seed range.");
end

values = values(:);
stream = RandStream("mt19937ar", "Seed", double(seed));
indices = randi(stream, numel(values), numel(values), resample_count);
bootstrap_means = mean(values(indices), 1);
interval = prctile(bootstrap_means, [2.5, 97.5]);
low = interval(1);
high = interval(2);
end

function valid = is_valid_vector(value)
valid = isnumeric(value) && isreal(value) && isvector(value) && ...
    ~isempty(value) && all(isfinite(value));
end

function valid = is_valid_positive_integer(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value) && value > 0 && fix(value) == value;
end

function valid = is_valid_seed(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value) && value >= 0 && value <= 2^32 - 1 && ...
    fix(value) == value;
end
