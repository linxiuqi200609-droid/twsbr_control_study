function delta = cliffs_delta(first, second)
%CLIFFS_DELTA Return oriented Cliff's delta for two samples.

if ~is_valid_vector(first) || ~is_valid_vector(second)
    error("twsbr:statistics:InvalidValues", ...
        "Samples must be nonempty finite real numeric vectors.");
end

first = first(:);
second = second(:);
comparison = sign(first - second.');
delta = sum(comparison, "all") / numel(comparison);
end

function valid = is_valid_vector(value)
valid = isnumeric(value) && isreal(value) && isvector(value) && ...
    ~isempty(value) && all(isfinite(value));
end
