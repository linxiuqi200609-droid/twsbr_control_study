function adjusted = holm_adjust(p_values)
%HOLM_ADJUST Return Holm-adjusted p-values in the input shape.

if ~is_valid_p_values(p_values)
    error("twsbr:statistics:InvalidPValues", ...
        "P values must be a nonempty finite real numeric vector in [0, 1].");
end

original_size = size(p_values);
[sorted_values, sort_order] = sort(p_values(:));
count = numel(sorted_values);
sorted_adjusted = zeros(count, 1);
running_maximum = 0;
for index = 1:count
    candidate = min(1, sorted_values(index) * (count - index + 1));
    running_maximum = max(running_maximum, candidate);
    sorted_adjusted(index) = running_maximum;
end
adjusted_values = zeros(count, 1);
adjusted_values(sort_order) = sorted_adjusted;
adjusted = reshape(adjusted_values, original_size);
end

function valid = is_valid_p_values(value)
valid = isnumeric(value) && isreal(value) && isvector(value) && ...
    ~isempty(value) && all(isfinite(value)) && all(value >= 0) && ...
    all(value <= 1);
end
