function [controllers, metrics] = validate_statistics_data( ...
        data, metric_names, varargin)
%VALIDATE_STATISTICS_DATA Validate shared statistical table inputs.

if ~istable(data) || isempty(data) || ~all(ismember( ...
        ["controller", "success"], string(data.Properties.VariableNames)))
    error("twsbr:statistics:InvalidData", ...
        "Data must be a nonempty table with controller and success columns.");
end
if ~islogical(data.success) || ~isvector(data.success) || ...
        numel(data.success) ~= height(data)
    error("twsbr:statistics:InvalidSuccess", ...
        "Success must be a logical column aligned with the table.");
end
validate_controller_column(data.controller, height(data));
metrics = validate_metric_names(metric_names);
for index = 1:numel(metrics)
    if ~ismember(metrics(index), string(data.Properties.VariableNames))
        error("twsbr:statistics:MissingMetric", ...
            "Every requested metric must be a data column.");
    end
    values = data.(metrics(index));
    if ~isnumeric(values) || ~isreal(values) || ~isvector(values) || ...
            numel(values) ~= height(data)
        error("twsbr:statistics:InvalidMetric", ...
            "Each requested metric must be a real numeric table column.");
    end
    if any(~isfinite(values(data.success)))
        error("twsbr:statistics:InvalidMetricValues", ...
            "Requested metric values must be finite for successful trials.");
    end
end
if ~isempty(varargin)
    validate_bootstrap_inputs(varargin{:});
end
controllers = unique(string(data.controller), "stable");
end

function validate_controller_column(value, row_count)
if ~(isstring(value) || iscellstr(value) || iscategorical(value)) || ...
        ~isvector(value) || numel(value) ~= row_count
    error("twsbr:statistics:InvalidController", ...
        "Controller must be a nonmissing text column aligned with the table.");
end
as_text = string(value);
if any(ismissing(as_text)) || any(strlength(as_text) == 0)
    error("twsbr:statistics:InvalidController", ...
        "Controller must be a nonmissing text column aligned with the table.");
end
end

function metrics = validate_metric_names(value)
if ~isstring(value) || ~isvector(value) || isempty(value) || ...
        any(ismissing(value)) || any(strlength(value) == 0)
    error("twsbr:statistics:InvalidMetricNames", ...
        "Metric names must be a nonempty string scalar or vector.");
end
metrics = value(:);
end

function validate_bootstrap_inputs(resamples, seed)
if ~is_valid_positive_integer(resamples)
    error("twsbr:statistics:InvalidResampleCount", ...
        "Bootstrap resamples must be a positive integer.");
end
if ~is_valid_seed(seed)
    error("twsbr:statistics:InvalidSeed", ...
        "Seed must be a nonnegative integer in the MATLAB seed range.");
end
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
