function summary = summarize_deterministic_results(data, metric_names)
%SUMMARIZE_DETERMINISTIC_RESULTS Summarize successful deterministic trials.

[controllers, metrics] = validate_inputs(data, metric_names);
summary = create_summary(data, controllers, metrics);
end

function summary = create_summary(data, controllers, metrics)
controller_count = numel(controllers);
metric_count = numel(metrics);
row_count = controller_count * metric_count;
controller_column = strings(row_count, 1);
metric_column = strings(row_count, 1);
n_column = zeros(row_count, 1);
mean_column = nan(row_count, 1);
median_column = nan(row_count, 1);
std_column = nan(row_count, 1);
q1_column = nan(row_count, 1);
q3_column = nan(row_count, 1);
bootstrap_low_column = nan(row_count, 1);
bootstrap_high_column = nan(row_count, 1);
total_column = zeros(row_count, 1);
success_count_column = zeros(row_count, 1);
success_rate_column = nan(row_count, 1);
success_low_column = nan(row_count, 1);
success_high_column = nan(row_count, 1);
row = 0;
for controller_index = 1:controller_count
    controller_mask = string(data.controller) == controllers(controller_index);
    total_count = sum(controller_mask);
    success_mask = controller_mask & data.success;
    success_count = sum(success_mask);
    [success_low, success_high] = wilson_interval( ...
        success_count, total_count, 1.959963984540054);
    for metric_index = 1:metric_count
        row = row + 1;
        values = data.(metrics(metric_index));
        successful_values = values(success_mask);
        controller_column(row) = controllers(controller_index);
        metric_column(row) = metrics(metric_index);
        n_column(row) = numel(successful_values);
        total_column(row) = total_count;
        success_count_column(row) = success_count;
        success_rate_column(row) = success_count / total_count;
        success_low_column(row) = success_low;
        success_high_column(row) = success_high;
        if isempty(successful_values)
            continue
        end
        mean_column(row) = mean(successful_values);
        median_column(row) = median(successful_values);
        std_column(row) = std(successful_values, 0);
        quartiles = prctile(successful_values, [25, 75]);
        q1_column(row) = quartiles(1);
        q3_column(row) = quartiles(2);
    end
end
summary = table(controller_column, metric_column, n_column, mean_column, ...
    median_column, std_column, q1_column, q3_column, bootstrap_low_column, ...
    bootstrap_high_column, total_column, success_count_column, ...
    success_rate_column, success_low_column, success_high_column, ...
    'VariableNames', {'controller', 'metric', 'n', 'mean', 'median', 'std', ...
    'q1', 'q3', 'bootstrap_ci_low', 'bootstrap_ci_high', 'total_n', ...
    'success_count', 'success_rate', 'success_ci_low', 'success_ci_high'});
end

function [controllers, metrics] = validate_inputs(data, metric_names)
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
if ~is_text_column(data.controller) || numel(data.controller) ~= height(data)
    error("twsbr:statistics:InvalidController", ...
        "Controller must be a nonmissing text column aligned with the table.");
end
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
controllers = unique(string(data.controller), "stable");
end

function metrics = validate_metric_names(value)
if ~isstring(value) || ~isvector(value) || isempty(value) || ...
        any(ismissing(value)) || any(strlength(value) == 0)
    error("twsbr:statistics:InvalidMetricNames", ...
        "Metric names must be a nonempty string scalar or vector.");
end
metrics = value(:);
end

function valid = is_text_column(value)
as_text = string(value);
valid = isvector(value) && numel(value) > 0 && all(~ismissing(as_text)) && ...
    all(strlength(as_text) > 0) && (isstring(value) || iscellstr(value) || ...
    iscategorical(value));
end
