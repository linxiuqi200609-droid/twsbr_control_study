function summary = build_statistics_summary(data, controllers, metrics, varargin)
%BUILD_STATISTICS_SUMMARY Build deterministic or bootstrap summary tables.

bootstrap_enabled = ~isempty(varargin);
if bootstrap_enabled
    resamples = varargin{1};
    seed = varargin{2};
end
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
controller_values = string(data.controller);
row = 0;
for controller_index = 1:controller_count
    controller_mask = controller_values == controllers(controller_index);
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
        if bootstrap_enabled
            derived_seed = mod(double(seed) + 104729 * controller_index + ...
                1009 * metric_index, 2^32);
            [bootstrap_low_column(row), bootstrap_high_column(row)] = ...
                bootstrap_mean_ci(successful_values, resamples, derived_seed);
        end
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
