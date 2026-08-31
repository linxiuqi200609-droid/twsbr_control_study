function [omnibus, pairwise] = run_nonparametric_tests(data, metric_names)
%RUN_NONPARAMETRIC_TESTS Run successful-trial rank tests by metric.

[controllers, metrics] = validate_statistics_data(data, metric_names);
validate_statistics_toolbox();
[omnibus, pairwise] = calculate_tests(data, controllers, metrics);
end

function [omnibus, pairwise] = calculate_tests(data, controllers, metrics)
metric_count = numel(metrics);
controller_count = numel(controllers);
pair_count = controller_count * (controller_count - 1) / 2;
omnibus_metric = strings(metric_count, 1);
omnibus_group_count = zeros(metric_count, 1);
analyzed_group_count = zeros(metric_count, 1);
omnibus_successful_n = zeros(metric_count, 1);
omnibus_p_value = nan(metric_count, 1);
pairwise_metric = strings(metric_count * pair_count, 1);
first_controller = strings(metric_count * pair_count, 1);
second_controller = strings(metric_count * pair_count, 1);
first_n = zeros(metric_count * pair_count, 1);
second_n = zeros(metric_count * pair_count, 1);
p_value = nan(metric_count * pair_count, 1);
p_value_holm = nan(metric_count * pair_count, 1);
effect_size = nan(metric_count * pair_count, 1);
controller_values = string(data.controller);
pair_row = 0;
for metric_index = 1:metric_count
    metric = metrics(metric_index);
    values = data.(metric);
    successful_mask = data.success;
    successful_values = values(successful_mask);
    successful_controllers = controller_values(successful_mask);
    omnibus_metric(metric_index) = metric;
    omnibus_group_count(metric_index) = controller_count;
    omnibus_successful_n(metric_index) = numel(successful_values);
    active_groups = unique(successful_controllers, "stable");
    analyzed_group_count(metric_index) = numel(active_groups);
    if numel(active_groups) >= 2
        omnibus_p_value(metric_index) = kruskalwallis( ...
            successful_values, categorical(successful_controllers), "off");
    end
    metric_pair_rows = zeros(pair_count, 1);
    metric_pair_index = 0;
    for first_index = 1:(controller_count - 1)
        for second_index = (first_index + 1):controller_count
            pair_row = pair_row + 1;
            metric_pair_index = metric_pair_index + 1;
            metric_pair_rows(metric_pair_index) = pair_row;
            first_values = values(data.success & ...
                controller_values == controllers(first_index));
            second_values = values(data.success & ...
                controller_values == controllers(second_index));
            pairwise_metric(pair_row) = metric;
            first_controller(pair_row) = controllers(first_index);
            second_controller(pair_row) = controllers(second_index);
            first_n(pair_row) = numel(first_values);
            second_n(pair_row) = numel(second_values);
            if isempty(first_values) || isempty(second_values)
                continue
            end
            p_value(pair_row) = ranksum(first_values, second_values);
            effect_size(pair_row) = cliffs_delta(first_values, second_values);
        end
    end
    valid_rows = metric_pair_rows(isfinite(p_value(metric_pair_rows)));
    if ~isempty(valid_rows)
        p_value_holm(valid_rows) = holm_adjust(p_value(valid_rows));
    end
end
omnibus = table(omnibus_metric, omnibus_group_count, omnibus_successful_n, ...
    omnibus_p_value, 'VariableNames', {'metric', 'group_count', ...
    'successful_n', 'p_value'});
omnibus.configured_group_count = omnibus_group_count;
omnibus.analyzed_group_count = analyzed_group_count;
pairwise = table(pairwise_metric, first_controller, second_controller, ...
    first_n, second_n, p_value, p_value_holm, effect_size, ...
    'VariableNames', {'metric', 'first_controller', 'second_controller', ...
    'first_n', 'second_n', 'p_value', 'p_value_holm', 'cliffs_delta'});
end

function validate_statistics_toolbox()
if exist("kruskalwallis", "file") ~= 2 || exist("ranksum", "file") ~= 2
    error("twsbr:statistics:MissingStatisticsToolbox", ...
        "Kruskal-Wallis and rank-sum functions must be available.");
end
end
