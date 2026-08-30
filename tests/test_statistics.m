function tests = test_statistics
%TEST_STATISTICS Tests for robust controller-study statistics.
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function teardownOnce(test_case)
path(test_case.TestData.original_path);
end

function test_wilson_and_cliffs_delta_known_values(test_case)
[low, high] = wilson_interval(5, 10, 1.959963984540054);
verifyEqual(test_case, low, 0.2365930905, "AbsTol", 1e-9);
verifyEqual(test_case, high, 0.7634069095, "AbsTol", 1e-9);
verifyEqual(test_case, cliffs_delta([3,4], [1,2]), 1.0);
verifyEqual(test_case, cliffs_delta([1,2], [3,4]), -1.0);
end

function test_wilson_boundaries_and_invalid_inputs(test_case)
z_value = 1.959963984540054;
[low_zero, high_zero] = wilson_interval(0, 10, z_value);
[low_all, high_all] = wilson_interval(10, 10, z_value);
verifyEqual(test_case, low_zero, 0.0, "AbsTol", eps);
verifyGreaterThan(test_case, high_zero, 0.0);
verifyLessThan(test_case, low_all, 1.0);
verifyEqual(test_case, high_all, 1.0, "AbsTol", eps);
invalid_calls = {@() wilson_interval(1.5, 2, z_value), ...
    @() wilson_interval(-1, 2, z_value), @() wilson_interval(3, 2, z_value), ...
    @() wilson_interval(0, 0, z_value)};
verify_all_errors(test_case, invalid_calls, "twsbr:statistics:InvalidCounts");
verifyError(test_case, @() wilson_interval(1, 2, 0), ...
    "twsbr:statistics:InvalidZValue");
end

function test_bootstrap_is_deterministic_and_preserves_global_rng(test_case)
rng(412, "twister");
state_before = rng;
[low_first, high_first] = bootstrap_mean_ci([1;2;4;8], 300, 19);
state_after = rng;
[low_second, high_second] = bootstrap_mean_ci([1;2;4;8], 300, 19);
verifyEqual(test_case, [low_first, high_first], [low_second, high_second]);
verifyEqual(test_case, state_after, state_before);
verifyTrue(test_case, isfinite(low_first) && isfinite(high_first));
verifyLessThanOrEqual(test_case, low_first, high_first);
end

function test_bootstrap_constant_single_and_invalid_inputs(test_case)
verifyEqual(test_case, bootstrap_mean_ci(7, 20, 1), 7);
[low, high] = bootstrap_mean_ci([3,3,3], 20, 1);
verifyEqual(test_case, [low, high], [3, 3]);
verifyError(test_case, @() bootstrap_mean_ci([], 10, 1), ...
    "twsbr:statistics:InvalidValues");
verifyError(test_case, @() bootstrap_mean_ci([1,nan], 10, 1), ...
    "twsbr:statistics:InvalidValues");
verifyError(test_case, @() bootstrap_mean_ci([1,2], 1.5, 1), ...
    "twsbr:statistics:InvalidResampleCount");
verifyError(test_case, @() bootstrap_mean_ci([1,2], 1, -1), ...
    "twsbr:statistics:InvalidSeed");
end

function test_cliffs_delta_ties_and_invalid_inputs(test_case)
verifyEqual(test_case, cliffs_delta([1,2], [1,2]), 0.0);
verifyEqual(test_case, cliffs_delta([1;2], [2;3]), -0.75);
verifyError(test_case, @() cliffs_delta([], 1), ...
    "twsbr:statistics:InvalidValues");
verifyError(test_case, @() cliffs_delta([1,nan], 1), ...
    "twsbr:statistics:InvalidValues");
verifyError(test_case, @() cliffs_delta(complex(1,1), 1), ...
    "twsbr:statistics:InvalidValues");
end

function test_holm_known_values_shape_monotonicity_and_ties(test_case)
input = [0.01, 0.04, 0.03, 0.20];
expected = [0.04, 0.09, 0.09, 0.20];
adjusted = holm_adjust(input);
verifyEqual(test_case, adjusted, expected, "AbsTol", 1e-14);
column = holm_adjust([0.04; 0.01; 0.03]);
verifySize(test_case, column, [3, 1]);
[~, order] = sort(input);
sorted_adjusted = adjusted(order);
verifyGreaterThanOrEqual(test_case, diff(sorted_adjusted), zeros(1, 3));
verifyEqual(test_case, holm_adjust([0.05, 0.05, 0.50]), ...
    [0.15, 0.15, 0.50], "AbsTol", 1e-14);
verifyError(test_case, @() holm_adjust([]), ...
    "twsbr:statistics:InvalidPValues");
verifyError(test_case, @() holm_adjust([0.1, 1.1]), ...
    "twsbr:statistics:InvalidPValues");
end

function test_descriptive_statistics_condition_on_success(test_case)
data = table(["A";"A";"B";"B"], [true;false;true;true], ...
    [1;1000;2;4], 'VariableNames', ...
    {'controller','success','theta_rms_deg'});
summary = summarize_monte_carlo_results(data, "theta_rms_deg", 100, 7);
row_a = summary(summary.controller == "A", :);
verifyEqual(test_case, row_a.mean, 1);
verifyEqual(test_case, row_a.success_rate, 0.5);
verifyEqual(test_case, row_a.n, 1);
end

function test_summary_order_multiple_metrics_and_zero_success_retention(test_case)
data = synthetic_data();
metrics = ["theta_rms_deg", "position_itae"];
summary = summarize_monte_carlo_results(data, metrics, 80, 23);
expected_controllers = ["B";"B";"A";"A";"C";"C"];
expected_metrics = ["theta_rms_deg";"position_itae"; ...
    "theta_rms_deg";"position_itae";"theta_rms_deg";"position_itae"];
verifyEqual(test_case, summary.controller, expected_controllers);
verifyEqual(test_case, summary.metric, expected_metrics);
row_b_theta = summary(summary.controller == "B" & ...
    summary.metric == "theta_rms_deg", :);
verifyEqual(test_case, row_b_theta.n, 2);
verifyEqual(test_case, row_b_theta.mean, 2.5);
row_c = summary(summary.controller == "C", :);
verifyEqual(test_case, row_c.n, [0;0]);
verifyEqual(test_case, row_c.success_count, [0;0]);
verifyEqual(test_case, row_c.total_n, [1;1]);
verifyEqual(test_case, row_c.success_rate, [0;0]);
verifyTrue(test_case, all(isnan(row_c.mean)) && all(isnan(row_c.bootstrap_ci_low)));
verifyTrue(test_case, all(row_c.success_ci_low >= 0 & row_c.success_ci_high <= 1));
end

function test_deterministic_schema_and_success_subset(test_case)
data = synthetic_data();
summary = summarize_deterministic_results(data, ["theta_rms_deg", "position_itae"]);
required = ["controller", "metric", "n", "mean", "median", "std", ...
    "q1", "q3", "bootstrap_ci_low", "bootstrap_ci_high", "total_n", ...
    "success_count", "success_rate", "success_ci_low", "success_ci_high"];
verifyTrue(test_case, all(ismember(required, string(summary.Properties.VariableNames))));
row_a = summary(summary.controller == "A" & ...
    summary.metric == "theta_rms_deg", :);
verifyEqual(test_case, row_a.mean, 1);
verifyEqual(test_case, row_a.n, 1);
verifyTrue(test_case, isnan(row_a.bootstrap_ci_low));
end

function test_failed_nonfinite_values_are_excluded_before_statistics(test_case)
data = table(["A";"A";"B";"B"], [true;false;true;false], ...
    [1;nan;5;inf], 'VariableNames', {'controller','success','metric'});
summary = summarize_monte_carlo_results(data, "metric", 50, 5);
verifyEqual(test_case, summary.mean, [1;5]);
[omnibus, pairwise] = run_nonparametric_tests(data, "metric");
verifyEqual(test_case, omnibus.successful_n, 2);
verifyEqual(test_case, height(pairwise), 1);
verifyEqual(test_case, pairwise.cliffs_delta, -1);
end

function test_nonparametric_pairs_holm_and_oriented_effects(test_case)
data = table(["A";"A";"A";"B";"B";"B";"C";"C";"C"], ...
    true(9,1), [1;2;3;10;11;12;20;21;22], ...
    [22;21;20;12;11;10;3;2;1], 'VariableNames', ...
    {'controller','success','metric_one','metric_two'});
[omnibus, pairwise] = run_nonparametric_tests(data, ["metric_one", "metric_two"]);
verifyEqual(test_case, omnibus.metric, ["metric_one";"metric_two"]);
verifyEqual(test_case, omnibus.group_count, [3;3]);
verifyEqual(test_case, omnibus.successful_n, [9;9]);
verifyEqual(test_case, height(pairwise), 6);
first = pairwise(pairwise.metric == "metric_one" & ...
    pairwise.first_controller == "A" & pairwise.second_controller == "B", :);
second = pairwise(pairwise.metric == "metric_two" & ...
    pairwise.first_controller == "A" & pairwise.second_controller == "B", :);
verifyEqual(test_case, first.cliffs_delta, -1);
verifyEqual(test_case, second.cliffs_delta, 1);
for metric = ["metric_one", "metric_two"]
    rows = pairwise(pairwise.metric == metric, :);
    expected = independent_holm(rows.p_value);
    verifyEqual(test_case, rows.p_value_holm, expected, "AbsTol", 1e-12);
end
end

function test_nonparametric_retains_empty_groups(test_case)
data = table(["A";"B";"C"], [true;false;false], [1;nan;nan], ...
    'VariableNames', {'controller','success','metric'});
[omnibus, pairwise] = run_nonparametric_tests(data, "metric");
verifyEqual(test_case, omnibus.group_count, 3);
verifyEqual(test_case, omnibus.successful_n, 1);
verifyTrue(test_case, isnan(omnibus.p_value));
verifyEqual(test_case, height(pairwise), 3);
verifyTrue(test_case, all(isnan(pairwise.p_value)) && ...
    all(isnan(pairwise.p_value_holm)) && all(isnan(pairwise.cliffs_delta)));
end

function test_summary_and_test_input_errors_are_stable(test_case)
data = synthetic_data();
verifyError(test_case, @() summarize_monte_carlo_results(data, "missing", 10, 1), ...
    "twsbr:statistics:MissingMetric");
verifyError(test_case, @() summarize_monte_carlo_results(data, ["theta_rms_deg"; "bad"], 10, 1), ...
    "twsbr:statistics:MissingMetric");
bad_success = data;
bad_success.success = double(bad_success.success);
verifyError(test_case, @() summarize_deterministic_results(bad_success, "theta_rms_deg"), ...
    "twsbr:statistics:InvalidSuccess");
bad_metric = data;
bad_metric.theta_rms_deg(1) = nan;
verifyError(test_case, @() summarize_deterministic_results(bad_metric, "theta_rms_deg"), ...
    "twsbr:statistics:InvalidMetricValues");
verifyError(test_case, @() run_nonparametric_tests(data, 'theta_rms_deg'), ...
    "twsbr:statistics:InvalidMetricNames");
verifyError(test_case, @() run_nonparametric_tests(struct(), "theta_rms_deg"), ...
    "twsbr:statistics:InvalidData");
end

function verify_all_errors(test_case, calls, identifier)
for index = 1:numel(calls)
    verifyError(test_case, calls{index}, identifier);
end
end

function data = synthetic_data()
data = table(["B";"A";"B";"A";"C"], ...
    [true;true;true;false;false], [2;1;3;1000;nan], ...
    [20;10;30;inf;nan], 'VariableNames', ...
    {'controller','success','theta_rms_deg','position_itae'});
end

function adjusted = independent_holm(p_values)
[sorted, order] = sort(p_values(:));
count = numel(sorted);
running = zeros(count, 1);
previous = 0;
for index = 1:count
    previous = max(previous, min(1, (count - index + 1) * sorted(index)));
    running(index) = previous;
end
adjusted = nan(count, 1);
adjusted(order) = running;
end
