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

function test_wilson_rejects_each_malformed_input_category(test_case)
success_count_calls = {@() wilson_interval("bad", 2, 1), ...
    @() wilson_interval(complex(1, 1), 2, 1), @() wilson_interval([], 2, 1), ...
    @() wilson_interval([1, 2], 2, 1), @() wilson_interval(nan, 2, 1), ...
    @() wilson_interval(inf, 2, 1), @() wilson_interval(-inf, 2, 1), ...
    @() wilson_interval(-1, 2, 1), @() wilson_interval(1.5, 2, 1), ...
    @() wilson_interval(3, 2, 1)};
verify_all_errors(test_case, success_count_calls, ...
    "twsbr:statistics:InvalidCounts");
total_count_calls = {@() wilson_interval(1, "bad", 1), ...
    @() wilson_interval(1, complex(2, 1), 1), @() wilson_interval(1, [], 1), ...
    @() wilson_interval(1, [2, 3], 1), @() wilson_interval(1, nan, 1), ...
    @() wilson_interval(1, inf, 1), @() wilson_interval(1, -inf, 1), ...
    @() wilson_interval(1, -1, 1), @() wilson_interval(1, 0, 1), ...
    @() wilson_interval(1, 1.5, 1)};
verify_all_errors(test_case, total_count_calls, "twsbr:statistics:InvalidCounts");
z_calls = {@() wilson_interval(1, 2, "bad"), ...
    @() wilson_interval(1, 2, complex(1, 1)), @() wilson_interval(1, 2, []), ...
    @() wilson_interval(1, 2, [1, 2]), @() wilson_interval(1, 2, nan), ...
    @() wilson_interval(1, 2, inf), @() wilson_interval(1, 2, -inf), ...
    @() wilson_interval(1, 2, 0), @() wilson_interval(1, 2, -1)};
verify_all_errors(test_case, z_calls, "twsbr:statistics:InvalidZValue");
end

function test_wilson_single_trial_boundaries_are_hand_derived(test_case)
z_value = 1.959963984540054;
z_squared = z_value ^ 2;
[low_zero, high_zero] = wilson_interval(0, 1, z_value);
[low_all, high_all] = wilson_interval(1, 1, z_value);
verifyEqual(test_case, low_zero, 0.0, "AbsTol", eps);
verifyEqual(test_case, high_zero, z_squared / (1 + z_squared), "AbsTol", 1e-14);
verifyEqual(test_case, low_all, 1 / (1 + z_squared), "AbsTol", 1e-14);
verifyEqual(test_case, high_all, 1.0, "AbsTol", eps);
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

function test_bootstrap_rejects_each_malformed_input_category(test_case)
value_calls = {@() bootstrap_mean_ci("bad", 1, 0), ...
    @() bootstrap_mean_ci(complex(1, 1), 1, 0), ...
    @() bootstrap_mean_ci([], 1, 0), ...
    @() bootstrap_mean_ci(ones(2), 1, 0), ...
    @() bootstrap_mean_ci([1, inf], 1, 0)};
verify_all_errors(test_case, value_calls, "twsbr:statistics:InvalidValues");
resample_calls = {@() bootstrap_mean_ci(1, "bad", 0), ...
    @() bootstrap_mean_ci(1, [1, 2], 0), ...
    @() bootstrap_mean_ci(1, 0, 0), @() bootstrap_mean_ci(1, 1.5, 0)};
verify_all_errors(test_case, resample_calls, ...
    "twsbr:statistics:InvalidResampleCount");
seed_calls = {@() bootstrap_mean_ci(1, 1, "bad"), ...
    @() bootstrap_mean_ci(1, 1, complex(1, 1)), ...
    @() bootstrap_mean_ci(1, 1, [1, 2]), ...
    @() bootstrap_mean_ci(1, 1, -1), @() bootstrap_mean_ci(1, 1, 1.5), ...
    @() bootstrap_mean_ci(1, 1, 2^32)};
verify_all_errors(test_case, seed_calls, "twsbr:statistics:InvalidSeed");
verifyEqual(test_case, bootstrap_mean_ci(1, 1, 0), 1);
verifyEqual(test_case, bootstrap_mean_ci(1, 1, 2^32 - 1), 1);
end

function test_bootstrap_count_and_seed_cover_all_scalar_categories(test_case)
resample_calls = {@() bootstrap_mean_ci(1, "bad", 0), ...
    @() bootstrap_mean_ci(1, complex(1, 1), 0), ...
    @() bootstrap_mean_ci(1, [], 0), @() bootstrap_mean_ci(1, [1, 2], 0), ...
    @() bootstrap_mean_ci(1, nan, 0), @() bootstrap_mean_ci(1, inf, 0), ...
    @() bootstrap_mean_ci(1, -inf, 0), @() bootstrap_mean_ci(1, -1, 0), ...
    @() bootstrap_mean_ci(1, 0, 0), @() bootstrap_mean_ci(1, 1.5, 0)};
verify_all_errors(test_case, resample_calls, ...
    "twsbr:statistics:InvalidResampleCount");
seed_calls = {@() bootstrap_mean_ci(1, 1, "bad"), ...
    @() bootstrap_mean_ci(1, 1, complex(1, 1)), ...
    @() bootstrap_mean_ci(1, 1, []), @() bootstrap_mean_ci(1, 1, [1, 2]), ...
    @() bootstrap_mean_ci(1, 1, nan), @() bootstrap_mean_ci(1, 1, inf), ...
    @() bootstrap_mean_ci(1, 1, -inf), @() bootstrap_mean_ci(1, 1, -1), ...
    @() bootstrap_mean_ci(1, 1, 1.5), @() bootstrap_mean_ci(1, 1, 2^32)};
verify_all_errors(test_case, seed_calls, "twsbr:statistics:InvalidSeed");
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

function test_cliffs_delta_rejects_every_vector_category(test_case)
first_calls = {@() cliffs_delta("bad", 1), ...
    @() cliffs_delta(complex(1, 1), 1), @() cliffs_delta([], 1), ...
    @() cliffs_delta(ones(2), 1), @() cliffs_delta([1, nan], 1), ...
    @() cliffs_delta([1, inf], 1), @() cliffs_delta([1, -inf], 1)};
second_calls = {@() cliffs_delta(1, "bad"), ...
    @() cliffs_delta(1, complex(1, 1)), @() cliffs_delta(1, []), ...
    @() cliffs_delta(1, ones(2)), @() cliffs_delta(1, [1, nan]), ...
    @() cliffs_delta(1, [1, inf]), @() cliffs_delta(1, [1, -inf])};
verify_all_errors(test_case, first_calls, "twsbr:statistics:InvalidValues");
verify_all_errors(test_case, second_calls, "twsbr:statistics:InvalidValues");
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

function test_holm_rejects_each_malformed_input_category(test_case)
invalid_calls = {@() holm_adjust("bad"), @() holm_adjust(complex(0.1, 1)), ...
    @() holm_adjust([]), @() holm_adjust(ones(2) / 2), ...
    @() holm_adjust([0.1, nan]), @() holm_adjust([0.1, inf]), ...
    @() holm_adjust([0.1, -inf]), @() holm_adjust([-eps, 0.1]), ...
    @() holm_adjust([0.1, 1 + eps])};
verify_all_errors(test_case, invalid_calls, "twsbr:statistics:InvalidPValues");
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

function test_deterministic_and_monte_carlo_share_core_summary_values(test_case)
data = synthetic_data();
metrics = ["theta_rms_deg", "position_itae"];
deterministic = summarize_deterministic_results(data, metrics);
monte_carlo = summarize_monte_carlo_results(data, metrics, 40, 17);
expected_controller = ["B";"B";"A";"A";"C";"C"];
expected_metric = ["theta_rms_deg";"position_itae"; ...
    "theta_rms_deg";"position_itae";"theta_rms_deg";"position_itae"];
expected_n = [2;2;1;1;0;0];
expected_mean = [2.5;25;1;10;nan;nan];
expected_median = expected_mean;
expected_std = [sqrt(0.5);sqrt(50);0;0;nan;nan];
expected_q1 = [2;20;1;10;nan;nan];
expected_q3 = [3;30;1;10;nan;nan];
expected_total = [2;2;2;2;1;1];
expected_success_count = [2;2;1;1;0;0];
expected_success_rate = [1;1;0.5;0.5;0;0];
for summary = {deterministic, monte_carlo}
    value = summary{1};
    verifyEqual(test_case, value.controller, expected_controller);
    verifyEqual(test_case, value.metric, expected_metric);
    verifyEqual(test_case, value.n, expected_n);
    verifyEqual(test_case, value.mean, expected_mean, "AbsTol", 1e-12);
    verifyEqual(test_case, value.median, expected_median, "AbsTol", 1e-12);
    verifyEqual(test_case, value.std, expected_std, "AbsTol", 1e-12);
    verifyEqual(test_case, value.q1, expected_q1, "AbsTol", 1e-12);
    verifyEqual(test_case, value.q3, expected_q3, "AbsTol", 1e-12);
    verifyEqual(test_case, value.total_n, expected_total);
    verifyEqual(test_case, value.success_count, expected_success_count);
    verifyEqual(test_case, value.success_rate, expected_success_rate);
end
verifyTrue(test_case, all(isnan(deterministic.bootstrap_ci_low)));
core_columns = ["controller", "metric", "n", "mean", "median", "std", ...
    "q1", "q3", "total_n", "success_count", "success_rate", ...
    "success_ci_low", "success_ci_high"];
verifyEqual(test_case, deterministic(:, core_columns), ...
    monte_carlo(:, core_columns));
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

function test_omnibus_distinguishes_configured_and_analyzed_groups(test_case)
data = table(["A";"A";"B";"B";"C";"C"], false(6,1), ...
    [1;2;4;5;7;8], 'VariableNames', {'controller','success','metric'});
for active_count = 0:3
    data.success = (1:6).' <= 2 * active_count;
    [omnibus, ~] = run_nonparametric_tests(data, "metric");
    verifyEqual(test_case, omnibus.group_count, 3);
    verifyEqual(test_case, omnibus.configured_group_count, 3);
    verifyEqual(test_case, omnibus.analyzed_group_count, active_count);
    verifyEqual(test_case, omnibus.successful_n, 2 * active_count);
    if active_count < 2
        verifyTrue(test_case, isnan(omnibus.p_value));
    else
        expected = kruskalwallis(data.metric(data.success), ...
            categorical(data.controller(data.success)), "off");
        verifyEqual(test_case, omnibus.p_value, expected);
    end
end
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

function test_unsupported_controller_column_is_rejected_stably(test_case)
controller_values = {struct("name", "A"); "B"};
data = table(controller_values, [true; true], [1; 2], ...
    'VariableNames', {'controller', 'success', 'metric'});
calls = {@() summarize_deterministic_results(data, "metric"), ...
    @() summarize_monte_carlo_results(data, "metric", 10, 1), ...
    @() run_nonparametric_tests(data, "metric")};
verify_all_errors(test_case, calls, "twsbr:statistics:InvalidController");
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
