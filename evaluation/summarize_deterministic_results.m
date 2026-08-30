function summary = summarize_deterministic_results(data, metric_names)
%SUMMARIZE_DETERMINISTIC_RESULTS Summarize successful deterministic trials.

[controllers, metrics] = validate_statistics_data(data, metric_names);
summary = build_statistics_summary(data, controllers, metrics);
end
