function figure_handle = plot_normalized_radar(monte_carlo)
%PLOT_NORMALIZED_RADAR Plot relative successful-median scores where one is best.
monte_carlo = validate_figure_monte_carlo(monte_carlo);
[controllers, colors] = figure_controller_style();
metrics = ["theta_rms_deg", "position_itae", "control_energy", "saturation_ratio"];
labels = ["Tilt RMS", "Position ITAE", "Control cost", "Saturation"];
median_values = nan(numel(controllers), numel(metrics));
for controller_index = 1:numel(controllers)
    rows = monte_carlo.controller == controllers(controller_index) & monte_carlo.success;
    for metric_index = 1:numel(metrics)
        values = monte_carlo.(metrics(metric_index));
        if any(rows)
            median_values(controller_index, metric_index) = median(values(rows));
        end
    end
end
normalized = nan(size(median_values));
for metric_index = 1:numel(metrics)
    values = median_values(:, metric_index);
    available = isfinite(values);
    if any(available)
        low = min(values(available));
        high = max(values(available));
        if high == low
            normalized(available, metric_index) = 1;
        else
            normalized(available, metric_index) = (high - values(available)) / (high - low);
        end
    end
end
figure_handle = figure("Visible", "off", "Tag", "twsbrPaperFigure", ...
    "Name", "Normalized radar", "Color", "w");
axis_handle = polaraxes(figure_handle);
unavailable = controllers(all(isnan(normalized), 2));
if ~isempty(unavailable)
    axis_handle.Position = [0.12, 0.32, 0.60, 0.57];
    annotate_unavailable_controllers(figure_handle, unavailable, "Unavailable (no successful trials):");
end
hold(axis_handle, "on");
angles = linspace(0, 2 * pi, numel(metrics) + 1);
for controller_index = 1:numel(controllers)
    values = normalized(controller_index, :);
    if all(isnan(values))
        continue
    end
    polarplot(axis_handle, angles, [values, values(1)], ...
        "Color", colors(controller_index, :), "LineWidth", 1.25, ...
        "DisplayName", controllers(controller_index));
end
axis_handle.ThetaTick = rad2deg(angles(1:end - 1));
axis_handle.ThetaTickLabel = labels;
axis_handle.RLim = [0, 1];
axis_handle.RTick = [0.5, 1];
if numel(unavailable) < numel(controllers)
    legend(axis_handle, "Location", "bestoutside", "Box", "off", "Interpreter", "none");
end
note_bottom = 0.01 + 0.17 * ~isempty(unavailable);
annotation(figure_handle, "textbox", [0.14, note_bottom, 0.70, 0.05], ...
    "String", "Relative comparison: 1 is best", "Interpreter", "none", ...
    "EdgeColor", "none", "HorizontalAlignment", "center", "FontSize", 8);
end
