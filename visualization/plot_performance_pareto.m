function figure_handle = plot_performance_pareto(monte_carlo)
%PLOT_PERFORMANCE_PARETO Compare successful medians with all-trial success rates.
monte_carlo = validate_figure_monte_carlo(monte_carlo);
[controllers, colors] = figure_controller_style();
figure_handle = figure("Visible", "off", "Tag", "twsbrPaperFigure", ...
    "Name", "Performance Pareto", "Color", "w");
axis_handle = axes(figure_handle);
hold(axis_handle, "on");
unavailable = strings(0, 1);
energy_medians = zeros(0, 1);
position_medians = zeros(0, 1);
for index = 1:numel(controllers)
    rows = monte_carlo.controller == controllers(index);
    successful = rows & monte_carlo.success;
    if ~any(successful)
        unavailable(end + 1, 1) = controllers(index); %#ok<AGROW>
        continue
    end
    position_median = median(monte_carlo.position_itae(successful));
    position_medians(end + 1, 1) = position_median; %#ok<AGROW>
    energy_median = median(monte_carlo.control_energy(successful));
    energy_medians(end + 1, 1) = energy_median; %#ok<AGROW>
    success_rate = sum(successful) / sum(rows);
    scatter(axis_handle, position_median, energy_median, 60 + 160 * success_rate, ...
        "filled", "MarkerFaceColor", colors(index, :), ...
        "MarkerEdgeColor", colors(index, :), "DisplayName", controllers(index));
    text(axis_handle, position_median, energy_median, " " + controllers(index), ...
        "VerticalAlignment", "bottom", "Interpreter", "none");
end
xlabel(axis_handle, "Median position ITAE");
ylabel(axis_handle, "Median applied-input-squared control-cost proxy");
grid(axis_handle, "on");
axis_handle.GridAlpha = 0.25;
if ~isempty(energy_medians)
    legend(axis_handle, "Location", "best", "Box", "off", "Interpreter", "none");
    margin = max(0.08 * range(energy_medians), 0.05);
    ylim(axis_handle, [min(energy_medians) - margin, max(energy_medians) + margin]);
    left_margin = max(0.08 * range(position_medians), 0.05);
    right_margin = max(0.45 * range(position_medians), 0.25);
    xlim(axis_handle, [min(position_medians) - left_margin, ...
        max(position_medians) + right_margin]);
end
if ~isempty(unavailable)
    axis_handle.Position = [0.15, 0.30, 0.80, 0.64];
    annotate_unavailable_controllers(figure_handle, unavailable, "Unavailable (no successful trials):");
end
end
