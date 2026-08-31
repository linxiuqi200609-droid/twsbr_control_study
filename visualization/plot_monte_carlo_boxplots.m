function figure_handle = plot_monte_carlo_boxplots(monte_carlo)
%PLOT_MONTE_CARLO_BOXPLOTS Plot successful-trial metric distributions only.
monte_carlo = validate_figure_monte_carlo(monte_carlo);
[controllers, colors] = figure_controller_style();
metrics = ["position_itae", "theta_rms_deg", "control_energy", "saturation_ratio"];
labels = ["Position ITAE", "Tilt RMS (deg)", ...
    "Applied-input-squared" + newline + "control-cost proxy", "Saturation ratio"];
figure_handle = figure("Visible", "off", "Tag", "twsbrPaperFigure", ...
    "Name", "Monte Carlo boxplots", "Color", "w");
layout = tiledlayout(figure_handle, 2, 2, "TileSpacing", "compact", "Padding", "compact");
unavailable = controllers(~arrayfun(@(controller) any(monte_carlo.success & ...
    monte_carlo.controller == controller), controllers));
if ~isempty(unavailable)
    layout.OuterPosition = [0, 0.18, 1, 0.82];
    annotate_unavailable_controllers(figure_handle, unavailable, "No successful trials:");
end
for metric_index = 1:numel(metrics)
    axis_handle = nexttile(layout);
    hold(axis_handle, "on");
    for controller_index = 1:numel(controllers)
        values = monte_carlo.(metrics(metric_index));
        values = values(monte_carlo.success & ...
            monte_carlo.controller == controllers(controller_index));
        if ~isempty(values)
            boxchart(axis_handle, controller_index * ones(size(values)), values, ...
                "BoxFaceColor", colors(controller_index, :), "MarkerStyle", "none");
        end
    end
    axis_handle.XTick = 1:numel(controllers);
    axis_handle.XLim = [0.5, numel(controllers) + 0.5];
    axis_handle.XTickLabel = controllers;
    axis_handle.TickLabelInterpreter = "none";
    axis_handle.XTickLabelRotation = 25;
    ylabel(axis_handle, labels(metric_index));
    grid(axis_handle, "on");
    axis_handle.GridAlpha = 0.25;
end
end
