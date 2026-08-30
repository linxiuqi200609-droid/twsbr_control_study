function figure_handle = plot_saturation_response(raw)
%PLOT_SATURATION_RESPONSE Compare raw and applied inputs under saturation stress.
results = validate_figure_raw(raw, "S2_saturation_stress");
[controllers, colors] = figure_controller_style();
figure_handle = figure("Visible", "off", "Tag", "twsbrPaperFigure", ...
    "Name", "Saturation response", "Color", "w");
layout = tiledlayout(figure_handle, numel(controllers), 1, ...
    "TileSpacing", "compact", "Padding", "compact");
for index = 1:numel(controllers)
    axis_handle = nexttile(layout);
    result = results{index};
    hold(axis_handle, "on");
    area(axis_handle, result.time, 2 * double(result.saturated) - 1, -1, ...
        "FaceColor", colors(index, :), "FaceAlpha", 0.14, "EdgeColor", "none", ...
        "DisplayName", "Saturation");
    plot(axis_handle, result.time, result.u_raw, "Color", [0.3, 0.3, 0.3], ...
        "LineWidth", 1.0, "DisplayName", "Raw input");
    plot(axis_handle, result.time, result.u, "Color", colors(index, :), ...
        "LineWidth", 1.3, "DisplayName", "Applied input");
    ylabel(axis_handle, "Input");
    title(axis_handle, controllers(index), "Interpreter", "none", "FontWeight", "normal");
    grid(axis_handle, "on");
    axis_handle.GridAlpha = 0.25;
    if index == 1
        legend(axis_handle, "Location", "best", "NumColumns", 3, "Box", "off", ...
            "Interpreter", "none");
    end
end
xlabel(axis_handle, "Time (s)");
end
