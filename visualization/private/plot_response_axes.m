function figure_handle = plot_response_axes(results, scenario_label)
%PLOT_RESPONSE_AXES Render the shared three-axis deterministic response layout.
[controllers, colors] = figure_controller_style();
figure_handle = figure("Visible", "off", "Tag", "twsbrPaperFigure", ...
    "Name", scenario_label, "Color", "w");
layout = tiledlayout(figure_handle, 3, 1, "TileSpacing", "compact", ...
    "Padding", "compact");
position_axis = nexttile(layout);
hold(position_axis, "on");
plot(position_axis, results{1}.time, results{1}.position_reference, "k--", ...
    "LineWidth", 1.0, "DisplayName", "Reference");
for index = 1:numel(controllers)
    result = results{index};
    plot(position_axis, result.time, result.state(:, 1), "Color", colors(index, :), ...
        "LineWidth", 1.25, "DisplayName", controllers(index));
end
ylabel(position_axis, "Position (m)");
legend(position_axis, "Location", "best", "NumColumns", 2, "Box", "off", ...
    "Interpreter", "none");
tilt_axis = nexttile(layout);
hold(tilt_axis, "on");
for index = 1:numel(controllers)
    result = results{index};
    plot(tilt_axis, result.time, rad2deg(result.state(:, 3)), ...
        "Color", colors(index, :), "LineWidth", 1.25);
end
ylabel(tilt_axis, "Tilt (deg)");
input_axis = nexttile(layout);
hold(input_axis, "on");
for index = 1:numel(controllers)
    result = results{index};
    plot(input_axis, result.time, result.u, "Color", colors(index, :), ...
        "LineWidth", 1.25);
end
ylabel(input_axis, "Applied input");
xlabel(input_axis, "Time (s)");
for axis_handle = [position_axis, tilt_axis, input_axis]
    grid(axis_handle, "on");
    axis_handle.GridAlpha = 0.25;
end
end
