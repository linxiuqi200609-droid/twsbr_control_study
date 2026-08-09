function output_path = plot_cascade_pid_results(simulation, output_path)
%PLOT_CASCADE_PID_RESULTS Save a five-panel cascade PID response plot.

output_path = string(output_path);
if ~isscalar(output_path) || ismissing(output_path) || strlength(output_path) == 0
    error("twsbr:cascade_plot:invalid_output_path", ...
        "output_path must be a nonempty text scalar.");
end

output_directory = string(fileparts(output_path));
if strlength(output_directory) > 0 && ~isfolder(output_directory)
    mkdir(output_directory);
end

warning_id = "MATLAB:uicontainer:ScrollableOnWithTextScaling";
warning_state = warning("query", warning_id);
warning("off", warning_id);
warning_cleanup = onCleanup( ...
    @() warning(warning_state.state, warning_id));

figure_handle = figure( ...
    "Visible", "off", ...
    "Color", "white", ...
    "Name", "Cascade PID Response", ...
    "Position", [100, 100, 1100, 900]);
figure_cleanup = onCleanup(@() close_if_valid(figure_handle));
layout = tiledlayout(figure_handle, 5, 1, ...
    "TileSpacing", "compact", "Padding", "compact");
title(layout, sprintf("Cascade PID: %s", ...
    strrep(char(simulation.scenario_name), "_", " ")));

time = simulation.time;

axis_handle = nexttile(layout);
plot(axis_handle, time, simulation.state(:, 1), ...
    "LineWidth", 1.5, "DisplayName", "Position");
hold(axis_handle, "on");
plot(axis_handle, time, simulation.position_reference, "--", ...
    "LineWidth", 1.2, "DisplayName", "Reference");
title(axis_handle, "Position Tracking");
ylabel(axis_handle, "Position (m)");
grid(axis_handle, "on");
legend(axis_handle, "Location", "best");

axis_handle = nexttile(layout);
plot(axis_handle, time, rad2deg(simulation.state(:, 3)), ...
    "LineWidth", 1.5, "DisplayName", "Body tilt");
hold(axis_handle, "on");
plot(axis_handle, time, rad2deg(simulation.theta_reference), "--", ...
    "LineWidth", 1.2, "DisplayName", "Attitude reference");
title(axis_handle, "Tilt Tracking");
ylabel(axis_handle, "Tilt (deg)");
grid(axis_handle, "on");
legend(axis_handle, "Location", "best");

axis_handle = nexttile(layout);
plot(axis_handle, time, rad2deg(simulation.state(:, 4)), ...
    "LineWidth", 1.5, "DisplayName", "Angular rate");
title(axis_handle, "Angular Rate");
ylabel(axis_handle, "Angular rate (deg/s)");
grid(axis_handle, "on");

axis_handle = nexttile(layout);
plot(axis_handle, time, simulation.u_raw, ...
    "LineWidth", 1.2, "DisplayName", "Raw command");
hold(axis_handle, "on");
plot(axis_handle, time, simulation.u, ...
    "LineWidth", 1.5, "DisplayName", "Applied command");
title(axis_handle, "Control Command");
ylabel(axis_handle, "Control command");
grid(axis_handle, "on");
legend(axis_handle, "Location", "best");

axis_handle = nexttile(layout);
plot(axis_handle, time, simulation.position_integral, ...
    "LineWidth", 1.5, "DisplayName", "Position integral");
title(axis_handle, "Position Integral");
xlabel(axis_handle, "Time (s)");
ylabel(axis_handle, "Position integral (m s)");
grid(axis_handle, "on");

drawnow;
exportgraphics(figure_handle, output_path, "Resolution", 160);
close(figure_handle);
clear figure_cleanup;
end

function close_if_valid(figure_handle)
if isgraphics(figure_handle, "figure")
    close(figure_handle);
end
end
