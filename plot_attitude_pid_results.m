function figure_handle = plot_attitude_pid_results(simulation)
%PLOT_ATTITUDE_PID_RESULTS Create the four panel attitude PID response plot.

warning_id = "MATLAB:uicontainer:ScrollableOnWithTextScaling";
warning_state = warning("query", warning_id);
warning("off", warning_id);
warning_cleanup = onCleanup( ...
    @() warning(warning_state.state, warning_id));

figure_handle = figure( ...
    "Visible", "off", ...
    "Color", "white", ...
    "Name", "Basic Attitude PID Response", ...
    "Position", [100, 100, 1000, 760]);
layout = tiledlayout(figure_handle, 4, 1, ...
    "TileSpacing", "compact", "Padding", "compact");
title(layout, sprintf("Basic Attitude PID: %s", ...
    strrep(char(simulation.scenario_name), "_", " ")));

time = simulation.time;

nexttile(layout);
plot(time, rad2deg(simulation.state(:, 3)), ...
    "LineWidth", 1.5, "DisplayName", "Body tilt");
hold on
plot(time, rad2deg(simulation.theta_reference), "--", ...
    "LineWidth", 1.0, "DisplayName", "Reference");
yline(0.5, ":", "HandleVisibility", "off");
yline(-0.5, ":", "HandleVisibility", "off");
ylabel("Tilt (deg)");
grid on
legend("Location", "best");

nexttile(layout);
plot(time, rad2deg(simulation.state(:, 4)), "LineWidth", 1.5);
ylabel("Rate (deg/s)");
grid on

nexttile(layout);
plot(time, simulation.u_raw, "LineWidth", 1.2, ...
    "DisplayName", "Raw control");
hold on
plot(time, simulation.u, "LineWidth", 1.5, ...
    "DisplayName", "Applied control");
ylabel("Control");
grid on
legend("Location", "best");

nexttile(layout);
plot(time, simulation.state(:, 1), "LineWidth", 1.5);
xlabel("Time (s)");
ylabel("Position (m)");
grid on

drawnow;
end
