function simulation = simulate_open_loop(params, initial_state, ...
    stop_time, make_plot)
%SIMULATE_OPEN_LOOP Simulate the uncontrolled nonlinear TWSBR plant.

if nargin < 1
    params = twsbr_params();
end
if nargin < 2
    initial_state = [0.0; 0.0; deg2rad(3.0); 0.0];
end
if nargin < 3
    stop_time = 2.0;
end
if nargin < 4
    make_plot = true;
end

if ~isnumeric(initial_state) || numel(initial_state) ~= 4 || ...
        any(~isfinite(initial_state), "all")
    error("twsbr:simulation:invalid_initial_state", ...
        "Initial state must contain four finite numeric values.");
end
if ~isnumeric(stop_time) || ~isscalar(stop_time) || ...
        ~isfinite(stop_time) || stop_time <= 0
    error("twsbr:simulation:invalid_stop_time", ...
        "Stop time must be a positive finite scalar.");
end
if ~islogical(make_plot) || ~isscalar(make_plot)
    error("twsbr:simulation:invalid_plot_flag", ...
        "Plot flag must be a logical scalar.");
end

initial_state = initial_state(:);
derivative = @(time, state) twsbr_dynamics( ...
    time, state, 0.0, params, 0.0, 0.0);
solver_options = odeset( ...
    "RelTol", 1e-9, ...
    "AbsTol", 1e-11, ...
    "MaxStep", 0.01);

[time, state] = ode45(derivative, [0.0, stop_time], ...
    initial_state, solver_options);

simulation = struct();
simulation.time = time;
simulation.state = state;
simulation.input = zeros(size(time));
simulation.figure_handle = [];

if make_plot
    warning_id = "MATLAB:uicontainer:ScrollableOnWithTextScaling";
    previous_warning = warning("off", warning_id);
    warning_cleanup = onCleanup(@() warning(previous_warning));

    figure_handle = figure( ...
        "Name", "Open Loop Plant Response", ...
        "Color", "white", ...
        "Visible", "off");
    tiledlayout(2, 2, "Padding", "compact", "TileSpacing", "compact");

    nexttile;
    plot(time, state(:, 1), "LineWidth", 1.4);
    grid on;
    xlabel("Time (s)");
    ylabel("Position (m)");
    title("Wheel Position");

    nexttile;
    plot(time, state(:, 2), "LineWidth", 1.4);
    grid on;
    xlabel("Time (s)");
    ylabel("Velocity (m/s)");
    title("Wheel Velocity");

    nexttile;
    plot(time, rad2deg(state(:, 3)), "LineWidth", 1.4);
    grid on;
    xlabel("Time (s)");
    ylabel("Tilt Angle (deg)");
    title("Body Tilt");

    nexttile;
    plot(time, rad2deg(state(:, 4)), "LineWidth", 1.4);
    grid on;
    xlabel("Time (s)");
    ylabel("Angular Velocity (deg/s)");
    title("Body Angular Velocity");

    simulation.figure_handle = figure_handle;
    clear warning_cleanup;
end
end
