function tests = test_plot_cascade_pid_results
%TEST_PLOT_CASCADE_PID_RESULTS Tests for the cascade PID result plot.
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

function test_plot_creates_parent_folder_and_png(test_case)
simulation = example_simulation();
temporary_root = string(tempname);
cleanup = onCleanup(@() remove_directory_if_present(temporary_root));
output_path = fullfile(temporary_root, "nested", "cascade.png");

returned_path = plot_cascade_pid_results(simulation, output_path);

verifyEqual(test_case, string(returned_path), output_path);
verifyTrue(test_case, isfile(output_path));
image_information = imfinfo(output_path);
verifyEqual(test_case, string(image_information.Format), "png");
verifyGreaterThan(test_case, image_information.Width, 0);
verifyGreaterThan(test_case, image_information.Height, 0);
end

function test_plot_rejects_non_png_extension(test_case)
temporary_root = string(tempname);
cleanup = onCleanup(@() remove_directory_if_present(temporary_root));
output_path = fullfile(temporary_root, "cascade.jpg");

verifyError(test_case, @() plot_cascade_pid_results( ...
    example_simulation(), output_path), ...
    "twsbr:cascade_plot:invalid_output_extension");
verifyFalse(test_case, isfile(output_path));
end

function test_plot_rejects_missing_extension(test_case)
temporary_root = string(tempname);
cleanup = onCleanup(@() remove_directory_if_present(temporary_root));
output_path = fullfile(temporary_root, "cascade");

verifyError(test_case, @() plot_cascade_pid_results( ...
    example_simulation(), output_path), ...
    "twsbr:cascade_plot:invalid_output_extension");
verifyFalse(test_case, isfile(output_path));
end

function test_plot_closes_only_the_figure_it_creates(test_case)
caller_figure = figure("Visible", "off", "Name", "Caller figure");
cleanup = onCleanup(@() close_if_valid(caller_figure));
figures_before = findall(groot, "Type", "figure");
temporary_root = string(tempname);
directory_cleanup = onCleanup( ...
    @() remove_directory_if_present(temporary_root));

plot_cascade_pid_results(example_simulation(), ...
    fullfile(temporary_root, "cascade.png"));

verifyTrue(test_case, isgraphics(caller_figure, "figure"));
verifyEqual(test_case, findall(groot, "Type", "figure"), figures_before);
end

function test_veto_callback_cannot_leak_owned_figure_after_export(test_case)
original_close_callback = get(groot, "defaultFigureCloseRequestFcn");
set(groot, "defaultFigureCloseRequestFcn", @veto_close);
callback_cleanup = onCleanup(@() set(groot, ...
    "defaultFigureCloseRequestFcn", original_close_callback));
caller_figure = figure("Visible", "off", "Name", "Caller figure");
caller_cleanup = onCleanup(@() delete_if_valid(caller_figure));
figures_before = findall(groot, "Type", "figure");
leak_cleanup = onCleanup(@() delete_figures_except(figures_before));
temporary_root = string(tempname);
directory_cleanup = onCleanup( ...
    @() remove_directory_if_present(temporary_root));

plot_cascade_pid_results(example_simulation(), ...
    fullfile(temporary_root, "cascade.png"));

verifyTrue(test_case, isgraphics(caller_figure, "figure"));
verifyEqual(test_case, findall(groot, "Type", "figure"), figures_before);
end

function test_export_failure_deletes_owned_figure_despite_veto(test_case)
original_close_callback = get(groot, "defaultFigureCloseRequestFcn");
set(groot, "defaultFigureCloseRequestFcn", @veto_close);
callback_cleanup = onCleanup(@() set(groot, ...
    "defaultFigureCloseRequestFcn", original_close_callback));
caller_figure = figure("Visible", "off", "Name", "Caller figure");
caller_cleanup = onCleanup(@() delete_if_valid(caller_figure));
figures_before = findall(groot, "Type", "figure");
leak_cleanup = onCleanup(@() delete_figures_except(figures_before));
temporary_root = string(tempname);
directory_cleanup = onCleanup( ...
    @() remove_directory_if_present(temporary_root));
mkdir(temporary_root);
output_path = fullfile(temporary_root, "directory_conflict.png");
mkdir(output_path);

export_failed = false;
try
    plot_cascade_pid_results(example_simulation(), output_path);
catch exception
    export_failed = true;
    verifyNotEmpty(test_case, string(exception.identifier));
end

verifyTrue(test_case, export_failed);
verifyTrue(test_case, isgraphics(caller_figure, "figure"));
verifyEqual(test_case, findall(groot, "Type", "figure"), figures_before);
end

function test_plot_has_five_ordered_panels_with_aligned_line_data(test_case)
snapshot_key = "twsbr_cascade_plot_snapshot";
remove_snapshot(snapshot_key);
original_delete_callback = get(groot, "defaultFigureDeleteFcn");
set(groot, "defaultFigureDeleteFcn", ...
    {@capture_figure_before_delete, snapshot_key});
callback_cleanup = onCleanup(@() restore_default_delete_callback( ...
    original_delete_callback, snapshot_key));
temporary_root = string(tempname);
directory_cleanup = onCleanup( ...
    @() remove_directory_if_present(temporary_root));
simulation = example_simulation();

plot_cascade_pid_results(simulation, fullfile(temporary_root, "cascade.png"));

verifyTrue(test_case, isappdata(groot, snapshot_key));
snapshot = getappdata(groot, snapshot_key);
verifyEqual(test_case, snapshot.panel_titles, ...
    ["Position Tracking"; "Tilt Tracking"; "Angular Rate"; ...
    "Control Command"; "Position Integral"]);
verifyEqual(test_case, snapshot.y_labels, ...
    ["Position (m)"; "Tilt (deg)"; "Angular rate (deg/s)"; ...
    "Control command"; "Position integral (m s)"]);
verifyEqual(test_case, snapshot.x_labels, [""; ""; ""; ""; "Time (s)"]);

verify_panel_lines(test_case, snapshot.panels(1), simulation.time, ...
    ["Position", "Reference"], ...
    {simulation.state(:, 1), simulation.position_reference});
verify_panel_lines(test_case, snapshot.panels(2), simulation.time, ...
    ["Body tilt", "Attitude reference"], ...
    {rad2deg(simulation.state(:, 3)), ...
    rad2deg(simulation.theta_reference)});
verify_panel_lines(test_case, snapshot.panels(3), simulation.time, ...
    "Angular rate", {rad2deg(simulation.state(:, 4))});
verify_panel_lines(test_case, snapshot.panels(4), simulation.time, ...
    ["Raw command", "Applied command"], ...
    {simulation.u_raw, simulation.u});
verify_panel_lines(test_case, snapshot.panels(5), simulation.time, ...
    "Position integral", {simulation.position_integral});
end

function simulation = example_simulation()
simulation = struct();
simulation.scenario_name = "plot_fixture";
simulation.time = [0.0; 0.1; 0.2];
simulation.state = [ ...
    0.0, 0.1, deg2rad(1.0), deg2rad(2.0); ...
    0.2, 0.0, deg2rad(-2.0), deg2rad(-3.0); ...
    0.4, -0.1, deg2rad(3.0), deg2rad(4.0)];
simulation.position_reference = [0.1; 0.3; 0.5];
simulation.theta_reference = deg2rad([0.5; -1.0; 1.5]);
simulation.u_raw = [1.5; -2.0; 2.5];
simulation.u = [1.0; -1.0; 1.0];
simulation.position_integral = [0.0; 0.02; 0.05];
end

function capture_figure_before_delete(figure_handle, ~, snapshot_key)
axes_handles = findall(figure_handle, "Type", "axes");
tile_numbers = arrayfun(@(axis_handle) axis_handle.Layout.Tile, axes_handles);
[~, order] = sort(tile_numbers);
axes_handles = axes_handles(order);

snapshot = struct();
snapshot.panel_titles = strings(numel(axes_handles), 1);
snapshot.y_labels = strings(numel(axes_handles), 1);
snapshot.x_labels = strings(numel(axes_handles), 1);
snapshot.panels = repmat(struct("names", strings(0, 1), ...
    "x_data", {{}}, "y_data", {{}}), numel(axes_handles), 1);
for panel_index = 1:numel(axes_handles)
    axis_handle = axes_handles(panel_index);
    snapshot.panel_titles(panel_index) = string(axis_handle.Title.String);
    snapshot.y_labels(panel_index) = string(axis_handle.YLabel.String);
    snapshot.x_labels(panel_index) = string(axis_handle.XLabel.String);
    line_handles = flipud(findall(axis_handle, "Type", "line"));
    names = string({line_handles.DisplayName}).';
    snapshot.panels(panel_index).names = names;
    snapshot.panels(panel_index).x_data = ...
        arrayfun(@(line_handle) line_handle.XData(:), ...
        line_handles, "UniformOutput", false);
    snapshot.panels(panel_index).y_data = ...
        arrayfun(@(line_handle) line_handle.YData(:), ...
        line_handles, "UniformOutput", false);
end
setappdata(groot, snapshot_key, snapshot);
end

function verify_panel_lines(test_case, panel, expected_time, ...
    expected_names, expected_y_data)
expected_names = string(expected_names(:));
verifyEqual(test_case, panel.names, expected_names);
for index = 1:numel(expected_names)
    verifyEqual(test_case, panel.x_data{index}, expected_time, ...
        "AbsTol", 1e-15);
    verifyEqual(test_case, panel.y_data{index}, expected_y_data{index}, ...
        "AbsTol", 1e-12);
end
end

function restore_default_delete_callback(original_callback, snapshot_key)
set(groot, "defaultFigureDeleteFcn", original_callback);
remove_snapshot(snapshot_key);
end

function remove_snapshot(snapshot_key)
if isappdata(groot, snapshot_key)
    rmappdata(groot, snapshot_key);
end
end

function close_if_valid(figure_handle)
if isgraphics(figure_handle, "figure")
    close(figure_handle);
end
end

function veto_close(~, ~)
end

function delete_if_valid(figure_handle)
if isgraphics(figure_handle, "figure")
    delete(figure_handle);
end
end

function delete_figures_except(preserved_figures)
current_figures = findall(groot, "Type", "figure");
for index = 1:numel(current_figures)
    if ~any(current_figures(index) == preserved_figures)
        delete(current_figures(index));
    end
end
end

function remove_directory_if_present(directory_path)
if isfolder(directory_path)
    rmdir(directory_path, "s");
end
end
