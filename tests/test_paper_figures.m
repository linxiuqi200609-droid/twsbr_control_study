function tests = test_paper_figures
%TEST_PAPER_FIGURES Behavioral tests for publication-oriented figures.
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

function test_generate_paper_figures_creates_png_and_pdf(test_case)
output = string(tempname);
mkdir(output);
cleanup = onCleanup(@() rmdir(output, "s"));
[raw, monte_carlo] = synthetic_figure_inputs();
paths = generate_paper_figures(raw, monte_carlo, output);
verifyEqual(test_case, numel(paths), 12);
verifyTrue(test_case, all(isfile(paths)));
verifyTrue(test_case, all(cellfun(@(path) dir(path).bytes > 0, cellstr(paths))));
verifyTrue(test_case, any(endsWith(paths, "F1_nominal_response.png")));
verifyTrue(test_case, any(endsWith(paths, "F6_normalized_radar.pdf")));
delete(cleanup)
end

function test_time_response_has_fixed_controller_order_colors_and_traces(test_case)
[raw, ~] = synthetic_figure_inputs();
figure_handle = plot_nominal_response(raw);
cleanup = onCleanup(@() close_if_valid(figure_handle));
axes_handles = flipud(findall(figure_handle, "Type", "axes"));
verifyEqual(test_case, numel(axes_handles), 3);
lines = findall(axes_handles(1), "Type", "line");
lines = flipud(lines);
verifyEqual(test_case, numel(lines), 6);
verifyEqual(test_case, lines(2).YData(:), raw.ATTITUDE_PID__S2_position_step_0p75m.state(:, 1));
verifyEqual(test_case, lines(2).Color, [0, 0.4470, 0.7410], "AbsTol", 1e-4);
verifyEqual(test_case, lines(6).Color, [0.4660, 0.6740, 0.1880], "AbsTol", 1e-4);
verifyEqual(test_case, string(axes_handles(3).XLabel.String), "Time (s)");
legend_handle = findall(figure_handle, "Type", "legend");
verifyEqual(test_case, string(legend_handle.Interpreter), "none");
delete(cleanup)
end

function test_saturation_response_keeps_five_axes_and_marks_saturation(test_case)
[raw, ~] = synthetic_figure_inputs();
raw.ATTITUDE_PID__S2_saturation_stress.u_raw = 2 * raw.ATTITUDE_PID__S2_saturation_stress.u_raw;
raw.ATTITUDE_PID__S2_saturation_stress.u = min(max( ...
    raw.ATTITUDE_PID__S2_saturation_stress.u_raw, -1), 1);
raw.ATTITUDE_PID__S2_saturation_stress.saturated = ...
    abs(raw.ATTITUDE_PID__S2_saturation_stress.u_raw) > 1;
figure_handle = plot_saturation_response(raw);
cleanup = onCleanup(@() close_if_valid(figure_handle));
axes_handles = flipud(findall(figure_handle, "Type", "axes"));
verifyEqual(test_case, numel(axes_handles), 5);
verifyEqual(test_case, string(axes_handles(1).Title.String), "ATTITUDE_PID");
verifyEqual(test_case, numel(findall(axes_handles(1), "Type", "line")), 2);
verifyNotEmpty(test_case, findall(axes_handles(1), "Type", "area"));
delete(cleanup)
end

function test_boxplots_exclude_failed_extreme_metrics_and_keep_empty_group(test_case)
[~, monte_carlo] = synthetic_figure_inputs();
failed = monte_carlo(1, :);
failed.success = false;
failed.position_itae = inf;
failed.theta_rms_deg = nan;
failed.control_energy = 999;
failed.saturation_ratio = inf;
zero_success = monte_carlo(monte_carlo.controller == "LQI", :);
zero_success.success(:) = false;
zero_success.position_itae(:) = inf;
zero_success.theta_rms_deg(:) = nan;
zero_success.control_energy(:) = inf;
zero_success.saturation_ratio(:) = nan;
monte_carlo(monte_carlo.controller == "LQI", :) = zero_success;
monte_carlo = [monte_carlo; failed];
figure_handle = plot_monte_carlo_boxplots(monte_carlo);
cleanup = onCleanup(@() close_if_valid(figure_handle));
axes_handles = findall(figure_handle, "Type", "axes");
verifyEqual(test_case, numel(axes_handles), 4);
verifyEqual(test_case, string(axes_handles(1).TickLabelInterpreter), "none");
box_handles = findall(axes_handles(1), "Type", "BoxChart");
verifyTrue(test_case, all(~isinf([box_handles.YData])));
verifyTrue(test_case, any_text_contains(figure_handle, "No successful trials: LQI"));
delete(cleanup)
end

function test_pareto_uses_success_rates_for_marker_area_and_omits_unavailable(test_case)
[~, monte_carlo] = synthetic_figure_inputs();
failed_lqr = monte_carlo(monte_carlo.controller == "LQR", :);
failed_lqr = failed_lqr(1, :);
failed_lqr.success = false;
failed_lqr.position_itae = inf;
failed_lqr.control_energy = inf;
failed_lqi = monte_carlo(monte_carlo.controller == "LQI", :);
failed_lqi.success(:) = false;
failed_lqi.position_itae(:) = inf;
failed_lqi.control_energy(:) = inf;
monte_carlo(monte_carlo.controller == "LQI", :) = failed_lqi;
monte_carlo = [monte_carlo; failed_lqr];
figure_handle = plot_performance_pareto(monte_carlo);
cleanup = onCleanup(@() close_if_valid(figure_handle));
scatter_handles = flipud(findall(figure_handle, "Type", "scatter"));
verifyEqual(test_case, numel(scatter_handles), 4);
verifyEqual(test_case, scatter_handles(4).SizeData, 60 + 160 * (3 / 4));
verifyFalse(test_case, any(string({scatter_handles.DisplayName}) == "LQI"));
verifyTrue(test_case, any_text_contains(figure_handle, "Unavailable: LQI"));
axis_handle = findall(figure_handle, "Type", "axes");
verifyLessThan(test_case, axis_handle.YLim(1), 0.4);
verifyGreaterThan(test_case, axis_handle.YLim(2), 2.1);
delete(cleanup)
end

function test_radar_normalizes_known_values_and_constant_ties(test_case)
monte_carlo = table(["ATTITUDE_PID"; "CASCADE_PID"; "FUZZY_PID"; "LQR"; "LQI"], ...
    true(5, 1), [5;4;3;2;1], [10;8;6;4;2], [7;7;7;7;7], [1;2;3;4;5], ...
    'VariableNames', {'controller', 'success', 'theta_rms_deg', ...
    'position_itae', 'control_energy', 'saturation_ratio'});
figure_handle = plot_normalized_radar(monte_carlo);
cleanup = onCleanup(@() close_if_valid(figure_handle));
line_handles = flipud(findall(figure_handle, "Type", "line"));
verifyEqual(test_case, numel(line_handles), 5);
verifyEqual(test_case, line_handles(1).RData(:), [0;0;1;1;0]);
verifyEqual(test_case, line_handles(5).RData(:), [1;1;1;0;1]);
axis_handle = findall(figure_handle, "Type", "polaraxes");
verifyEqual(test_case, axis_handle.RLim, [0, 1]);
verifyGreaterThan(test_case, axis_handle.RTick, zeros(size(axis_handle.RTick)));
legend_handle = findall(figure_handle, "Type", "legend");
verifyEqual(test_case, string(legend_handle.Interpreter), "none");
annotation_handle = findall(figure_handle, "Type", "textboxshape");
verifyNotEmpty(test_case, annotation_handle);
if isempty(annotation_handle)
    delete(cleanup)
    return
end
verifyEqual(test_case, string(annotation_handle.String), "Relative comparison: 1 is best");
delete(cleanup)
end

function test_figure_functions_reject_malformed_inputs_with_stable_identifiers(test_case)
[raw, monte_carlo] = synthetic_figure_inputs();
missing = rmfield(raw, "LQI__S2_position_step_0p75m");
verifyError(test_case, @() plot_nominal_response(missing), "twsbr:figures:MissingRawResult");
bad_time = raw;
bad_time.ATTITUDE_PID__S2_position_step_0p75m.time(3) = 0.1;
verifyError(test_case, @() plot_nominal_response(bad_time), "twsbr:figures:InvalidRawResult");
bad_state = raw;
bad_state.ATTITUDE_PID__S2_position_step_0p75m.state = ones(21, 3);
verifyError(test_case, @() plot_nominal_response(bad_state), "twsbr:figures:InvalidRawResult");
bad_success = monte_carlo;
bad_success.success = double(bad_success.success);
verifyError(test_case, @() plot_performance_pareto(bad_success), "twsbr:figures:InvalidMonteCarlo");
bad_metric = monte_carlo;
bad_metric.position_itae(1) = nan;
verifyError(test_case, @() plot_monte_carlo_boxplots(bad_metric), "twsbr:figures:InvalidMonteCarlo");
end

function test_generator_closes_only_its_own_figures_when_later_input_fails(test_case)
[raw, monte_carlo] = synthetic_figure_inputs();
sentinel = figure("Visible", "off");
cleanup = onCleanup(@() close_if_valid(sentinel));
raw = rmfield(raw, "ATTITUDE_PID__S2_saturation_stress");
output = string(tempname);
mkdir(output);
output_cleanup = onCleanup(@() rmdir(output, "s"));
verifyError(test_case, @() generate_paper_figures(raw, monte_carlo, output), ...
    "twsbr:figures:MissingRawResult");
verifyTrue(test_case, isvalid(sentinel));
generated_figures = findall(groot, "Type", "figure", "Tag", "twsbrPaperFigure");
verifyEmpty(test_case, generated_figures);
delete(cleanup)
delete(output_cleanup)
end

function [raw, monte_carlo] = synthetic_figure_inputs()
controllers = ["ATTITUDE_PID","CASCADE_PID","FUZZY_PID","LQR","LQI"];
scenarios = ["S2_position_step_0p75m", ...
    "S2_saturation_stress", "S3_positive_impulse"];
raw = struct();
time = (0:0.1:2).';
for controller = controllers
    for scenario = scenarios
        result = struct();
        result.time = time;
        result.state = [0.5*(1-exp(-time)), zeros(size(time)), ...
            deg2rad(2)*exp(-time), zeros(size(time))];
        result.position_reference = 0.5*ones(size(time));
        result.u_raw = 0.8*sin(time);
        result.u = min(max(result.u_raw,-1),1);
        result.saturated = abs(result.u_raw) > 1;
        key = matlab.lang.makeValidName(controller + "__" + scenario);
        raw.(key) = result;
    end
end
records = repmat(struct("controller","", "success",true, ...
    "position_itae",0, "theta_rms_deg",0, "control_energy",0, ...
    "saturation_ratio",0), numel(controllers)*3, 1);
index = 0;
for controller_index = 1:numel(controllers)
    for repeat = 1:3
        index = index + 1;
        records(index).controller = controllers(controller_index);
        records(index).position_itae = controller_index + 0.1*repeat;
        records(index).theta_rms_deg = 0.2*controller_index;
        records(index).control_energy = 0.5*controller_index;
        records(index).saturation_ratio = 0.01*repeat;
    end
end
monte_carlo = struct2table(records);
end

function close_if_valid(figure_handle)
if isgraphics(figure_handle)
    close(figure_handle);
end
end

function matches = any_text_contains(figure_handle, expected)
text_handles = findall(figure_handle, "Type", "text");
matches = any(arrayfun(@(handle) contains(string(handle.String), expected), text_handles));
end
