function comparison = compare_matlab_simulink( ...
        matlab_result, simulink_result, plant_step)
%COMPARE_MATLAB_SIMULINK Compare common applied-input trajectories.

validate_result(matlab_result);
validate_result(simulink_result);
validate_plant_step(plant_step);
[common_time, ~] = common_time_grid( ...
    matlab_result.time, simulink_result.time, plant_step);
if isfield(matlab_result, "timing")
    validate_timing(matlab_result.timing, matlab_result.time(end));
end
if isfield(simulink_result, "timing")
    validate_timing(simulink_result.timing, simulink_result.time(end));
end

matlab_state = interpolate_result(matlab_result.time, ...
    matlab_result.state, common_time);
simulink_state = interpolate_result(simulink_result.time, ...
    simulink_result.state, common_time);
matlab_u = interpolate_result(matlab_result.time, matlab_result.u, common_time);
simulink_u = interpolate_result(simulink_result.time, simulink_result.u, common_time);

comparison = struct();
comparison.time = common_time;
comparison.max_tilt_difference_deg = rad2deg(max(abs( ...
    matlab_state(:, 3) - simulink_state(:, 3))));
comparison.max_position_difference_m = max(abs( ...
    matlab_state(:, 1) - simulink_state(:, 1)));
comparison.max_applied_input_difference = max(abs(matlab_u - simulink_u));
comparison.tilt_tolerance_deg = 0.2;
comparison.position_tolerance_m = 0.01;
comparison.input_tolerance = 0.02;
[comparison.max_fuzzy_gain_relative_error, comparison.fuzzy_gain_accepted] = ...
    compare_optional_fuzzy_diagnostics( ...
    matlab_result, simulink_result, common_time);
comparison.accepted = comparison.max_tilt_difference_deg < ...
    comparison.tilt_tolerance_deg && ...
    comparison.max_position_difference_m < comparison.position_tolerance_m && ...
    comparison.max_applied_input_difference < comparison.input_tolerance && ...
    comparison.fuzzy_gain_accepted;
end

function validate_result(result)
required_fields = ["time", "state", "u"];
if ~isstruct(result) || ~isscalar(result) || ...
        ~all(isfield(result, required_fields))
    error("twsbr:equivalence:invalid_result", ...
        "Result must be a scalar structure with time, state, and u fields.");
end
validate_time(result.time, "twsbr:equivalence:invalid_time");
if ~isnumeric(result.state) || ~isreal(result.state) || ...
        ~isequal(size(result.state), [numel(result.time), 4]) || ...
        any(~isfinite(result.state), "all")
    error("twsbr:equivalence:invalid_result", ...
        "Result state must be finite N-by-4 data aligned with time.");
end
if ~isnumeric(result.u) || ~isreal(result.u) || ~iscolumn(result.u) || ...
        numel(result.u) ~= numel(result.time) || any(~isfinite(result.u))
    error("twsbr:equivalence:invalid_result", ...
        "Applied input must be a finite N-by-1 vector aligned with time.");
end
end

function validate_plant_step(plant_step)
if ~isnumeric(plant_step) || ~isreal(plant_step) || ...
        ~isscalar(plant_step) || ~isfinite(plant_step) || plant_step <= 0
    error("twsbr:equivalence:invalid_step", ...
        "Plant step must be a positive finite real scalar.");
end
end

function [common_time, final_time] = common_time_grid( ...
        matlab_time, simulink_time, plant_step)
if matlab_time(1) > 0 || simulink_time(1) > 0
    error("twsbr:equivalence:no_overlap", ...
        "Results must cover time zero for comparison.");
end
final_time = min(matlab_time(end), simulink_time(end));
if final_time <= 0
    error("twsbr:equivalence:no_overlap", ...
        "Results must overlap for a positive duration.");
end
common_time = (0:plant_step:final_time).';
if isempty(common_time)
    error("twsbr:equivalence:no_overlap", ...
        "Results must overlap for a positive duration.");
end
end

function values = interpolate_result(time, values, common_time)
if common_time(1) < time(1) - 1e-12 || ...
        common_time(end) > time(end) + 1e-12
    error("twsbr:equivalence:no_overlap", ...
        "Results must cover the common comparison time grid.");
end
values = interp1(time, values, common_time, "pchip");
end

function [maximum_relative_error, accepted] = ...
        compare_optional_fuzzy_diagnostics( ...
        matlab_result, simulink_result, common_time)
gain_names = ["kp_theta"; "ki_theta"; "kd_theta"];
matlab_has_fuzzy = has_fuzzy_diagnostics(matlab_result);
simulink_has_fuzzy = has_fuzzy_diagnostics(simulink_result);
if ~matlab_has_fuzzy && ~simulink_has_fuzzy
    maximum_relative_error = nan;
    accepted = true;
    return
end
if ~matlab_has_fuzzy || ~simulink_has_fuzzy
    error("twsbr:equivalence:invalid_diagnostics", ...
        "Both results must expose all fuzzy gain diagnostics.");
end
maximum_relative_error = 0.0;
for index = 1:numel(gain_names)
    name = gain_names(index);
    matlab_values = fuzzy_diagnostic_values(matlab_result, name);
    simulink_values = fuzzy_diagnostic_values(simulink_result, name);
    matlab_values = interpolate_result( ...
        matlab_result.time, matlab_values, common_time);
    simulink_values = interpolate_result( ...
        simulink_result.time, simulink_values, common_time);
    relative_error = abs(matlab_values - simulink_values) ./ ...
        max(abs(matlab_values), eps);
    maximum_relative_error = max(maximum_relative_error, max(relative_error));
end
accepted = maximum_relative_error < 1e-6;
end

function present = has_fuzzy_diagnostics(result)
gain_names = ["kp_theta"; "ki_theta"; "kd_theta"];
present = isfield(result, "diagnostics") && isstruct(result.diagnostics) && ...
        ~isempty(result.diagnostics) && all(isfield(result.diagnostics, gain_names));
end

function values = fuzzy_diagnostic_values(result, name)
diagnostics = result.diagnostics;
if numel(diagnostics) ~= numel(result.time) || ...
        ~all(isfield(diagnostics, name))
    error("twsbr:equivalence:invalid_diagnostics", ...
        "Fuzzy diagnostic %s must align with result time.", name);
end
values = reshape([diagnostics.(name)], [], 1);
if ~isnumeric(values) || ~isreal(values) || any(~isfinite(values))
    error("twsbr:equivalence:invalid_diagnostics", ...
        "Fuzzy diagnostic %s must be finite real numeric data.", name);
end
end

function validate_timing(timing, final_time)
required_fields = ["u_raw_time"; "u_time"; "controller_update_time"; ...
    "u_raw"; "u"; "sample_time"; "validated"];
if ~isstruct(timing) || ~isscalar(timing) || ...
        ~all(isfield(timing, required_fields)) || ...
        ~islogical(timing.validated) || ~isscalar(timing.validated) || ...
        ~timing.validated || ~isnumeric(timing.sample_time) || ...
        ~isreal(timing.sample_time) || ~isscalar(timing.sample_time) || ...
        ~isfinite(timing.sample_time) || abs(timing.sample_time - 0.01) > 1e-15
    error("twsbr:equivalence:invalid_timing", ...
        "Timing metadata must confirm the 0.01 second control schedule.");
end
validate_controller_log_axis( ...
    timing.u_raw_time, timing.u_raw, final_time, timing.sample_time, true);
validate_controller_log_axis( ...
    timing.u_time, timing.u, final_time, timing.sample_time, true);
validate_controller_log_axis( ...
    timing.controller_update_time, [], final_time, timing.sample_time, false);
if isfield(timing, "fuzzy_gain_time") && ~isempty(timing.fuzzy_gain_time)
    validate_controller_log_axis( ...
        timing.fuzzy_gain_time, [], final_time, timing.sample_time, false);
end
end

function validate_controller_log_axis( ...
        time, values, final_time, sample_time, allow_terminal_hold)
validate_time(time, "twsbr:equivalence:invalid_timing");
if ~isempty(values) && (~isnumeric(values) || ~isreal(values) || ...
        ~iscolumn(values) || numel(values) ~= numel(time) || ...
        any(~isfinite(values)))
    error("twsbr:equivalence:invalid_timing", ...
        "Controller log values must be finite real columns aligned with time.");
end
update_time = controller_schedule(final_time, sample_time);
matches_updates = numel(time) == numel(update_time) && ...
    max(abs(time - update_time)) <= 1e-12;
has_terminal_hold = allow_terminal_hold && ...
    final_time > update_time(end) + 1e-12 && ...
    numel(time) == numel(update_time) + 1 && ...
    max(abs(time(1:end - 1) - update_time)) <= 1e-12 && ...
    abs(time(end) - final_time) <= 1e-12;
if has_terminal_hold && ~isempty(values)
    has_terminal_hold = terminal_value_is_held(values);
end
if ~matches_updates && ~has_terminal_hold
    error("twsbr:equivalence:invalid_timing", ...
        "Controller logs must follow the update schedule and terminal hold.");
end

function held = terminal_value_is_held(values)
reference = values(end - 1, :);
terminal = values(end, :);
tolerance = 1e-12 * max(1.0, max(abs([reference; terminal]), [], "all"));
held = all(abs(terminal - reference) <= tolerance, "all");
end
end

function time = controller_schedule(duration, sample_time)
update_count = floor(duration / sample_time + 1e-12);
time = (0:update_count).' * sample_time;
end

function validate_time(time, identifier)
if ~isnumeric(time) || ~isreal(time) || ~iscolumn(time) || ...
        numel(time) < 2 || any(~isfinite(time)) || any(diff(time) <= 0)
    error(identifier, "Time must be a finite strictly increasing column vector.");
end
end
