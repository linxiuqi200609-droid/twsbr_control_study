function comparison = compare_matlab_simulink( ...
        matlab_result, simulink_result, plant_step)
%COMPARE_MATLAB_SIMULINK Compare common applied-input trajectories.

validate_result(matlab_result);
validate_result(simulink_result);
validate_plant_step(plant_step);

final_time = min(matlab_result.time(end), simulink_result.time(end));
if final_time <= 0
    error("twsbr:equivalence:no_overlap", ...
        "Results must overlap for a positive duration.");
end
common_time = (0:plant_step:final_time).';
if isempty(common_time)
    error("twsbr:equivalence:no_overlap", ...
        "Results must overlap for a positive duration.");
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
comparison.accepted = comparison.max_tilt_difference_deg < ...
    comparison.tilt_tolerance_deg && ...
    comparison.max_position_difference_m < comparison.position_tolerance_m && ...
    comparison.max_applied_input_difference < comparison.input_tolerance;
end

function validate_result(result)
required_fields = ["time", "state", "u"];
if ~isstruct(result) || ~isscalar(result) || ...
        ~all(isfield(result, required_fields))
    error("twsbr:equivalence:invalid_result", ...
        "Result must be a scalar structure with time, state, and u fields.");
end
time = result.time;
state = result.state;
u = result.u;
if ~isnumeric(time) || ~isreal(time) || ~iscolumn(time) || ...
        numel(time) < 2 || any(~isfinite(time))
    error("twsbr:equivalence:invalid_time", ...
        "Result time must be a finite column vector with at least two samples.");
end
if any(diff(time) <= 0)
    error("twsbr:equivalence:invalid_time", ...
        "Result time must be strictly increasing.");
end
if ~isnumeric(state) || ~isreal(state) || ...
        ~isequal(size(state), [numel(time), 4]) || any(~isfinite(state), "all")
    error("twsbr:equivalence:invalid_result", ...
        "Result state must be finite N-by-4 data aligned with time.");
end
if ~isnumeric(u) || ~isreal(u) || ~iscolumn(u) || ...
        numel(u) ~= numel(time) || any(~isfinite(u))
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

function values = interpolate_result(time, values, common_time)
if common_time(1) < time(1) - 1e-12 || ...
        common_time(end) > time(end) + 1e-12
    error("twsbr:equivalence:no_overlap", ...
        "Results must cover the common comparison time grid.");
end
values = interp1(time, values, common_time, "pchip");
end
