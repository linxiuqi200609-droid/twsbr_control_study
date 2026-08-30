function results = validate_figure_raw(raw, scenario)
%VALIDATE_FIGURE_RAW Validate and collect deterministic traces for a scenario.
if ~isstruct(raw) || ~isscalar(raw) || ~isstring(scenario) || ~isscalar(scenario)
    error("twsbr:figures:InvalidRawResult", "Raw results must be a scalar struct.");
end
[controllers, ~] = figure_controller_style();
results = cell(numel(controllers), 1);
required = ["time", "state", "position_reference", "u_raw", "u", "saturated"];
for index = 1:numel(controllers)
    key = matlab.lang.makeValidName(controllers(index) + "__" + scenario);
    if ~isfield(raw, key)
        error("twsbr:figures:MissingRawResult", ...
            "Missing raw result for %s in %s.", controllers(index), scenario);
    end
    result = raw.(key);
    if ~isstruct(result) || ~isscalar(result) || ~all(isfield(result, required))
        invalid_result();
    end
    time = result.time;
    state = result.state;
    if ~isnumeric(time) || ~isreal(time) || ~isvector(time) || isempty(time) || ...
            any(~isfinite(time(:))) || (~isscalar(time) && any(diff(time(:)) <= 0)) || ...
            ~isnumeric(state) || ~isreal(state) || ...
            ~isequal(size(state), [numel(time), 4]) || any(~isfinite(state), "all")
        invalid_result();
    end
    vector_fields = ["position_reference", "u_raw", "u"];
    for field = vector_fields
        value = result.(field);
        if ~isnumeric(value) || ~isreal(value) || ~isvector(value) || ...
                numel(value) ~= numel(time) || any(~isfinite(value(:)))
            invalid_result();
        end
        result.(field) = value(:);
    end
    if ~islogical(result.saturated) || ~isvector(result.saturated) || ...
            numel(result.saturated) ~= numel(time)
        invalid_result();
    end
    result.time = time(:);
    result.saturated = result.saturated(:);
    results{index} = result;
end
end

function invalid_result()
error("twsbr:figures:InvalidRawResult", ...
    "Raw traces must be finite, aligned, and use [position speed tilt tilt_rate].");
end
