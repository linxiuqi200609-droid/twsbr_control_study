function record = scenario_to_record(scenario)
%SCENARIO_TO_RECORD Convert one executable scenario into JSON-safe metadata.

if ~isstruct(scenario) || ~isscalar(scenario)
    error("twsbr:reporting:InvalidScenario", ...
        "Scenario must be a scalar structure.");
end
handle_fields = ["x_reference", "force_disturbance", "torque_disturbance"];
if ~all(isfield(scenario, handle_fields)) || ...
        ~all(structfun(@(value) ~isa(value, "function_handle"), ...
        rmfield(scenario, handle_fields)))
    error("twsbr:reporting:InvalidScenario", ...
        "Scenario must contain only the three supported function handles.");
end
for index = 1:numel(handle_fields)
    if ~isa(scenario.(handle_fields(index)), "function_handle")
        error("twsbr:reporting:InvalidScenario", ...
            "Scenario signal fields must be function handles.");
    end
end
record = rmfield(scenario, cellstr(handle_fields));
end
