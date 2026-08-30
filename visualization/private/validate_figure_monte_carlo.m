function monte_carlo = validate_figure_monte_carlo(monte_carlo)
%VALIDATE_FIGURE_MONTE_CARLO Validate trial metrics used in comparison figures.
required = ["controller", "success", "position_itae", "theta_rms_deg", ...
    "control_energy", "saturation_ratio"];
if ~istable(monte_carlo) || ~all(ismember(required, string(monte_carlo.Properties.VariableNames)))
    invalid_data();
end
controller = monte_carlo.controller;
if ~(isstring(controller) || iscategorical(controller) || iscellstr(controller)) || ...
        numel(controller) ~= height(monte_carlo)
    invalid_data();
end
controller = string(controller(:));
[controllers, ~] = figure_controller_style();
if any(ismissing(controller)) || any(~ismember(controller, controllers))
    invalid_data();
end
if ~islogical(monte_carlo.success) || ~isvector(monte_carlo.success) || ...
        numel(monte_carlo.success) ~= height(monte_carlo)
    invalid_data();
end
metric_names = required(3:end);
for metric = metric_names
    value = monte_carlo.(metric);
    if ~isnumeric(value) || ~isreal(value) || ~isvector(value) || ...
            numel(value) ~= height(monte_carlo)
        invalid_data();
    end
    if any(~isfinite(value(monte_carlo.success)))
        invalid_data();
    end
    monte_carlo.(metric) = value(:);
end
monte_carlo.controller = controller;
monte_carlo.success = monte_carlo.success(:);
end

function invalid_data()
error("twsbr:figures:InvalidMonteCarlo", ...
    "Monte Carlo data must contain valid controller, success, and metric columns.");
end
