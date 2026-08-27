function [adjustments, diagnostics] = fuzzy_pid_inference( ...
        error_normalized, rate_normalized)
%FUZZY_PID_INFERENCE Evaluate the fixed normalized fuzzy PID adjustments.

validate_input(error_normalized, "error_normalized");
validate_input(rate_normalized, "rate_normalized");

error_clipped = min(max(error_normalized, -1.0), 1.0);
rate_clipped = min(max(rate_normalized, -1.0), 1.0);
error_membership = normalized_membership(error_clipped);
rate_membership = normalized_membership(rate_clipped);

firing = zeros(7, 7);
for error_index = 1:7
    for rate_index = 1:7
        firing(error_index, rate_index) = min( ...
            error_membership(error_index), rate_membership(rate_index));
    end
end
total_firing_strength = sum(firing, "all");
if ~isfinite(total_firing_strength) || total_firing_strength <= 0
    invalid_input("Fuzzy memberships must have positive finite firing strength.");
end

rules = fuzzy_pid_rule_base();
denominator = 3.0 * total_firing_strength;
delta_kp = sum(firing .* rules.delta_kp, "all") / denominator;
delta_ki = sum(firing .* rules.delta_ki, "all") / denominator;
delta_kd = sum(firing .* rules.delta_kd, "all") / denominator;
if any(~isfinite([delta_kp, delta_ki, delta_kd]))
    invalid_input("Fuzzy adjustments must remain finite.");
end

adjustments = struct( ...
    "delta_kp", delta_kp, ...
    "delta_ki", delta_ki, ...
    "delta_kd", delta_kd);
diagnostics = struct( ...
    "error_membership", error_membership, ...
    "rate_membership", rate_membership, ...
    "total_firing_strength", total_firing_strength);
end

function memberships = normalized_membership(value)
centers = linspace(-1.0, 1.0, 7);
spacing = centers(2) - centers(1);
memberships = zeros(1, 7);

if value <= centers(1)
    memberships(1) = 1.0;
elseif value >= centers(end)
    memberships(end) = 1.0;
else
    for index = 1:6
        if value <= centers(index + 1)
            right_weight = (value - centers(index)) / spacing;
            memberships(index) = 1.0 - right_weight;
            memberships(index + 1) = right_weight;
            break
        end
    end
end

membership_sum = sum(memberships);
if ~isfinite(membership_sum) || membership_sum <= 0
    invalid_input("Fuzzy memberships must have positive finite weight.");
end
memberships = memberships / membership_sum;
end

function validate_input(value, value_name)
if ~isnumeric(value) || ~isreal(value) || ...
        ~isscalar(value) || ~isfinite(value)
    invalid_input(sprintf( ...
        "%s must be a finite real numeric scalar.", value_name));
end
end

function invalid_input(message)
error("twsbr:fuzzy:invalid_input", "%s", message);
end
