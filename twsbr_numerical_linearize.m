function [a_matrix, b_matrix] = twsbr_numerical_linearize( ...
    params, state_zero, u_zero, epsilon)
%TWSBR_NUMERICAL_LINEARIZE Compute a central difference linearization.

if nargin < 1
    params = twsbr_params();
end
if nargin < 2
    state_zero = zeros(4, 1);
end
if nargin < 3
    u_zero = 0.0;
end
if nargin < 4
    epsilon = 1e-6;
end

if ~isnumeric(epsilon) || ~isscalar(epsilon) || ...
        ~isfinite(epsilon) || epsilon <= 0
    error("twsbr:linearize:invalid_epsilon", ...
        "Epsilon must be a positive finite scalar.");
end

state_zero = state_zero(:);
if numel(state_zero) ~= 4 || any(~isfinite(state_zero))
    error("twsbr:linearize:invalid_state", ...
        "Linearization state must contain four finite values.");
end

a_matrix = zeros(4, 4);
for state_index = 1:4
    perturbation = zeros(4, 1);
    perturbation(state_index) = epsilon;
    derivative_positive = twsbr_dynamics(0.0, ...
        state_zero + perturbation, u_zero, params, 0.0, 0.0);
    derivative_negative = twsbr_dynamics(0.0, ...
        state_zero - perturbation, u_zero, params, 0.0, 0.0);
    a_matrix(:, state_index) = ...
        (derivative_positive - derivative_negative) / (2.0 * epsilon);
end

derivative_positive = twsbr_dynamics(0.0, state_zero, ...
    u_zero + epsilon, params, 0.0, 0.0);
derivative_negative = twsbr_dynamics(0.0, state_zero, ...
    u_zero - epsilon, params, 0.0, 0.0);
b_matrix = (derivative_positive - derivative_negative) / (2.0 * epsilon);
end
