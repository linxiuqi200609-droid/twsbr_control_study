function next_state = twsbr_rk4_step(state, u, step_size, ...
    plant_params, force_disturbance, torque_disturbance)
%TWSBR_RK4_STEP Advance the nonlinear plant by one fixed RK4 step.

if nargin < 5
    force_disturbance = 0.0;
end
if nargin < 6
    torque_disturbance = 0.0;
end

if ~isnumeric(step_size) || ~isscalar(step_size) || ...
        ~isfinite(step_size) || step_size <= 0
    error("twsbr:simulation:invalid_step", ...
        "Step size must be a positive finite scalar.");
end

state = state(:);
dynamics = @(value) twsbr_dynamics(0.0, value, u, plant_params, ...
    force_disturbance, torque_disturbance);

k1 = dynamics(state);
k2 = dynamics(state + 0.5 * step_size * k1);
k3 = dynamics(state + 0.5 * step_size * k2);
k4 = dynamics(state + step_size * k3);

next_state = state + step_size * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
end
