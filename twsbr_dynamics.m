function state_dot = twsbr_dynamics(~, state, u, params, ...
    force_disturbance, torque_disturbance)
%TWSBR_DYNAMICS Evaluate nonlinear longitudinal plant dynamics.
% State order: [x; x_dot; theta; theta_dot].

if nargin < 5
    force_disturbance = 0.0;
end
if nargin < 6
    torque_disturbance = 0.0;
end

if ~isnumeric(state) || numel(state) ~= 4 || any(~isfinite(state), "all")
    error("twsbr:dynamics:invalid_state", ...
        "State must contain four finite numeric values.");
end

scalar_inputs = [u, force_disturbance, torque_disturbance];
if ~isnumeric(scalar_inputs) || numel(scalar_inputs) ~= 3 || ...
        any(~isfinite(scalar_inputs))
    error("twsbr:dynamics:invalid_input", ...
        "Control and disturbance inputs must be finite numeric scalars.");
end

state = state(:);
x_dot = state(2);
theta = state(3);
theta_dot = state(4);

body_mass = params.body_mass;
wheel_mass = params.wheel_mass_equiv;
com_length = params.com_length;
body_inertia = params.body_inertia;
damping = params.viscous_damping;
gravity = params.gravity;

force = params.motor_force_gain * u + force_disturbance;
mass_sum = wheel_mass + body_mass;
rotational_term = body_inertia + body_mass * com_length^2;
coupling = body_mass * com_length * cos(theta);

rhs_position = force - damping * x_dot + ...
    body_mass * com_length * theta_dot^2 * sin(theta);
rhs_angle = body_mass * gravity * com_length * sin(theta) + ...
    torque_disturbance;

determinant = mass_sum * rotational_term - coupling^2;
if determinant <= 1e-12
    error("twsbr:dynamics:singular_mass_matrix", ...
        "The nonlinear mass matrix is singular.");
end

x_ddot = (rotational_term * rhs_position - coupling * rhs_angle) ...
    / determinant;
theta_ddot = (-coupling * rhs_position + mass_sum * rhs_angle) ...
    / determinant;

state_dot = [x_dot; x_ddot; theta_dot; theta_ddot];
end
