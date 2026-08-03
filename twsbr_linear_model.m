function [a_matrix, b_matrix, c_matrix, d_matrix] = ...
    twsbr_linear_model(params)
%TWSBR_LINEAR_MODEL Linearize the plant at the upright equilibrium.

if nargin < 1
    params = twsbr_params();
end

body_mass = params.body_mass;
wheel_mass = params.wheel_mass_equiv;
com_length = params.com_length;
body_inertia = params.body_inertia;
damping = params.viscous_damping;
gravity = params.gravity;
motor_gain = params.motor_force_gain;

mass_sum = wheel_mass + body_mass;
rotational_term = body_inertia + body_mass * com_length^2;
mass_length = body_mass * com_length;
determinant = mass_sum * rotational_term - mass_length^2;

a_matrix = [ ...
    0.0, 1.0, 0.0, 0.0; ...
    0.0, -rotational_term * damping / determinant, ...
        -(body_mass^2) * gravity * com_length^2 / determinant, 0.0; ...
    0.0, 0.0, 0.0, 1.0; ...
    0.0, mass_length * damping / determinant, ...
        mass_sum * body_mass * gravity * com_length / determinant, 0.0];

b_matrix = [ ...
    0.0; ...
    rotational_term * motor_gain / determinant; ...
    0.0; ...
    -mass_length * motor_gain / determinant];

c_matrix = eye(4);
d_matrix = zeros(4, 1);
end
