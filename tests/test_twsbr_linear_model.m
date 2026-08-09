function tests = test_twsbr_linear_model
%TEST_TWSBR_LINEAR_MODEL Tests for plant linearization.
tests = functiontests(localfunctions);
end

function setupOnce(~)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function test_analytical_linearization_matches_numerical(test_case)
params = twsbr_params();
[a_analytical, b_analytical] = twsbr_linear_model(params);
[a_numerical, b_numerical] = twsbr_numerical_linearize(params);

verifyEqual(test_case, a_analytical, a_numerical, ...
    "AbsTol", 1e-7, "RelTol", 1e-6);
verifyEqual(test_case, b_analytical, b_numerical, ...
    "AbsTol", 1e-7, "RelTol", 1e-6);
end

function test_linearized_model_is_controllable(test_case)
params = twsbr_params();
[a_matrix, b_matrix, c_matrix, d_matrix] = twsbr_linear_model(params);
controllability = [b_matrix, a_matrix * b_matrix, ...
    a_matrix^2 * b_matrix, a_matrix^3 * b_matrix];

verifyEqual(test_case, rank(controllability), 4);
verifyEqual(test_case, c_matrix, eye(4), "AbsTol", 1e-12);
verifyEqual(test_case, d_matrix, zeros(4, 1), "AbsTol", 1e-12);
end
