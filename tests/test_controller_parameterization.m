function tests = test_controller_parameterization
%TEST_CONTROLLER_PARAMETERIZATION Tests for legacy vector parameterization.
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function teardownOnce(test_case)
path(test_case.TestData.original_path);
end

function test_legacy_parameter_spaces_and_decoding(test_case)
plant = twsbr_params();
config = experiment_config("quick");
attitude_space = controller_parameter_space("ATTITUDE_PID");
cascade_space = controller_parameter_space("CASCADE_PID");

verifyEqual(test_case, attitude_space.name, "ATTITUDE_PID");
verifyEqual(test_case, attitude_space.parameter_names, ["kp"; "ki"; "kd"]);
verifyEqual(test_case, attitude_space.lower_bounds, [-1.0, -4.0, -2.0]);
verifyEqual(test_case, attitude_space.upper_bounds, [2.0, 1.0, 1.5]);
verifyEqual(test_case, attitude_space.dimension, 3);
verifyEqual(test_case, cascade_space.name, "CASCADE_PID");
verifyEqual(test_case, cascade_space.parameter_names, ...
    ["kp_x"; "ki_x"; "kd_x"; "kp_theta"; "kd_theta"]);
verifyEqual(test_case, cascade_space.lower_bounds, [-2.0, -4.0, -2.0, 0.0, -1.5]);
verifyEqual(test_case, cascade_space.upper_bounds, [0.5, 0.0, 1.0, 2.0, 1.5]);
verifyEqual(test_case, cascade_space.dimension, 5);

attitude = decode_controller_vector("ATTITUDE_PID", ...
    log10([1.9, 0.2, 0.18]), plant, config);
cascade = decode_controller_vector("CASCADE_PID", log10([0.24, ...
    0.0004, 0.19, 9.25, 1.01]), plant, config);
verifyEqual(test_case, attitude.kp, 1.9, "RelTol", 1e-12);
verifyEqual(test_case, attitude.u_max, plant.u_max);
verifyEqual(test_case, cascade.theta_reference_limit, deg2rad(12));
verifyEqual(test_case, cascade.sample_time, 0.01);
end

function test_legacy_vector_decoder_rejects_invalid_vectors(test_case)
plant = twsbr_params();
config = experiment_config("quick");

verifyError(test_case, @() decode_controller_vector("ATTITUDE_PID", ...
    [0, 0], plant, config), "twsbr:controller:invalid_vector");
verifyError(test_case, @() decode_controller_vector("ATTITUDE_PID", ...
    [2.01, 0, 0], plant, config), "twsbr:controller:invalid_vector");
verifyError(test_case, @() decode_controller_vector("CASCADE_PID", ...
    [0, 0, 0, 0, Inf], plant, config), "twsbr:controller:invalid_vector");
verifyError(test_case, @() decode_controller_vector("CASCADE_PID", ...
    [0, 0, 0, 0, 1i], plant, config), "twsbr:controller:invalid_vector");
end

function test_vector_decoder_rejects_unknown_controller_names(test_case)
verifyError(test_case, @() decode_controller_vector("MPC", [0, 0, 0], ...
    twsbr_params(), experiment_config("quick")), ...
    "twsbr:controller:unsupported_name");
end
