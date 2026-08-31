function tests = test_controller_nonfinite_output
%TEST_CONTROLLER_NONFINITE_OUTPUT Arithmetic failure versus invalid input.
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
addpath(fileparts(fileparts(mfilename("fullpath"))));
setup_project();
end

function teardownOnce(test_case)
path(test_case.TestData.original_path);
end

function test_finite_overflow_uses_common_identifier_for_new_controllers(test_case)
names = ["LQR", "LQI", "FUZZY_PID"];
for name = names
    controller = controller_fixture(name);
    verifyError(test_case, @() controller_step(controller, 0, ...
        [0;0;0;realmax], 0), "twsbr:controller:nonfinite_output");
    verifyError(test_case, @() controller_step(controller, 0, ...
        [0;0;0;Inf], 0), "twsbr:controller:invalid_state");
end
verifyError(test_case, @() lqr_step([realmax;0;0;0], 0, ...
    struct("gain",[2,0,0,0])), "twsbr:controller:nonfinite_output");
verifyError(test_case, @() lqr_step([NaN;0;0;0], 0, ...
    struct("gain",[2,0,0,0])), "twsbr:lqr:invalid_input");
end

function test_simulator_retains_new_controller_overflow_failures(test_case)
scenarios = training_scenarios(3.2);
scenario = scenarios.T1_initial_tilt_5deg;
scenario.duration = 0.04;
scenario.initial_state = [0;0;0;realmax];
for name = ["LQR", "LQI", "FUZZY_PID"]
    [~, vector] = controller_fixture(name);
    simulation = simulate_control_system(name, vector, twsbr_params(), ...
        experiment_config("quick"), scenario, 0);
    verifyFalse(test_case, simulation.success);
    verifyEqual(test_case, simulation.failure_reason, "nonfinite_control");
    metrics = calculate_control_metrics(simulation, scenario, twsbr_params());
    verifyFalse(test_case, metrics.success);
    verifyEqual(test_case, metrics.failure_reason, "nonfinite_control");
end
end

function [controller, vector] = controller_fixture(name)
switch name
    case "LQR"
        vector = log10([10,1,200,10,0.1]);
    case "LQI"
        vector = log10([10,1,200,10,100,0.1]);
    otherwise
        vector = [log10([0.241,0.000396,0.193,9.255,0.05,2]),0.2,0.2,0.2];
end
controller = reset_controller(create_controller(name, vector, ...
    twsbr_params(), experiment_config("quick")));
end
