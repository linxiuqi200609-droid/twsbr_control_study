function tests = test_twsbr_simulink
%TEST_TWSBR_SIMULINK Integration tests for the Simulink plant.
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

function test_model_builds_updates_and_runs(test_case)
verifyNotEmpty(test_case, ver("simulink"));
model_path = build_twsbr_simulink();

verifyTrue(test_case, isfile(model_path));
simulation = run_simulink_open_loop(twsbr_params(), zeros(4, 1), 0.25);
verifyLessThan(test_case, max(abs(simulation.state), [], "all"), 1e-10);
end

function test_matlab_and_simulink_trajectories_agree(test_case)
params = twsbr_params();
initial_state = [0.0; 0.0; deg2rad(3.0); 0.0];
matlab_simulation = simulate_open_loop(params, initial_state, 1.0, false);
simulink_simulation = run_simulink_open_loop(params, initial_state, 1.0);

matlab_state = interp1(matlab_simulation.time, matlab_simulation.state, ...
    simulink_simulation.time, "pchip");
maximum_error = max(abs(matlab_state - simulink_simulation.state), [], "all");

verifyLessThan(test_case, maximum_error, 1e-4);
end
