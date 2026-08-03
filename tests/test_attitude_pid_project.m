function tests = test_attitude_pid_project
%TEST_ATTITUDE_PID_PROJECT End to end artifact generation test.
tests = functiontests(localfunctions);
end

function test_one_command_workflow_creates_complete_artifacts(test_case)
summary = run_attitude_pid(false);

verifyTrue(test_case, isfile(summary.model_path));
verifyTrue(test_case, isfile(summary.results_path));
verifyTrue(test_case, isfile(summary.figure_path));
verifyGreaterThan(test_case, dir(summary.model_path).bytes, 0);
verifyGreaterThan(test_case, dir(summary.results_path).bytes, 0);
verifyGreaterThan(test_case, dir(summary.figure_path).bytes, 0);
verifyEqual(test_case, summary.pid_params.kp, attitude_pid_params().kp);
verifyLessThan(test_case, summary.comparison.maximum_tilt_error_deg, 0.2);

scenario_names = fieldnames(attitude_pid_scenarios());
for index = 1:numel(scenario_names)
    verifyTrue(test_case, ...
        summary.acceptance.(scenario_names{index}).accepted, ...
        scenario_names{index});
end

saved_results = load(summary.results_path);
verifyTrue(test_case, isfield(saved_results, "plant_params"));
verifyTrue(test_case, isfield(saved_results, "pid_params"));
verifyTrue(test_case, isfield(saved_results, "matlab_simulations"));
verifyTrue(test_case, isfield(saved_results, "simulink_simulation"));
verifyTrue(test_case, isfield(saved_results, "comparison"));
verifyTrue(test_case, isfield(saved_results, "acceptance"));
end
