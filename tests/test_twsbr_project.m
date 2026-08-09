function tests = test_twsbr_project
%TEST_TWSBR_PROJECT End to end project artifact test.
tests = functiontests(localfunctions);
end

function setupOnce(~)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function test_run_project_creates_expected_artifacts(test_case)
summary = run_project(false);

verifyTrue(test_case, isfile(summary.model_path));
verifyTrue(test_case, isfile(summary.results_path));
verifyTrue(test_case, isfile(summary.figure_path));
verifyGreaterThan(test_case, dir(summary.model_path).bytes, 0);
verifyGreaterThan(test_case, dir(summary.results_path).bytes, 0);
verifyGreaterThan(test_case, dir(summary.figure_path).bytes, 0);
verifyEqual(test_case, summary.controllability_rank, 4);
end
