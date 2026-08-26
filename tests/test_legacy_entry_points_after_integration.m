function tests = test_legacy_entry_points_after_integration
%TEST_LEGACY_ENTRY_POINTS_AFTER_INTEGRATION Guard public workflow artifacts.
tests = functiontests(localfunctions);
end

function test_entry_points_create_expected_artifacts_from_external_directory( ...
        test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
original_path = path;
original_directory = string(pwd);
external_directory = string(tempname(tempdir));
mkdir(external_directory);
cleanup = onCleanup(@() restore_test_state( ...
    original_path, original_directory, external_directory));

restoredefaultpath;
addpath(project_root, "-begin");
cd(external_directory);
clear setup_project run_project run_attitude_pid run_cascade_pid ...
    run_project_workflow run_attitude_pid_workflow ...
    run_cascade_pid_workflow

plant_summary = run_project(false);
verify_summary_artifacts(test_case, plant_summary, project_root, ...
    "twsbr_plant.slx", "open_loop_results.mat", ...
    "open_loop_response.png");
verifyEqual(test_case, string(pwd), external_directory);

attitude_summary = run_attitude_pid(false);
verify_summary_artifacts(test_case, attitude_summary, project_root, ...
    "twsbr_attitude_pid.slx", "attitude_pid_results.mat", ...
    "attitude_pid_response.png");
verifyEqual(test_case, string(pwd), external_directory);

cascade_summary = run_cascade_pid(false);
verify_summary_artifacts(test_case, cascade_summary, project_root, ...
    "twsbr_cascade_pid.slx", "cascade_pid_results.mat", ...
    "cascade_pid_response.png");
verifyEqual(test_case, string(pwd), external_directory);

clear cleanup
verifyEqual(test_case, string(pwd), original_directory);
verifyEqual(test_case, path, original_path);
verifyFalse(test_case, isfolder(external_directory));
end

function verify_summary_artifacts(test_case, summary, project_root, ...
        model_name, results_name, figure_name)
expected_model_path = fullfile(project_root, "simulink_models", model_name);
expected_results_path = fullfile(project_root, "results", results_name);
expected_figure_path = fullfile(project_root, "results", figure_name);

verifyEqual(test_case, string(summary.model_path), expected_model_path);
verifyEqual(test_case, string(summary.results_path), expected_results_path);
verifyEqual(test_case, string(summary.figure_path), expected_figure_path);
verifyTrue(test_case, isfile(summary.model_path));
verifyTrue(test_case, isfile(summary.results_path));
verifyTrue(test_case, isfile(summary.figure_path));
end

function restore_test_state(original_path, original_directory, external_directory)
path(original_path);
cd(original_directory);
if isfolder(external_directory)
    rmdir(external_directory, "s");
end
end
