function tests = test_root_entry_points
%TEST_ROOT_ENTRY_POINTS Verify stable wrappers bootstrap from outside the project.
tests = functiontests(localfunctions);
end

function test_wrappers_run_outside_project_with_only_root_on_initial_path(test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
original_path = path;
original_directory = string(pwd);
external_directory = string(tempname);
mkdir(external_directory);
cleanup = onCleanup(@() restore_test_state( ...
    original_path, original_directory, external_directory));

restoredefaultpath;
addpath(project_root, "-begin");
cd(external_directory);
clear setup_project run_project run_attitude_pid run_cascade_pid ...
    run_project_workflow run_attitude_pid_workflow ...
    run_cascade_pid_workflow run_control_study run_control_study_workflow

plant_summary = run_project(false);
verifyEqual(test_case, string(pwd), external_directory);
verifyEqual(test_case, plant_summary.controllability_rank, 4);
verifyEqual(test_case, string(plant_summary.model_path), fullfile( ...
    project_root, "simulink_models", "twsbr_plant.slx"));
verifyEqual(test_case, string(which("run_project_workflow")), fullfile( ...
    project_root, "workflows", "run_project_workflow.m"));

pid_summary = run_attitude_pid(false);
verifyEqual(test_case, string(pwd), external_directory);
verifyTrue(test_case, pid_summary.comparison.accepted);
verifyEqual(test_case, string(pid_summary.model_path), fullfile( ...
    project_root, "simulink_models", "twsbr_attitude_pid.slx"));
verifyEqual(test_case, string(which("run_attitude_pid_workflow")), fullfile( ...
    project_root, "workflows", "run_attitude_pid_workflow.m"));

cascade_summary = run_cascade_pid(false);
verifyEqual(test_case, string(pwd), external_directory);
verifyTrue(test_case, all(structfun( ...
    @(result) result.accepted, cascade_summary.acceptance)));
verifyTrue(test_case, cascade_summary.comparison.accepted);
verifyEqual(test_case, string(cascade_summary.model_path), fullfile( ...
    project_root, "simulink_models", "twsbr_cascade_pid.slx"));
verifyEqual(test_case, string(cascade_summary.results_path), fullfile( ...
    project_root, "results", "cascade_pid_results.mat"));
verifyEqual(test_case, string(cascade_summary.figure_path), fullfile( ...
    project_root, "results", "cascade_pid_response.png"));
verifyTrue(test_case, isfile(cascade_summary.results_path));
verifyTrue(test_case, isfile(cascade_summary.figure_path));
verifyEqual(test_case, string(which("run_cascade_pid_workflow")), fullfile( ...
    project_root, "workflows", "run_cascade_pid_workflow.m"));

control_output = string(tempname);
cleanup_output = onCleanup(@() remove_directory(control_output));
control_summary = run_control_study("quick", false, struct( ...
    "frozen_vectors", root_entry_starter_vectors(), ...
    "run_monte_carlo", false, "run_simulink", false, ...
    "require_statistics", false, "generate_figures", false, ...
    "output_root", control_output));
verifyEqual(test_case, string(pwd), external_directory);
verifyEqual(test_case, control_summary.mode, "quick");
verifyTrue(test_case, isfolder(control_summary.output_root));
verifyEqual(test_case, string(which("run_control_study_workflow")), fullfile( ...
    project_root, "workflows", "run_control_study_workflow.m"));
delete(cleanup_output);
end

function vectors = root_entry_starter_vectors()
vectors = struct();
vectors.ATTITUDE_PID = log10([1.9, 0.2, 0.18]);
vectors.CASCADE_PID = log10([0.241, 0.000396, 0.193, 9.255, 1.011]);
vectors.FUZZY_PID = [log10([0.241, 0.000396, 0.193, ...
    9.255, 0.05, 1.011]), 0.2, 0.2, 0.2];
vectors.LQR = log10([10, 1, 200, 10, 0.1]);
vectors.LQI = log10([10, 1, 200, 10, 100, 0.1]);
end

function remove_directory(path)
if isfolder(path)
    rmdir(path, "s");
end
end

function restore_test_state(original_path, original_directory, external_directory)
path(original_path);
cd(original_directory);
if isfolder(external_directory)
    rmdir(external_directory, "s");
end
end
