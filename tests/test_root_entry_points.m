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
    original_path, original_directory, external_directory)); %#ok<NASGU>

restoredefaultpath;
addpath(project_root, "-begin");
cd(external_directory);
clear setup_project run_project run_attitude_pid ...
    run_project_workflow run_attitude_pid_workflow

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
end

function restore_test_state(original_path, original_directory, external_directory)
path(original_path);
cd(original_directory);
if isfolder(external_directory)
    rmdir(external_directory, "s");
end
end
