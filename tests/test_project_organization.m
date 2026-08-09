function tests = test_project_organization
%TEST_PROJECT_ORGANIZATION Verify public MATLAB sources use purpose folders.
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

function test_suite_setup_captures_original_path(test_case)
verifyTrue(test_case, isfield(test_case.TestData, "original_path"));
end

function test_root_matlab_files_match_public_entry_point_allowlist(test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
root_files = string({dir(fullfile(project_root, "*.m")).name});
expected_files = ["run_attitude_pid.m", "run_project.m", "setup_project.m"];

verifyEqual(test_case, sort(root_files), sort(expected_files));
end

function test_public_functions_resolve_from_purpose_directories(test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
paths = setup_project();

[function_names, directory_names] = public_source_locations();
for index = 1:numel(function_names)
    function_name = function_names(index);
    expected_directory = fullfile(project_root, directory_names(index));
    expected_file = fullfile(expected_directory, function_name + ".m");
    root_file = fullfile(project_root, function_name + ".m");

    verifyTrue(test_case, any(paths.code_directories == expected_directory));
    verifyEqual(test_case, string(which(function_name)), expected_file);
    verifyFalse(test_case, isfile(root_file));
    verifyEqual(test_case, string(which(function_name, "-all")), expected_file);
end
end

function test_resolved_non_simulink_functions_execute(test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));

plant_params = twsbr_params();
state_dot = twsbr_dynamics(0.0, zeros(4, 1), 0.0, plant_params, 0.0, 0.0);
[a_matrix, b_matrix, c_matrix, d_matrix] = twsbr_linear_model(plant_params);
[a_numerical, b_numerical] = twsbr_numerical_linearize(plant_params);
next_state = twsbr_rk4_step(zeros(4, 1), 0.0, 0.001, plant_params, 0.0, 0.0);
pid_params = attitude_pid_params(struct(), plant_params);
[control, controller_state] = attitude_pid_step( ...
    struct("integral_error", 0.0), 0.0, 0.0, 0.0, pid_params);
scenarios = attitude_pid_scenarios();
open_loop = simulate_open_loop(plant_params, zeros(4, 1), 0.01, false);
closed_loop = simulate_attitude_pid(plant_params, pid_params, scenarios.zero_state);

verifyEqual(test_case, state_dot, zeros(4, 1), "AbsTol", 1e-12);
verifyEqual(test_case, a_matrix, a_numerical, "AbsTol", 1e-7, "RelTol", 1e-6);
verifySize(test_case, b_matrix, [4, 1]);
verifyEqual(test_case, c_matrix, eye(4), "AbsTol", 1e-12);
verifyEqual(test_case, d_matrix, zeros(4, 1), "AbsTol", 1e-12);
verifyEqual(test_case, b_matrix, b_numerical, "AbsTol", 1e-7, "RelTol", 1e-6);
verifyEqual(test_case, next_state, zeros(4, 1), "AbsTol", 1e-12);
verifyEqual(test_case, control.u, 0.0, "AbsTol", 1e-12);
verifyEqual(test_case, controller_state.integral_error, 0.0, "AbsTol", 1e-12);
verifyTrue(test_case, isfield(scenarios, "zero_state"));
verifyEqual(test_case, open_loop.state, zeros(size(open_loop.state)), "AbsTol", 1e-12);
verifyTrue(test_case, closed_loop.success);
end

function [function_names, directory_names] = public_source_locations()
function_names = [ ...
    "twsbr_params"; "twsbr_dynamics"; "twsbr_linear_model"; ...
    "twsbr_numerical_linearize"; "twsbr_rk4_step"; ...
    "attitude_pid_params"; "attitude_pid_step"; "attitude_pid_scenarios"; ...
    "simulate_open_loop"; "simulate_attitude_pid"; ...
    "run_simulink_open_loop"; "run_attitude_pid_simulink"; ...
    "build_twsbr_simulink"; "build_attitude_pid_simulink"; ...
    "plot_attitude_pid_results"; ...
    "run_project_workflow"; "run_attitude_pid_workflow"];
directory_names = [ ...
    repmat("models", 5, 1); repmat("controllers", 2, 1); "scenarios"; ...
    repmat("simulation", 4, 1); repmat("builders", 2, 1); "visualization"; ...
    repmat("workflows", 2, 1)];
end

function test_private_simulink_creators_are_isolated_under_builders(test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
private_directory = fullfile(project_root, "builders", "private");
old_private_directory = fullfile(project_root, "private");
function_names = [ ...
    "create_twsbr_simulink_model"; ...
    "create_attitude_pid_simulink_model"];

verifyFalse(test_case, is_on_matlab_path(private_directory));
for index = 1:numel(function_names)
    file_name = function_names(index) + ".m";
    verifyTrue(test_case, isfile(fullfile(private_directory, file_name)));
    verifyFalse(test_case, isfile(fullfile(old_private_directory, file_name)));
end
end

function test_root_entry_points_initialize_project_paths(test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
original_path = path;
original_directory = string(pwd);
cleanup = onCleanup(@() restore_project_state( ...
    original_path, original_directory)); %#ok<NASGU>

[function_names, directory_names] = public_source_locations();
source_directories = fullfile(project_root, unique(directory_names));
for index = 1:numel(source_directories)
    if is_on_matlab_path(source_directories(index))
        rmpath(source_directories(index));
    end
end
for index = 1:numel(function_names)
    eval("clear " + function_names(index));
end
clear setup_project
cd(project_root);

plant_summary = run_project(false);
expected_plant_model = fullfile( ...
    project_root, "simulink_models", "twsbr_plant.slx");
verifyEqual(test_case, plant_summary.controllability_rank, 4);
verifyEqual(test_case, string(plant_summary.model_path), expected_plant_model);
for index = 1:numel(source_directories)
    verifyTrue(test_case, is_on_matlab_path(source_directories(index)));
end

pid_summary = run_attitude_pid(false);
verifyTrue(test_case, pid_summary.comparison.accepted);
verifyEqual(test_case, string(pid_summary.model_path), fullfile( ...
    project_root, "simulink_models", "twsbr_attitude_pid.slx"));
for index = 1:numel(source_directories)
    verifyTrue(test_case, is_on_matlab_path(source_directories(index)));
end
end

function restore_project_state(original_path, original_directory)
path(original_path);
cd(original_directory);
end

function result = is_on_matlab_path(entry)
path_entries = string(strsplit(path, pathsep));
result = any(path_entries == string(entry));
end
