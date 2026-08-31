function tests = test_environment_dependencies
%TEST_ENVIRONMENT_DEPENDENCIES Process-local absent dependency fixtures.
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

function test_require_statistics_rejects_nonlogical_and_nonscalar_flags(test_case)
root = string(fileparts(fileparts(mfilename("fullpath"))));
for value = {0, 1, [], [true,false], "true", NaN, struct(), {true}}
    verifyError(test_case, @() validate_environment(root, value{1}), ...
        "twsbr:environment:invalid_require_statistics");
end
end

function test_missing_required_toolboxes_have_specific_errors(test_case)
root = string(fileparts(fileparts(mfilename("fullpath"))));
toolboxes = ["simulink", "control", "stats"];
identifiers = ["twsbr:environment:simulink_required", ...
    "twsbr:environment:control_required", "twsbr:environment:statistics_required"];
for index = 1:3
    cleanup = absent_dependency("ver", toolboxes(index));
    verifyError(test_case, @() validate_environment(root, true), identifiers(index));
    if index == 3
        report = validate_environment(root, false);
        verifyTrue(test_case, report.accepted);
        verifyFalse(test_case, report.statistics_available);
    end
    delete(cleanup)
end
report = validate_environment(root, true);
verifyTrue(test_case, report.accepted);
end

function test_valid_source_root_runs_without_git_and_unrelated_root_is_rejected(test_case)
root = string(fileparts(fileparts(mfilename("fullpath"))));
cleanup = absent_dependency("system", "git");
report = validate_environment(root, false);
verifyTrue(test_case, report.accepted);
verifyEqual(test_case, report.git_commit, "unavailable");
verifyError(test_case, @() validate_environment(tempdir, false), ...
    "twsbr:environment:invalid_project_root");
delete(cleanup)
report = validate_environment(root, false);
verifyNotEqual(test_case, string(report.git_commit), "unavailable");
end

function test_workflow_without_git_preserves_unknown_provenance(test_case)
cleanup = absent_dependency("system", "git");
output = string(tempname);
output_cleanup = onCleanup(@() remove_output(output));
vectors = struct("ATTITUDE_PID", log10([1.9,0.2,0.18]), ...
    "CASCADE_PID", log10([0.241,0.000396,0.193,9.255,1.011]), ...
    "FUZZY_PID", [log10([0.241,0.000396,0.193,9.255,0.05,1.011]),0.2,0.2,0.2], ...
    "LQR", log10([10,1,200,10,0.1]), "LQI", log10([10,1,200,10,100,0.1]));
summary = run_control_study_workflow("quick", false, struct( ...
    "frozen_vectors", vectors, "run_monte_carlo", false, ...
    "run_simulink", false, "require_statistics", false, "generate_figures", false, ...
    "training_duration", 3.2, "test_duration", 4.2, "output_root", output));
manifest = jsondecode(fileread(summary.manifest_path));
verifyEqual(test_case, string(manifest.git_commit), "unavailable");
verifyEqual(test_case, string(manifest.context.source_dirty), "unavailable");
verifyEqual(test_case, summary.stage_status.deterministic.state, "completed");
delete(output_cleanup)
delete(cleanup)
end

function cleanup = absent_dependency(function_name, dependency)
directory = string(tempname);
mkdir(directory);
original_path = path;
original_warnings = warning;
cleanup = onCleanup(@() restore_dependencies(directory, original_path, original_warnings));
if function_name == "system"
    lines = ["function varargout = system(varargin)"; ...
        "if startsWith(strtrim(string(varargin{1})), 'git ')"; ...
        "    varargout = {127, 'git unavailable in local test fixture'};"; ...
        "else"; "    [varargout{1:nargout}] = builtin('system', varargin{:});"; ...
        "end"; "end"];
else
    lines = ["function result = ver(name)"; ...
        "if string(name) == """ + dependency + """"; ...
        "    result = [];"; "else"; ...
        "    result = struct('Name', string(name));"; "end"; "end"];
end
file = fopen(fullfile(directory, function_name + ".m"), "w");
file_cleanup = onCleanup(@() fclose(file));
fprintf(file, "%s\n", lines);
delete(file_cleanup)
warning("off", "MATLAB:dispatcher:nameConflict");
addpath(directory, "-begin");
clear system ver
end

function restore_dependencies(directory, original_path, original_warnings)
path(original_path);
clear system ver
warning(original_warnings);
rmdir(directory, "s");
end

function remove_output(output)
if isfolder(output)
    rmdir(output, "s");
end
end
