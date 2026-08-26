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
artifact_paths = known_artifact_paths(project_root);
[backup_directory, backup_paths, artifact_was_present, ...
    original_artifact_bytes] = backup_artifacts(artifact_paths);
cleanup = onCleanup(@() restore_test_state( ...
    original_path, original_directory, external_directory, ...
    artifact_paths, backup_paths, artifact_was_present, backup_directory));
remove_exact_artifacts(artifact_paths);
verifyFalse(test_case, any(isfile(artifact_paths)), ...
    "Tracked artifacts must be absent before the workflows run.");

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
verifyFalse(test_case, isfolder(backup_directory));
verify_restored_artifacts(test_case, artifact_paths, ...
    artifact_was_present, original_artifact_bytes);
end

function test_restore_failure_preserves_backup_for_manual_recovery(test_case)
fixture_root = string(tempname(tempdir));
mkdir(fixture_root);
fixture_cleanup = onCleanup(@() remove_temp_directory(fixture_root));
external_directory = fullfile(fixture_root, "external");
backup_directory = fullfile(fixture_root, "backup");
mkdir(external_directory);
mkdir(backup_directory);

original_bytes = uint8([0; 1; 2; 255]);
backup_path = fullfile(backup_directory, "original.bin");
write_file_bytes(backup_path, original_bytes);
blocking_parent = fullfile(fixture_root, "blocking_parent");
write_file_bytes(blocking_parent, uint8(7));
artifact_path = fullfile(blocking_parent, "original.bin");

caught_exception = [];
try
    restore_test_state(path, string(pwd), external_directory, ...
        artifact_path, backup_path, true, backup_directory);
catch exception
    caught_exception = exception;
end

assertNotEmpty(test_case, caught_exception);
verifyEqual(test_case, string(caught_exception.identifier), ...
    "twsbr:test:artifact_restore_recovery_required");
verifyTrue(test_case, contains(string(caught_exception.message), ...
    backup_directory));
verifyEqual(test_case, numel(caught_exception.cause), 1);
if ~isempty(caught_exception.cause)
    verifyEqual(test_case, string(caught_exception.cause{1}.identifier), ...
        "twsbr:test:artifact_restore_failed");
end
verifyTrue(test_case, isfolder(backup_directory));
backup_exists = isfile(backup_path);
verifyTrue(test_case, backup_exists);
if backup_exists
    verifyEqual(test_case, read_file_bytes(backup_path), original_bytes);
end

clear fixture_cleanup
verifyFalse(test_case, isfolder(fixture_root));
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

function artifact_paths = known_artifact_paths(project_root)
artifact_paths = [ ...
    fullfile(project_root, "results", "open_loop_results.mat"); ...
    fullfile(project_root, "results", "open_loop_response.png"); ...
    fullfile(project_root, "results", "attitude_pid_results.mat"); ...
    fullfile(project_root, "results", "attitude_pid_response.png"); ...
    fullfile(project_root, "results", "cascade_pid_results.mat"); ...
    fullfile(project_root, "results", "cascade_pid_response.png"); ...
    fullfile(project_root, "simulink_models", "twsbr_plant.slx"); ...
    fullfile(project_root, "simulink_models", "twsbr_attitude_pid.slx"); ...
    fullfile(project_root, "simulink_models", "twsbr_cascade_pid.slx")];
end

function [backup_directory, backup_paths, artifact_was_present, ...
        original_artifact_bytes] = backup_artifacts(artifact_paths)
backup_directory = string(tempname(tempdir));
mkdir(backup_directory);
backup_paths = strings(size(artifact_paths));
artifact_was_present = isfile(artifact_paths);
original_artifact_bytes = cell(size(artifact_paths));

try
    for index = 1:numel(artifact_paths)
        [~, file_name, extension] = fileparts(artifact_paths(index));
        backup_paths(index) = fullfile(backup_directory, ...
            string(index) + "_" + string(file_name) + string(extension));
        if artifact_was_present(index)
            original_artifact_bytes{index} = read_file_bytes( ...
                artifact_paths(index));
            [copied, message] = copyfile(artifact_paths(index), ...
                backup_paths(index));
            if ~copied
                error("twsbr:test:artifact_backup_failed", ...
                    "Could not back up %s: %s", artifact_paths(index), message);
            end
        end
    end
catch exception
    if isfolder(backup_directory)
        rmdir(backup_directory, "s");
    end
    rethrow(exception);
end
end

function remove_exact_artifacts(artifact_paths)
for index = 1:numel(artifact_paths)
    if isfile(artifact_paths(index))
        delete(artifact_paths(index));
    end
end
end

function restore_test_state(original_path, original_directory, ...
        external_directory, artifact_paths, backup_paths, ...
        artifact_was_present, backup_directory)
first_exception = [];
artifact_restore_exception = [];
try
    path(original_path);
catch exception
    first_exception = retain_first_exception(first_exception, exception);
end
try
    cd(original_directory);
catch exception
    first_exception = retain_first_exception(first_exception, exception);
end
for index = 1:numel(artifact_paths)
    try
        if artifact_was_present(index)
            [copied, message] = copyfile(backup_paths(index), ...
                artifact_paths(index), "f");
            if ~copied
                error("twsbr:test:artifact_restore_failed", ...
                    "Could not restore %s: %s", artifact_paths(index), message);
            end
            restored_bytes = read_file_bytes(artifact_paths(index));
            backup_bytes = read_file_bytes(backup_paths(index));
            if ~isequal(restored_bytes, backup_bytes)
                error("twsbr:test:artifact_restore_verification_failed", ...
                    "Restored artifact does not match backup: %s", ...
                    artifact_paths(index));
            end
        elseif isfile(artifact_paths(index))
            delete(artifact_paths(index));
        end
    catch exception
        first_exception = retain_first_exception(first_exception, exception);
        artifact_restore_exception = retain_first_exception( ...
            artifact_restore_exception, exception);
    end
end
try
    if isfolder(external_directory)
        rmdir(external_directory, "s");
    end
catch exception
    first_exception = retain_first_exception(first_exception, exception);
end
if ~isempty(artifact_restore_exception)
    recovery_exception = MException( ...
        'twsbr:test:artifact_restore_recovery_required', ...
        'Artifact restoration failed. Backup retained for manual recovery: %s', ...
        char(backup_directory));
    recovery_exception = addCause(recovery_exception, ...
        artifact_restore_exception);
    throw(recovery_exception);
end
try
    if isfolder(backup_directory)
        rmdir(backup_directory, "s");
    end
catch exception
    first_exception = retain_first_exception(first_exception, exception);
end
if ~isempty(first_exception)
    throw(first_exception);
end
end

function first_exception = retain_first_exception(first_exception, exception)
if isempty(first_exception)
    first_exception = exception;
end
end

function verify_restored_artifacts(test_case, artifact_paths, ...
        artifact_was_present, original_artifact_bytes)
for index = 1:numel(artifact_paths)
    if artifact_was_present(index)
        verifyTrue(test_case, isfile(artifact_paths(index)));
        verifyEqual(test_case, read_file_bytes(artifact_paths(index)), ...
            original_artifact_bytes{index});
    else
        verifyFalse(test_case, isfile(artifact_paths(index)));
    end
end
end

function bytes = read_file_bytes(file_path)
file_id = fopen(file_path, "rb");
if file_id == -1
    error("twsbr:test:artifact_read_failed", ...
        "Could not read artifact: %s", file_path);
end
file_cleanup = onCleanup(@() fclose(file_id));
bytes = fread(file_id, Inf, "*uint8");
clear file_cleanup
end

function write_file_bytes(file_path, bytes)
file_id = fopen(file_path, "wb");
if file_id == -1
    error("twsbr:test:artifact_write_failed", ...
        "Could not write test artifact: %s", file_path);
end
file_cleanup = onCleanup(@() fclose(file_id));
written_count = fwrite(file_id, bytes, "uint8");
if written_count ~= numel(bytes)
    error("twsbr:test:artifact_write_failed", ...
        "Could not write all test artifact bytes: %s", file_path);
end
clear file_cleanup
end

function remove_temp_directory(directory_path)
if isfolder(directory_path)
    rmdir(directory_path, "s");
end
end
