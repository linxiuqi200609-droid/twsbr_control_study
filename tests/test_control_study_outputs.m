function tests = test_control_study_outputs
%TEST_CONTROL_STUDY_OUTPUTS Verify unified study reporting contracts.
functions = localfunctions;
names = string(cellfun(@func2str, functions, "UniformOutput", false));
tests = functiontests(functions(startsWith(names, "test_")));
end

function test_reporters_create_expected_artifacts(test_case)
root = string(tempname);
mkdir(root);
cleanup = onCleanup(@() rmdir(root, "s"));
[data, ~, vectors, validation] = synthetic_reporting_inputs();
paths = write_results_tables(root, data, data, data, data, data, validation);
manifest_path = build_run_manifest(root, experiment_config("quick"), ...
    objective_config(), vectors, "test-commit");
verifyTrue(test_case, isfile(paths.statistics_xlsx));
verifyTrue(test_case, isfile(paths.deterministic_csv));
verifyTrue(test_case, isfile(paths.monte_carlo_csv));
verifyTrue(test_case, isfile(manifest_path));
manifest = jsondecode(fileread(manifest_path));
verifyEqual(test_case, string(manifest.git_commit), "test-commit");
end

function test_reporter_optional_defaults_are_typed_and_headered(test_case)
root = string(tempname);
mkdir(root);
cleanup = onCleanup(@() rmdir(root, "s"));
[data, ~, ~, validation] = synthetic_reporting_inputs();
paths = write_results_tables(root, data, data, data, data, data, validation);
omnibus = readtable(paths.statistics_omnibus_csv);
pairwise = readtable(paths.statistics_pairwise_csv);
workbook_omnibus = readtable(paths.statistics_xlsx, "Sheet", "omnibus_tests");
workbook_pairwise = readtable(paths.statistics_xlsx, "Sheet", "pairwise_tests");
verifyEmpty(test_case, omnibus);
verifyEmpty(test_case, pairwise);
verifyEmpty(test_case, workbook_omnibus);
verifyEmpty(test_case, workbook_pairwise);
verifyTrue(test_case, ismember("metric", string(omnibus.Properties.VariableNames)));
verifyTrue(test_case, ismember("metric", string(pairwise.Properties.VariableNames)));
verifyTrue(test_case, ismember("metric", ...
    string(workbook_omnibus.Properties.VariableNames)));
end

function test_raw_simulation_round_trip_uses_safe_relative_index(test_case)
root = string(tempname);
mkdir(root);
cleanup = onCleanup(@() rmdir(root, "s"));
raw_directory = fullfile(root, "raw");
mkdir(raw_directory);
simulation = struct("time", [0; 0.1], "state", zeros(2, 4));
index = save_raw_simulation(root, raw_directory, "CASCADE_PID", ...
    "S2_position_step_0p75m", simulation);
verifyTrue(test_case, startsWith(string(index.relative_path), "raw" + filesep));
verifyFalse(test_case, contains(string(index.relative_path), ".."));
loaded = load(fullfile(root, index.relative_path));
verifyEqual(test_case, loaded.simulation, simulation);
end

function test_raw_simulation_rejects_sanitized_name_collision(test_case)
root = string(tempname);
mkdir(root);
cleanup = onCleanup(@() rmdir(root, "s"));
raw_directory = fullfile(root, "raw");
mkdir(raw_directory);
save_raw_simulation(root, raw_directory, "A/B", "scenario", struct("value", 1));
verifyError(test_case, @() save_raw_simulation(root, raw_directory, ...
    "A?B", "scenario", struct("value", 2)), ...
    "twsbr:reporting:RawNameCollision");
end

function test_scenario_record_preserves_metadata_and_removes_handles(test_case)
scenario = heldout_scenarios(4.2).S2_position_step_0p75m;
scenario.future_numeric_metadata = 42;
record = scenario_to_record(scenario);
verifyFalse(test_case, isfield(record, "x_reference"));
verifyFalse(test_case, isfield(record, "force_disturbance"));
verifyFalse(test_case, isfield(record, "torque_disturbance"));
verifyEqual(test_case, record.future_numeric_metadata, 42);
verifyEqual(test_case, record.reference_amplitude, 0.75);
end

function test_json_writer_round_trips_utf8(test_case)
path = string(tempname) + ".json";
cleanup = onCleanup(@() delete_if_present(path));
write_json_file(path, struct("label", "M\u00fcnchen", "value", 3));
record = jsondecode(fileread(path));
verifyEqual(test_case, string(record.label), "M\u00fcnchen");
verifyEqual(test_case, record.value, 3);
end

function test_manifest_hashes_actual_configuration_files(test_case)
root = string(tempname);
mkdir(root);
cleanup = onCleanup(@() rmdir(root, "s"));
vectors = starter_vectors_for_test();
manifest_path = build_run_manifest(root, experiment_config("quick"), ...
    objective_config(), vectors, "test-commit");
manifest = jsondecode(fileread(manifest_path));
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
verifyEqual(test_case, string(manifest.configuration_hashes.experiment_config), ...
    sha256_for_test(fullfile(project_root, "config", "experiment_config.m")));
verifyEqual(test_case, string(manifest.configuration_hashes.objective_config), ...
    sha256_for_test(fullfile(project_root, "config", "objective_config.m")));
verifyEqual(test_case, string(manifest.configuration_hashes.monte_carlo_config), ...
    sha256_for_test(fullfile(project_root, "config", "monte_carlo_config.m")));
end

function test_root_wrapper_delegates_to_workflow(test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
output_root = string(tempname);
cleanup = onCleanup(@() rmdir_if_present(output_root));
summary = run_control_study("quick", false, ...
    struct("frozen_vectors", starter_vectors_for_test(), ...
    "run_monte_carlo", false, "run_simulink", false, ...
    "require_statistics", false, "generate_figures", false, ...
    "output_root", output_root));
verifyEqual(test_case, summary.mode, "quick");
verifyTrue(test_case, isfolder(summary.output_root));
verifyEqual(test_case, summary.stage_status.monte_carlo.state, "skipped");
verifyEqual(test_case, summary.stage_status.simulink.state, "skipped");
end

function test_root_wrapper_rejects_invalid_public_inputs(test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
verifyError(test_case, @() run_control_study("invalid", false), ...
    "twsbr:study:invalid_mode");
verifyError(test_case, @() run_control_study("quick", 0), ...
    "twsbr:study:invalid_test_flag");
verifyError(test_case, @() run_control_study("quick", false, ...
    struct("unknown_option", true)), "twsbr:study:invalid_options");
end

function test_skipped_monte_carlo_replaces_stale_managed_output(test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
output_root = string(tempname);
mkdir(fullfile(output_root, "monte_carlo"));
stale_path = fullfile(output_root, "monte_carlo", "monte_carlo_metrics.csv");
writetable(table("stale", 'VariableNames', {'controller'}), stale_path);
cleanup = onCleanup(@() rmdir_if_present(output_root));
summary = run_control_study("quick", false, struct( ...
    "frozen_vectors", starter_vectors_for_test(), ...
    "run_monte_carlo", false, "run_simulink", false, ...
    "require_statistics", false, "generate_figures", false, ...
    "output_root", output_root));
current = readtable(summary.paths.monte_carlo_csv);
verifyEmpty(test_case, current);
verifyEqual(test_case, summary.stage_status.monte_carlo.reason, ...
    "disabled_by_option");
end

function test_skipped_stages_return_typed_empty_outputs_and_dirty_manifest(test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
output_root = string(tempname);
cleanup = onCleanup(@() rmdir_if_present(output_root));
summary = run_control_study("quick", false, struct( ...
    "frozen_vectors", starter_vectors_for_test(), ...
    "run_monte_carlo", false, "run_simulink", false, ...
    "require_statistics", false, "generate_figures", false, ...
    "output_root", output_root));
verifyTrue(test_case, ismember("controller", ...
    string(summary.monte_carlo.Properties.VariableNames)));
verifyTrue(test_case, ismember("accepted", ...
    string(summary.validation.Properties.VariableNames)));
verifyTrue(test_case, ismember("metric", ...
    string(summary.omnibus.Properties.VariableNames)));
verifyTrue(test_case, ismember("metric", ...
    string(summary.pairwise.Properties.VariableNames)));
manifest = jsondecode(fileread(summary.manifest_path));
verifyTrue(test_case, isfield(manifest.context, "source_dirty"));
verifyTrue(test_case, islogical(manifest.context.source_dirty));
end

function test_enabled_simulink_rejection_persists_failure_before_error(test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
output_root = string(tempname);
mock_root = string(tempname);
mkdir(mock_root);
write_rejected_validation_batch(mock_root);
addpath(mock_root, "-begin");
cleanup = onCleanup(@() remove_mock_validation_batch(mock_root, output_root));
clear run_simulink_validation_batch

verifyError(test_case, @() run_control_study("quick", false, struct( ...
    "frozen_vectors", starter_vectors_for_test(), ...
    "run_monte_carlo", false, "run_simulink", true, ...
    "require_statistics", false, "generate_figures", false, ...
    "output_root", output_root)), "twsbr:study:simulink_validation_failed");

validation_path = fullfile(output_root, "simulink_validation", ...
    "equivalence_summary.csv");
verifyTrue(test_case, isfile(validation_path));
validation = readtable(validation_path);
verifyEqual(test_case, height(validation), 5);
verifyEqual(test_case, sum(validation.accepted), 4);
manifest = jsondecode(fileread(fullfile(output_root, "run_manifest.json")));
verifyEqual(test_case, string(manifest.context.stage_status.simulink.state), ...
    "failed");
verifyEqual(test_case, string(manifest.context.stage_status.simulink.reason), ...
    "rejected_comparison");
verifyEqual(test_case, manifest.context.equivalence_accepted_count, 4);
end

function test_statistics_workbook_widens_text_columns(test_case)
root = string(tempname);
mkdir(root);
cleanup = onCleanup(@() rmdir(root, "s"));
[data, ~, ~, validation] = synthetic_reporting_inputs();
paths = write_results_tables(root, data, data, data, data, data, validation);
widths = worksheet_column_widths(paths.statistics_xlsx, 1);
verifyGreaterThanOrEqual(test_case, widths(1), 14);
verifyGreaterThanOrEqual(test_case, widths(2), 26);
verifyEqual(test_case, workbook_sheet_count(paths.statistics_xlsx), 11);
end

function test_header_only_workbook_has_widths_for_numeric_headers_and_readback(test_case)
root = string(tempname);
mkdir(root);
cleanup = onCleanup(@() rmdir(root, "s"));
data = table(strings(0,1), zeros(0,1), 'VariableNames', ...
    {'controller', 'controller_completed_count'});
paths = write_results_tables(root, data, data, data, data, data, data);
widths = worksheet_column_widths(paths.statistics_xlsx, 1);
verifyEqual(test_case, numel(widths), 2);
if numel(widths) == 2
    verifyGreaterThanOrEqual(test_case, widths(1), 14);
    verifyGreaterThanOrEqual(test_case, widths(2), 30);
end
readback = readtable(paths.statistics_xlsx, "Sheet", "deterministic_raw");
verifyEmpty(test_case, readback);
verifyEqual(test_case, readback.Properties.VariableNames, data.Properties.VariableNames);

data = table("LQR", 0.123456789012345, NaN, 'VariableNames', ...
    {'controller', 'controller_runtime_seconds', 'mean_step_runtime_us'});
paths = write_results_tables(root, data, data, data, data, data, data);
readback = readtable(paths.statistics_xlsx, "Sheet", "deterministic_raw");
verifyEqual(test_case, readback.controller_runtime_seconds, data.controller_runtime_seconds);
verifyTrue(test_case, isnan(readback.mean_step_runtime_us));
widths = worksheet_column_widths(paths.statistics_xlsx, 1);
verifyGreaterThanOrEqual(test_case, widths(2), 30);
end

function [data, raw, vectors, validation] = synthetic_reporting_inputs()
data = table("CASCADE_PID", "S2_position_step_0p75m", true, ...
    0.2, 1.0, 0.5, 0.01, 'VariableNames', ...
    {'controller','scenario','success','theta_rms_deg', ...
    'position_itae','control_energy','saturation_ratio'});
raw = struct();
vectors = starter_vectors_for_test();
validation = table("CASCADE_PID", true, 'VariableNames', ...
    {'controller','accepted'});
end

function vectors = starter_vectors_for_test()
vectors = struct();
vectors.ATTITUDE_PID = log10([1.9,0.2,0.18]);
vectors.CASCADE_PID = log10([0.241,0.000396,0.193,9.255,1.011]);
vectors.FUZZY_PID = [log10([0.241,0.000396,0.193, ...
    9.255,0.05,1.011]),0.2,0.2,0.2];
vectors.LQR = log10([10,1,200,10,0.1]);
vectors.LQI = log10([10,1,200,10,100,0.1]);
end

function digest = sha256_for_test(path)
message_digest = java.security.MessageDigest.getInstance("SHA-256");
message_digest.update(uint8(fileread(path)));
bytes = typecast(message_digest.digest(), "uint8");
digest = lower(string(reshape(dec2hex(bytes, 2).', 1, [])));
end

function delete_if_present(path)
if isfile(path)
    delete(path);
end
end

function rmdir_if_present(path)
if isfolder(path)
    rmdir(path, "s");
end
end

function write_rejected_validation_batch(mock_root)
path_value = fullfile(mock_root, "run_simulink_validation_batch.m");
file_identifier = fopen(path_value, "w", "n", "UTF-8");
cleanup = onCleanup(@() fclose(file_identifier));
lines = [ ...
    "function validation = run_simulink_validation_batch(~, ~, ~)"; ...
    "controller = [""ATTITUDE_PID""; ""CASCADE_PID""; ""FUZZY_PID""; ""LQR""; ""LQI""];"; ...
    "scenario = repmat(""mock_scenario"", 5, 1);"; ...
    "accepted = [true; true; true; true; false];"; ...
    "validation = table(controller, scenario, zeros(5,1), zeros(5,1), ..."; ...
    "    zeros(5,1), ones(5,1), ones(5,1), ones(5,1), zeros(5,1), ..."; ...
    "    true(5,1), accepted, 'VariableNames', {'controller', 'scenario', ..."; ...
    "    'max_tilt_difference_deg', 'max_position_difference_m', ..."; ...
    "    'max_applied_input_difference', 'tilt_tolerance_deg', ..."; ...
    "    'position_tolerance_m', 'input_tolerance', ..."; ...
    "    'max_fuzzy_gain_relative_error', 'fuzzy_gain_accepted', 'accepted'});"; ...
    "end"];
fprintf(file_identifier, "%s\n", lines);
clear cleanup
end

function remove_mock_validation_batch(mock_root, output_root)
clear run_simulink_validation_batch
if contains(path, char(mock_root))
    rmpath(mock_root);
end
if isfolder(mock_root)
    rmdir(mock_root, "s");
end
if isfolder(output_root)
    rmdir(output_root, "s");
end
end

function widths = worksheet_column_widths(workbook_path, sheet_index)
temporary_root = string(tempname);
mkdir(temporary_root);
cleanup = onCleanup(@() rmdir(temporary_root, "s"));
unzip(workbook_path, temporary_root);
xml = fileread(fullfile(temporary_root, "xl", "worksheets", ...
    sprintf("sheet%d.xml", sheet_index)));
columns = regexp(xml, '<col[^>]*width="([^"]+)"[^>]*/>', 'tokens');
widths = str2double(string(cellfun(@(token) token{1}, columns, ...
    "UniformOutput", false)));
end

function count = workbook_sheet_count(workbook_path)
temporary_root = string(tempname);
mkdir(temporary_root);
cleanup = onCleanup(@() rmdir(temporary_root, "s"));
unzip(workbook_path, temporary_root);
items = dir(fullfile(temporary_root, "xl", "worksheets", "sheet*.xml"));
count = numel(items);
end
