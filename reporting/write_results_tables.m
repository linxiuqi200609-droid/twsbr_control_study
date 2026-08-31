function paths = write_results_tables( ...
        output_root, tuning_runs, deterministic, monte_carlo, descriptive, ...
        complexity, validation, varargin)
%WRITE_RESULTS_TABLES Persist flat raw and summary study tables.

output_root = prepare_output_root(output_root);
validate_table_inputs(tuning_runs, deterministic, monte_carlo, ...
    descriptive, complexity, validation);
payload = optional_payload(varargin{:});
directories = make_directories(output_root);

paths = make_paths(output_root, directories);
save_tuning_runs(paths.tuning_mat, tuning_runs);
write_table(paths.tuning_csv, serialize_tuning_runs(tuning_runs));
write_table(paths.deterministic_csv, deterministic);
write_table(paths.parameters_csv, parameter_table(payload.frozen_vectors));
write_table(paths.raw_index_csv, payload.raw_index);
write_table(paths.complexity_csv, complexity);
write_table(paths.monte_carlo_csv, monte_carlo);
write_table(paths.equivalence_csv, validation);
write_table(paths.statistics_descriptive_csv, descriptive);
write_table(paths.statistics_omnibus_csv, payload.omnibus);
write_table(paths.statistics_pairwise_csv, payload.pairwise);
write_json_file(paths.frozen_vectors_json, payload.frozen_vectors);
write_json_file(paths.training_scenarios_json, ...
    scenario_collection_to_records(payload.training_scenarios));
write_json_file(paths.heldout_scenarios_json, ...
    scenario_collection_to_records(payload.heldout_scenarios));

sheet_data = { ...
    "deterministic_raw", deterministic; ...
    "deterministic_summary", payload.deterministic_descriptive; ...
    "monte_carlo_raw", monte_carlo; ...
    "descriptive_CI", descriptive; ...
    "omnibus_tests", payload.omnibus; ...
    "pairwise_tests", payload.pairwise; ...
    "complexity", complexity; ...
    "tuning", serialize_tuning_runs(tuning_runs); ...
    "parameters", parameter_table(payload.frozen_vectors); ...
    "raw_index", payload.raw_index; ...
    "validation", validation};
write_workbook(paths.statistics_xlsx, sheet_data);
paths = canonicalize_paths(paths);
end

function output_root = prepare_output_root(output_root)
valid = (isstring(output_root) && isscalar(output_root) && ...
    ~ismissing(output_root)) || (ischar(output_root) && isrow(output_root));
if ~valid || strlength(string(output_root)) == 0
    error("twsbr:reporting:InvalidPath", ...
        "Output root must be a nonempty scalar path.");
end
output_root = string(output_root);
if ~isfolder(output_root)
    mkdir(output_root);
end
if ~isfolder(output_root)
    error("twsbr:reporting:InvalidPath", ...
        "Output root could not be created.");
end
output_root = canonical_path(output_root);
end

function validate_table_inputs(varargin)
for index = 1:nargin
    if ~istable(varargin{index})
        error("twsbr:reporting:InvalidTable", ...
            "Every fixed reporting input must be a table.");
    end
end
end

function payload = optional_payload(varargin)
payload = struct( ...
    "omnibus", empty_omnibus_table(), ...
    "pairwise", empty_pairwise_table(), ...
    "deterministic_descriptive", empty_descriptive_table(), ...
    "frozen_vectors", struct(), ...
    "raw_index", empty_raw_index(), ...
    "training_scenarios", struct(), ...
    "heldout_scenarios", struct());
if nargin == 0
    return
end

if nargin ~= 1 || ~isstruct(varargin{1}) || ~isscalar(varargin{1})
    error("twsbr:reporting:InvalidPayload", ...
        "Optional reporting payload must be one scalar structure.");
end
provided = varargin{1};
allowed = string(fieldnames(payload));
provided_names = string(fieldnames(provided));
if any(~ismember(provided_names, allowed))
    error("twsbr:reporting:InvalidPayload", ...
        "Optional reporting payload contains an unknown field.");
end
for index = 1:numel(provided_names)
    payload.(provided_names(index)) = provided.(provided_names(index));
end
table_fields = ["omnibus", "pairwise", "deterministic_descriptive", "raw_index"];
for field = table_fields
    if ~istable(payload.(field))
        error("twsbr:reporting:InvalidPayload", ...
            "Optional table payload fields must be tables.");
    end
end
if ~isstruct(payload.frozen_vectors) || ~isscalar(payload.frozen_vectors) || ...
        ~isstruct(payload.training_scenarios) || ...
        ~isstruct(payload.heldout_scenarios)
    error("twsbr:reporting:InvalidPayload", ...
        "Optional vector and scenario payloads must be structures.");
end
end

function result = empty_descriptive_table()
result = table(strings(0,1),strings(0,1),zeros(0,1),nan(0,1),nan(0,1),nan(0,1),nan(0,1),nan(0,1),nan(0,1),nan(0,1),zeros(0,1),zeros(0,1),nan(0,1),nan(0,1),nan(0,1),'VariableNames',{'controller','metric','n','mean','median','std','q1','q3','bootstrap_ci_low','bootstrap_ci_high','total_n','success_count','success_rate','success_ci_low','success_ci_high'});
end

function result = empty_omnibus_table()
result = table(strings(0,1),zeros(0,1),zeros(0,1),nan(0,1),'VariableNames',{'metric','group_count','successful_n','p_value'});
end

function result = empty_pairwise_table()
result = table(strings(0,1),strings(0,1),strings(0,1),zeros(0,1),zeros(0,1),nan(0,1),nan(0,1),nan(0,1),'VariableNames',{'metric','first_controller','second_controller','first_n','second_n','p_value','p_value_holm','cliffs_delta'});
end

function directories = make_directories(output_root)
names = ["tuning", "deterministic", "raw", "monte_carlo", ...
    "simulink_validation", "figures"];
directories = struct();
for index = 1:numel(names)
    path_value = fullfile(output_root, names(index));
    if ~isfolder(path_value)
        mkdir(path_value);
    end
    if ~isfolder(path_value)
        error("twsbr:reporting:DirectoryCreateFailed", ...
            "Could not create output directory: %s", path_value);
    end
    directories.(names(index)) = string(path_value);
end
end

function paths = make_paths(output_root, directories)
paths = struct();
paths.output_root = output_root;
paths.tuning_directory = directories.tuning;
paths.deterministic_directory = directories.deterministic;
paths.raw_directory = directories.raw;
paths.monte_carlo_directory = directories.monte_carlo;
paths.simulink_validation_directory = directories.simulink_validation;
paths.figure_directory = directories.figures;
paths.tuning_mat = fullfile(directories.tuning, "tuning_runs.mat");
paths.tuning_csv = fullfile(directories.tuning, "tuning_runs.csv");
paths.frozen_vectors_json = fullfile(directories.tuning, ...
    "frozen_controller_parameters.json");
paths.training_scenarios_json = fullfile(output_root, "training_scenarios.json");
paths.heldout_scenarios_json = fullfile(output_root, "heldout_scenarios.json");
paths.deterministic_csv = fullfile(directories.deterministic, "summary_metrics.csv");
paths.parameters_csv = fullfile(directories.deterministic, "controller_parameters.csv");
paths.raw_index_csv = fullfile(directories.raw, "raw_index.csv");
paths.complexity_csv = fullfile(directories.deterministic, "controller_complexity.csv");
paths.monte_carlo_csv = fullfile(directories.monte_carlo, "monte_carlo_metrics.csv");
paths.equivalence_csv = fullfile(directories.simulink_validation, ...
    "equivalence_summary.csv");
paths.statistics_xlsx = fullfile(output_root, "statistics.xlsx");
paths.statistics_descriptive_csv = fullfile(output_root, "statistics_descriptive.csv");
paths.statistics_omnibus_csv = fullfile(output_root, "statistics_omnibus.csv");
paths.statistics_pairwise_csv = fullfile(output_root, "statistics_pairwise.csv");
end

function save_tuning_runs(path_value, tuning_runs)
save(path_value, "tuning_runs");
if ~isfile(path_value) || dir(path_value).bytes == 0
    error("twsbr:reporting:WriteFailed", ...
        "Tuning MAT output was not created.");
end
end

function write_table(path_value, data)
writetable(data, path_value, "WriteMode", "overwrite");
if ~isfile(path_value) || dir(path_value).bytes == 0
    error("twsbr:reporting:WriteFailed", ...
        "Table output was not created: %s", path_value);
end
end

function write_workbook(path_value, sheet_data)
if isfile(path_value)
    delete(path_value);
end
for index = 1:size(sheet_data, 1)
    writetable(sheet_data{index, 2}, path_value, "Sheet", sheet_data{index, 1}, ...
        "WriteMode", "overwritesheet", "AutoFitWidth", true);
end
widen_workbook_text_columns(path_value, sheet_data);
if ~isfile(path_value) || dir(path_value).bytes == 0
    error("twsbr:reporting:WriteFailed", ...
        "Statistics workbook was not created.");
end
end

function widen_workbook_text_columns(path_value, sheet_data)
temporary_root = string(tempname);
mkdir(temporary_root);
cleanup = onCleanup(@() remove_directory_if_present(temporary_root));
unzip(path_value, temporary_root);
for index = 1:size(sheet_data, 1)
    widths = text_column_widths(sheet_data{index, 2});
    if isempty(widths)
        continue
    end
    sheet_path = fullfile(temporary_root, "xl", "worksheets", ...
        sprintf("sheet%d.xml", index));
    if ~isfile(sheet_path)
        error("twsbr:reporting:WriteFailed", ...
            "Statistics workbook is missing a required worksheet XML file.");
    end
    write_text_file(sheet_path, widen_sheet_xml(fileread(sheet_path), widths));
end
archive_path = string(tempname) + ".zip";
archive_cleanup = onCleanup(@() delete_file_if_present(archive_path));
files = recursive_files(temporary_root);
zip(archive_path, cellstr(files), temporary_root);
[moved, message] = movefile(archive_path, path_value, "f");
if ~moved
    error("twsbr:reporting:WriteFailed", ...
        "Could not replace the widened statistics workbook: %s", message);
end
clear archive_cleanup cleanup
end

function widths = text_column_widths(data)
widths = zeros(1, width(data));
names = string(data.Properties.VariableNames);
for index = 1:width(data)
    values = data.(names(index));
    if isstring(values) || iscellstr(values) || iscategorical(values)
        lengths = strlength(string(values));
        widths(index) = min(60, max(12, double(max([strlength(names(index)); ...
            lengths(:); 0])) + 4));
    end
end
end

function xml = widen_sheet_xml(xml, widths)
tags = regexp(xml, '<col\s+[^>]*>', 'match');
for column = find(widths > 0)
    for tag_index = 1:numel(tags)
        minimum = regexp(tags{tag_index}, 'min="(\d+)"', 'tokens', 'once');
        maximum = regexp(tags{tag_index}, 'max="(\d+)"', 'tokens', 'once');
        if isempty(minimum) || isempty(maximum) || ...
                column < str2double(minimum{1}) || column > str2double(maximum{1})
            continue
        end
        existing = regexp(tags{tag_index}, 'width="([^"]+)"', ...
            'tokens', 'once');
        target_width = widths(column);
        if ~isempty(existing)
            target_width = max(target_width, str2double(existing{1}));
        end
        replacement = regexprep(tags{tag_index}, 'width="[^"]+"', ...
            sprintf('width="%.6f"', target_width));
        xml = strrep(xml, tags{tag_index}, replacement);
        tags{tag_index} = replacement;
        break
    end
end
end

function write_text_file(path_value, contents)
[file_identifier, message] = fopen(path_value, "w", "n", "UTF-8");
if file_identifier < 0
    error("twsbr:reporting:WriteFailed", ...
        "Could not update workbook XML: %s", message);
end
cleanup = onCleanup(@() fclose(file_identifier));
fwrite(file_identifier, unicode2native(contents, "UTF-8"), "uint8");
clear cleanup
end

function files = recursive_files(root)
items = dir(fullfile(root, "**", "*"));
items = items(~[items.isdir]);
absolute_paths = fullfile(string({items.folder}), string({items.name}));
files = erase(absolute_paths, string(root) + filesep);
end

function remove_directory_if_present(path_value)
if isfolder(path_value)
    rmdir(path_value, "s");
end
end

function delete_file_if_present(path_value)
if isfile(path_value)
    delete(path_value);
end
end

function output = serialize_tuning_runs(tuning_runs)
output = tuning_runs;
if ismember("vector", string(output.Properties.VariableNames)) && ...
        iscell(output.vector)
    values = strings(height(output), 1);
    for index = 1:height(output)
        values(index) = string(jsonencode(double(output.vector{index}(:).')));
    end
    output.vector = values;
end
end

function result = parameter_table(vectors)
controller = strings(0, 1);
parameter_index = zeros(0, 1);
value = zeros(0, 1);
if isstruct(vectors) && isscalar(vectors)
    names = string(fieldnames(vectors));
    for name_index = 1:numel(names)
        vector = vectors.(names(name_index));
        if ~isnumeric(vector) || ~isreal(vector) || ~isvector(vector) || ...
                any(~isfinite(vector))
            error("twsbr:reporting:InvalidVectors", ...
                "Frozen vectors must be finite real numeric vectors.");
        end
        vector = double(vector(:));
        controller = [controller; repmat(names(name_index), numel(vector), 1)]; %#ok<AGROW>
        parameter_index = [parameter_index; (1:numel(vector)).']; %#ok<AGROW>
        value = [value; vector]; %#ok<AGROW>
    end
end
result = table(controller, parameter_index, value, 'VariableNames', ...
    {'controller', 'parameter_index', 'value'});
end

function result = empty_raw_index()
result = table(strings(0, 1), strings(0, 1), strings(0, 1), ...
    'VariableNames', {'controller', 'scenario', 'relative_path'});
end

function records = scenario_collection_to_records(scenarios)
if isempty(fieldnames(scenarios))
    records = struct();
    return
end
if ~isscalar(scenarios)
    error("twsbr:reporting:InvalidScenario", ...
        "Scenario collection must be a scalar structure.");
end
records = struct();
names = fieldnames(scenarios);
for index = 1:numel(names)
    records.(names{index}) = scenario_to_record(scenarios.(names{index}));
end
end

function paths = canonicalize_paths(paths)
names = fieldnames(paths);
for index = 1:numel(names)
    paths.(names{index}) = canonical_path(paths.(names{index}));
end
end

function path_value = canonical_path(path_value)
path_value = string(java.io.File(char(string(path_value))).getCanonicalPath());
end
