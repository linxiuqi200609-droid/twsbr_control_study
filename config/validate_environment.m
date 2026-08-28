function report = validate_environment(project_root, require_statistics)
%VALIDATE_ENVIRONMENT Report whether this study can run in the project root.

valid_project_root_type = (isstring(project_root) && isscalar(project_root)) || ...
    (ischar(project_root) && isrow(project_root));
if ~valid_project_root_type
    error("twsbr:environment:invalid_project_root", ...
        "project_root must be a scalar text path.");
end
if ~islogical(require_statistics) || ~isscalar(require_statistics)
    error("twsbr:environment:invalid_require_statistics", ...
        "require_statistics must be a logical scalar.");
end

project_root = string(project_root);
if strlength(project_root) == 0 || ~isfolder(project_root)
    error("twsbr:environment:invalid_project_root", ...
        "project_root must identify an existing project repository.");
end

[git_status, repository_root] = system(sprintf( ...
    'git -C "%s" rev-parse --show-toplevel', project_root));
is_repository = git_status == 0 && strcmpi( ...
    normalize_path(repository_root), normalize_path(project_root));
if ~is_repository
    error("twsbr:environment:invalid_project_root", ...
        "project_root must identify this Git repository.");
end

simulink_available = ~isempty(ver("simulink"));
control_available = ~isempty(ver("control"));
statistics_available = ~isempty(ver("stats"));
output_writable = can_write_results(project_root);

[git_status, git_commit] = system(sprintf( ...
    'git -C "%s" rev-parse HEAD', project_root));
if git_status ~= 0
    git_commit = "unavailable";
else
    git_commit = strtrim(string(git_commit));
end

report = struct();
report.matlab_release = string(version("-release"));
report.platform = string(computer);
report.simulink_available = simulink_available;
report.control_available = control_available;
report.statistics_available = statistics_available;
report.output_writable = output_writable;
report.git_commit = git_commit;
report.accepted = simulink_available && control_available && output_writable && ...
    (~require_statistics || statistics_available);

if ~simulink_available
    error("twsbr:environment:simulink_required", ...
        "Simulink is required.");
end
if ~control_available
    error("twsbr:environment:control_required", ...
        "Control System Toolbox is required.");
end
if require_statistics && ~statistics_available
    error("twsbr:environment:statistics_required", ...
        "Statistics and Machine Learning Toolbox is required.");
end
end

function writable = can_write_results(project_root)
results_directory = fullfile(project_root, "results");
writable = false;
if ~isfolder(results_directory)
    return
end
temporary_file = string(tempname(results_directory));
cleanup = onCleanup(@() delete_if_present(temporary_file));
[file_id, message] = fopen(temporary_file, "w");
if file_id < 0
    return
end
fclose(file_id);
writable = isfile(temporary_file);
if ~isempty(message)
    writable = false;
end
clear cleanup
end

function delete_if_present(file_path)
if isfile(file_path)
    delete(file_path);
end
end

function path_value = normalize_path(path_value)
path_value = replace(string(strtrim(path_value)), "\", "/");
path_value = regexprep(path_value, "/+$", "");
end
