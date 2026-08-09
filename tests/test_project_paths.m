function tests = test_project_paths
%TEST_PROJECT_PATHS Verify explicit project path configuration.
tests = functiontests(localfunctions);
end

function test_setup_project_adds_only_allowed_source_directories(test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
source_names = ["models"; "controllers"; "simulation"; "scenarios"; ...
    "builders"; "visualization"; "workflows"];
source_directories = fullfile(project_root, source_names);
excluded_directories = fullfile(project_root, ["tests"; "results"; "docs"; ...
    "simulink_models"; "builders/private"; "slprj"]);

created_directories = create_missing_directories( ...
    [source_directories; excluded_directories]);
slxc_file = fullfile(project_root, "project_path_test.slxc");
created_slxc_file = create_empty_file_if_missing(slxc_file);

original_path = path;
original_directory = string(pwd);
cleanup = onCleanup(@() restore_project_state( ...
    original_path, original_directory, created_directories, slxc_file, ...
    created_slxc_file)); %#ok<NASGU>

addpath(project_root, "-begin");
remove_from_path([source_directories; excluded_directories; slxc_file]);
paths = setup_project();

for index = 1:numel(source_directories)
    verifyTrue(test_case, is_on_matlab_path(source_directories(index)));
end
for index = 1:numel(excluded_directories)
    verifyFalse(test_case, is_on_matlab_path(excluded_directories(index)));
end
verifyFalse(test_case, is_on_matlab_path(slxc_file));
verifyEqual(test_case, paths.code_directories, source_directories);
end

function test_setup_project_returns_absolute_project_paths(test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
paths = setup_project();

verifyEqual(test_case, paths.project_root, project_root);
verifyEqual(test_case, paths.result_directory, fullfile(project_root, "results"));
verifyEqual(test_case, paths.model_directory, fullfile(project_root, "simulink_models"));
verifyEqual(test_case, paths.test_directory, fullfile(project_root, "tests"));
verifyTrue(test_case, all(is_absolute_path(paths.missing_code_directories)));
end

function test_setup_project_is_deterministic_and_preserves_current_directory(test_case)
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
original_directory = string(pwd);
first_paths = setup_project();
second_paths = setup_project();

verifyEqual(test_case, second_paths, first_paths);
verifyEqual(test_case, string(pwd), original_directory);
end

function created_directories = create_missing_directories(directories)
created_directories = strings(0, 1);
for index = 1:numel(directories)
    if ~isfolder(directories(index))
        mkdir(directories(index));
        created_directories(end + 1, 1) = directories(index); %#ok<AGROW>
    end
end
end

function created_file = create_empty_file_if_missing(file_path)
created_file = false;
if ~isfile(file_path)
    file_identifier = fopen(file_path, "w");
    fclose(file_identifier);
    created_file = true;
end
end

function remove_from_path(entries)
for index = 1:numel(entries)
    if is_on_matlab_path(entries(index))
        rmpath(entries(index));
    end
end
end

function result = is_on_matlab_path(entry)
path_entries = string(strsplit(path, pathsep));
result = any(path_entries == string(entry));
end

function result = is_absolute_path(paths)
result = ~cellfun(@isempty, regexp(cellstr(paths), "^[A-Za-z]:[\\\\/]", "once"));
end

function restore_project_state(original_path, original_directory, created_directories, ...
    slxc_file, created_slxc_file)
path(original_path);
cd(original_directory);
if created_slxc_file && isfile(slxc_file)
    delete(slxc_file);
end
for index = numel(created_directories):-1:1
    directory = created_directories(index);
    if isfolder(directory)
        rmdir(directory);
    end
end
end
