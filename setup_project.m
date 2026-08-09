function paths = setup_project()
project_root = string(fileparts(mfilename("fullpath")));
directory_names = ["models"; "controllers"; "simulation"; "scenarios"; ...
    "builders"; "visualization"; "workflows"];
code_directories = fullfile(project_root, directory_names);
existing = isfolder(code_directories);
for index = 1:numel(code_directories)
    if existing(index)
        addpath(code_directories(index), "-end");
    end
end
paths.project_root = project_root;
paths.code_directories = code_directories(existing);
paths.missing_code_directories = code_directories(~existing);
paths.result_directory = fullfile(project_root, "results");
paths.model_directory = fullfile(project_root, "simulink_models");
paths.test_directory = fullfile(project_root, "tests");
end
