function model_path = build_twsbr_simulink(params)
%BUILD_TWSBR_SIMULINK Rebuild the nonlinear Simulink plant model.

if nargin < 1
    params = twsbr_params();
else
    params = twsbr_params(params);
end

if isempty(ver("simulink"))
    error("twsbr:simulink:not_available", ...
        "Simulink is required to build the plant model.");
end

builder_directory = fileparts(mfilename("fullpath"));
project_root = fileparts(builder_directory);
model_directory = fullfile(project_root, "simulink_models");
if ~isfolder(model_directory)
    mkdir(model_directory);
end
model_name = "twsbr_plant";
model_path = fullfile(model_directory, model_name + ".slx");

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
if isfile(model_path)
    delete(model_path);
end

model_path = create_twsbr_simulink_model(params, model_path);
end
