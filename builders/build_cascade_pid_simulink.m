function model_path = build_cascade_pid_simulink(plant_params, params)
%BUILD_CASCADE_PID_SIMULINK Rebuild the cascade PID Simulink model.

if nargin < 1
    plant_params = twsbr_params();
else
    plant_params = twsbr_params(plant_params);
end
if nargin < 2
    params = cascade_pid_params(struct(), plant_params);
else
    params = cascade_pid_params(params, plant_params);
end

if isempty(ver("simulink"))
    error("twsbr:simulink:not_available", ...
        "Simulink is required to build the cascade PID model.");
end

builder_directory = fileparts(mfilename("fullpath"));
project_root = fileparts(builder_directory);
model_directory = fullfile(project_root, "simulink_models");
if ~isfolder(model_directory)
    mkdir(model_directory);
end
plant_model_path = fullfile(model_directory, "twsbr_plant.slx");
if ~isfile(plant_model_path)
    plant_model_path = build_twsbr_simulink(plant_params);
end

model_name = "twsbr_cascade_pid";
model_path = fullfile(model_directory, model_name + ".slx");
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
if isfile(model_path)
    delete(model_path);
end

model_path = create_cascade_pid_simulink_model( ...
    plant_params, params, plant_model_path, model_path);
end
