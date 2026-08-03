function model_path = build_attitude_pid_simulink(plant_params, pid_params)
%BUILD_ATTITUDE_PID_SIMULINK Rebuild the basic attitude PID model.

if nargin < 1
    plant_params = twsbr_params();
else
    plant_params = twsbr_params(plant_params);
end
if nargin < 2
    pid_params = attitude_pid_params(struct(), plant_params);
else
    pid_params = attitude_pid_params(pid_params, plant_params);
end

if isempty(ver("simulink"))
    error("twsbr:simulink:not_available", ...
        "Simulink is required to build the attitude PID model.");
end

project_root = fileparts(mfilename("fullpath"));
plant_model_path = fullfile(project_root, "twsbr_plant.slx");
if ~isfile(plant_model_path)
    plant_model_path = build_twsbr_simulink(plant_params);
end

model_name = "twsbr_attitude_pid";
model_path = fullfile(project_root, model_name + ".slx");
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
if isfile(model_path)
    delete(model_path);
end

model_path = create_attitude_pid_simulink_model( ...
    plant_params, pid_params, plant_model_path, model_path);
end
