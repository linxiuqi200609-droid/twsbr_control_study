function model_path = build_lqr_simulink(plant_params, params)
%BUILD_LQR_SIMULINK Rebuild the sampled full-state LQR Simulink model.

if nargin < 1
    plant_params = twsbr_params();
else
    plant_params = twsbr_params(plant_params);
end
config = experiment_config("quick");
if nargin < 2
    params = decode_controller_vector("LQR", ...
        log10([10, 1, 200, 10, 0.1]), plant_params, config);
end
params = validate_lqr_simulink_params(params, config);

if isempty(ver("simulink"))
    error("twsbr:simulink:not_available", ...
        "Simulink is required to build the LQR model.");
end

builder_directory = fileparts(mfilename("fullpath"));
project_root = fileparts(builder_directory);
model_directory = fullfile(project_root, "simulink_models");
if ~isfolder(model_directory)
    mkdir(model_directory);
end
plant_model_path = build_twsbr_simulink(plant_params);

model_name = "twsbr_lqr";
model_path = fullfile(model_directory, model_name + ".slx");
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
if isfile(model_path)
    delete(model_path);
end

model_path = create_lqr_simulink_model( ...
    plant_params, params, plant_model_path, model_path);
end

function params = validate_lqr_simulink_params(params, config)
valid = isstruct(params) && isscalar(params) && ...
    isfield(params, "gain") && isfield(params, "sample_time");
if valid
    valid = isnumeric(params.gain) && isreal(params.gain) && ...
        isequal(size(params.gain), [1, 4]) && ...
        all(isfinite(params.gain(:))) && ...
        is_positive_finite_scalar(params.sample_time);
end
if ~valid
    error("twsbr:lqr:invalid_input", ...
        "LQR gain and sample time must be finite decoded parameters.");
end
if abs(params.sample_time - config.sample_time) > 1e-15 || ...
        isfield(params, "plant_step") && ( ...
        ~is_positive_finite_scalar(params.plant_step) || ...
        abs(params.plant_step - config.plant_step) > 1e-15)
    error("twsbr:lqr_simulation:invalid_timing", ...
        "LQR simulation requires plant_step=0.001 and sample_time=0.01.");
end
params.plant_step = config.plant_step;
end

function valid = is_positive_finite_scalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value) && value > 0;
end
