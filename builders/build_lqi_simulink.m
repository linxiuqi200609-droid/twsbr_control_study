function model_path = build_lqi_simulink(plant_params, params)
%BUILD_LQI_SIMULINK Rebuild the sampled full-state LQI Simulink model.

if nargin < 1
    plant_params = twsbr_params();
else
    plant_params = twsbr_params(plant_params);
end
config = experiment_config("quick");
if nargin < 2
    params = decode_controller_vector("LQI", ...
        log10([10, 1, 200, 10, 100, 0.1]), plant_params, config);
end
params = validate_lqi_simulink_params(params, config);

if isempty(ver("simulink"))
    error("twsbr:simulink:not_available", ...
        "Simulink is required to build the LQI model.");
end

builder_directory = fileparts(mfilename("fullpath"));
project_root = fileparts(builder_directory);
model_directory = fullfile(project_root, "simulink_models");
if ~isfolder(model_directory)
    mkdir(model_directory);
end
plant_model_path = build_twsbr_simulink(plant_params);

model_name = "twsbr_lqi";
model_path = fullfile(model_directory, model_name + ".slx");
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
if isfile(model_path)
    delete(model_path);
end

model_path = create_lqi_simulink_model( ...
    plant_params, params, plant_model_path, model_path);
end

function params = validate_lqi_simulink_params(params, config)
valid = isstruct(params) && isscalar(params) && ...
    isfield(params, "state_gain") && isfield(params, "integral_gain") && ...
    isfield(params, "sample_time") && isfield(params, "position_integral_limit");
if valid
    valid = isnumeric(params.state_gain) && isreal(params.state_gain) && ...
        isequal(size(params.state_gain), [1, 4]) && ...
        all(isfinite(params.state_gain(:))) && ...
        is_finite_scalar(params.integral_gain) && ...
        is_positive_finite_scalar(params.sample_time) && ...
        is_positive_finite_scalar(params.position_integral_limit);
end
if ~valid
    error("twsbr:lqi:invalid_input", ...
        "LQI gains, sample time, and integral limit must be finite.");
end
if abs(params.sample_time - config.sample_time) > 1e-15 || ...
        isfield(params, "plant_step") && ( ...
        ~is_positive_finite_scalar(params.plant_step) || ...
        abs(params.plant_step - config.plant_step) > 1e-15)
    error("twsbr:lqi_simulation:invalid_timing", ...
        "LQI simulation requires plant_step=0.001 and sample_time=0.01.");
end
params.plant_step = config.plant_step;
end

function valid = is_finite_scalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end

function valid = is_positive_finite_scalar(value)
valid = is_finite_scalar(value) && value > 0;
end
