function model_path = build_fuzzy_pid_simulink(plant_params, params)
%BUILD_FUZZY_PID_SIMULINK Rebuild the sampled fuzzy PID Simulink model.

if nargin < 1
    plant_params = twsbr_params();
else
    plant_params = twsbr_params(plant_params);
end
config = experiment_config("quick");
if nargin < 2
    vector = [log10([0.241, 0.000396, 0.193, 9.255, 0.05, 1.011]), ...
        0.2, 0.2, 0.2];
    params = decode_controller_vector("FUZZY_PID", vector, plant_params, config);
end
params = validate_fuzzy_pid_simulink_params(params, plant_params, config);

if isempty(ver("simulink"))
    error("twsbr:simulink:not_available", ...
        "Simulink is required to build the fuzzy PID model.");
end

builder_directory = fileparts(mfilename("fullpath"));
project_root = fileparts(builder_directory);
model_directory = fullfile(project_root, "simulink_models");
if ~isfolder(model_directory)
    mkdir(model_directory);
end
plant_model_path = build_twsbr_simulink(plant_params);

model_name = "twsbr_fuzzy_pid";
model_path = fullfile(model_directory, model_name + ".slx");
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
if isfile(model_path)
    delete(model_path);
end

model_path = create_fuzzy_pid_simulink_model( ...
    plant_params, params, plant_model_path, model_path);
end

function params = validate_fuzzy_pid_simulink_params(params, plant_params, config)
required_fields = { ...
    'kp_x', 'ki_x', 'kd_x', ...
    'kp_theta_base', 'ki_theta_base', 'kd_theta_base', ...
    'alpha_p', 'alpha_i', 'alpha_d', ...
    'theta_error_normalizer', 'theta_rate_normalizer', ...
    'kp_theta_max', 'ki_theta_max', 'kd_theta_max', ...
    'sample_time', 'theta_reference_limit', ...
    'position_integral_limit', 'theta_integral_limit', 'u_max'};
valid = isstruct(params) && isscalar(params) && all(isfield(params, required_fields));
if valid
    nonnegative = {'kp_x', 'ki_x', 'kd_x', ...
        'kp_theta_base', 'ki_theta_base', 'kd_theta_base', ...
        'kp_theta_max', 'ki_theta_max', 'kd_theta_max'};
    positive = {'theta_error_normalizer', 'theta_rate_normalizer', ...
        'theta_reference_limit', 'position_integral_limit', ...
        'theta_integral_limit', 'u_max'};
    valid = all(cellfun(@(name) is_nonnegative_finite_scalar(params.(name)), nonnegative)) && ...
        all(cellfun(@(name) is_positive_finite_scalar(params.(name)), positive)) && ...
        all(cellfun(@(name) is_finite_scalar(params.(name)) && ...
        params.(name) >= 0 && params.(name) <= 1.5, ...
        {'alpha_p', 'alpha_i', 'alpha_d'})) && ...
        is_positive_finite_scalar(params.sample_time);
end
if ~valid
    error("twsbr:fuzzy_pid:invalid_input", ...
        "Fuzzy PID gains, normalizers, limits, and timing must be finite.");
end
if abs(params.sample_time - config.sample_time) > 1e-15 || ...
        isfield(params, "plant_step") && ( ...
        ~is_positive_finite_scalar(params.plant_step) || ...
        abs(params.plant_step - config.plant_step) > 1e-15)
    error("twsbr:fuzzy_pid_simulation:invalid_timing", ...
        "Fuzzy PID simulation requires plant_step=0.001 and sample_time=0.01.");
end
if abs(params.u_max - plant_params.u_max) > 1e-15
    error("twsbr:fuzzy_pid_simulation:invalid_actuator_limit", ...
        "Fuzzy PID controller u_max must match the plant actuator limit.");
end
params.plant_step = config.plant_step;
end

function valid = is_finite_scalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value);
end

function valid = is_nonnegative_finite_scalar(value)
valid = is_finite_scalar(value) && value >= 0;
end

function valid = is_positive_finite_scalar(value)
valid = is_finite_scalar(value) && value > 0;
end
