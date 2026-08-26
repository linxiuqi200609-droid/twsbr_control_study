function simulation = simulate_control_system(controller_name, vector, ...
        plant_params, config, scenario, seed)
%SIMULATE_CONTROL_SYSTEM Run a controller against the fixed-step plant.

runtime_start = tic;
validate_inputs(plant_params, config, scenario, seed);

plant_step = config.plant_step;
controller_step_ratio = round(config.sample_time / plant_step);
step_count = round(scenario.duration / plant_step);
if abs(step_count * plant_step - scenario.duration) > 1e-12
    error("twsbr:simulation:invalid_duration", ...
        "Scenario duration must be an integer multiple of the plant step.");
end

time = (0:step_count).' * plant_step;
sample_count = numel(time);
state = nan(sample_count, 4);
state(1, :) = scenario.initial_state.';
position_reference = nan(sample_count, 1);
theta_reference = nan(sample_count, 1);
u_raw = nan(sample_count, 1);
u = nan(sample_count, 1);
saturated = false(sample_count, 1);
force_disturbance = nan(sample_count, 1);
torque_disturbance = nan(sample_count, 1);

noise_stream = RandStream("mt19937ar", "Seed", seed);
measurement_noise = randn(noise_stream, sample_count, 4) .* ...
    scenario.measurement_noise_std.';
zero_noise = scenario.measurement_noise_std == 0;
measurement_noise(:, zero_noise) = 0;

controller = reset_controller(create_controller( ...
    controller_name, vector, plant_params, config));
held_control = struct();
held_u = 0.0;
held_saturated = false;
diagnostic_samples = cell(sample_count, 1);
success = true;
failure_reason = "";
logged_count = 0;

for index = 1:sample_count
    current_time = time(index);
    reference = scalar_signal(scenario.x_reference, current_time, ...
        "x_reference");
    force = scalar_signal(scenario.force_disturbance, current_time, ...
        "force_disturbance");
    torque = scalar_signal(scenario.torque_disturbance, current_time, ...
        "torque_disturbance");

    if any(~isfinite(state(index, :)))
        success = false;
        failure_reason = "nonfinite_state";
        break
    end

    if mod(index - 1, controller_step_ratio) == 0
        measured_state = state(index, :).' + measurement_noise(index, :).';
        if any(~isfinite(measured_state))
            success = false;
            failure_reason = "nonfinite_state";
            break
        end
        try
            [next_control, controller] = controller_step(controller, ...
                current_time, measured_state, reference);
        catch exception
            if strcmp(exception.identifier, ...
                    "twsbr:cascade_pid:nonfinite_output")
                success = false;
                failure_reason = "nonfinite_control";
                break
            end
            rethrow(exception);
        end
        validate_control_contract(next_control);
        if ~isfinite(next_control.u_raw) || ...
                ~isfinite(next_control.theta_reference)
            success = false;
            failure_reason = "nonfinite_control";
            break
        end
        held_control = next_control;
        held_u = min(max(held_control.u_raw, -plant_params.u_max), ...
            plant_params.u_max);
        held_saturated = abs(held_control.u_raw) > plant_params.u_max;
        controller = controller_after_actuation( ...
            controller, held_control.u_raw, held_u);
    end

    position_reference(index) = reference;
    theta_reference(index) = held_control.theta_reference;
    u_raw(index) = held_control.u_raw;
    u(index) = held_u;
    saturated(index) = held_saturated;
    force_disturbance(index) = force;
    torque_disturbance(index) = torque;
    diagnostic_samples{index} = held_control.diagnostics;
    logged_count = index;

    [failed, reason] = state_failure(state(index, :));
    if failed
        success = false;
        failure_reason = reason;
        break
    end
    if index == sample_count
        break
    end

    try
        next_state = twsbr_rk4_step(state(index, :).', held_u, ...
            plant_step, plant_params, force, torque);
    catch exception
        if strcmp(exception.identifier, "twsbr:dynamics:invalid_state")
            success = false;
            failure_reason = "nonfinite_state";
            break
        elseif strcmp(exception.identifier, ...
                "twsbr:dynamics:singular_mass_matrix")
            success = false;
            failure_reason = "singular_mass_matrix";
            break
        end
        rethrow(exception);
    end
    if ~isnumeric(next_state) || ~isreal(next_state) || ...
            ~isequal(size(next_state), [4, 1])
        error("twsbr:simulation:invalid_plant_output", ...
            "Plant integration must return a real numeric 4-by-1 state.");
    end
    if any(~isfinite(next_state))
        success = false;
        failure_reason = "nonfinite_state";
        break
    end
    state(index + 1, :) = next_state.';
end

time = time(1:logged_count);
state = state(1:logged_count, :);
position_reference = position_reference(1:logged_count);
theta_reference = theta_reference(1:logged_count);
u_raw = u_raw(1:logged_count);
u = u(1:logged_count);
saturated = saturated(1:logged_count);
force_disturbance = force_disturbance(1:logged_count);
torque_disturbance = torque_disturbance(1:logged_count);
if logged_count == 0
    diagnostics = repmat(struct(), 0, 1);
else
    diagnostics = vertcat(diagnostic_samples{1:logged_count});
end

if success
    survived_time = scenario.duration;
elseif logged_count == 0
    survived_time = 0.0;
else
    survived_time = time(end);
end

simulation = struct();
simulation.controller_name = string(controller.name);
simulation.scenario_name = string(scenario.name);
simulation.seed = seed;
simulation.time = time;
simulation.state = state;
simulation.position_reference = position_reference;
simulation.theta_reference = theta_reference;
simulation.u_raw = u_raw;
simulation.u = u;
simulation.saturated = saturated;
simulation.force_disturbance = force_disturbance;
simulation.torque_disturbance = torque_disturbance;
simulation.diagnostics = diagnostics;
simulation.success = success;
simulation.failure_reason = failure_reason;
simulation.survived_time = survived_time;
simulation.runtime_seconds = toc(runtime_start);
end

function validate_inputs(plant_params, config, scenario, seed)
validate_plant_params(plant_params);
validate_config(config);
validate_scenario(scenario);
if ~isnumeric(seed) || ~isreal(seed) || ~isscalar(seed) || ...
        ~isfinite(seed) || seed < 0 || seed > 2^32 - 1 || ...
        seed ~= floor(seed)
    error("twsbr:simulation:invalid_seed", ...
        "Seed must be an integer from 0 through 2^32-1.");
end
end

function validate_plant_params(plant_params)
required_fields = ["body_mass", "wheel_mass_equiv", "com_length", ...
    "body_inertia", "wheel_radius", "viscous_damping", "gravity", ...
    "motor_force_gain", "u_max", "theta_fail_deg", "x_limit"];
if ~isstruct(plant_params) || ~isscalar(plant_params) || ...
        ~all(isfield(plant_params, required_fields))
    error("twsbr:simulation:invalid_plant_params", ...
        "Plant parameters are missing required fields.");
end
for index = 1:numel(required_fields)
    value = plant_params.(required_fields(index));
    if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ...
            ~isfinite(value)
        error("twsbr:simulation:invalid_plant_params", ...
            "Plant parameters must be finite real numeric scalars.");
    end
end
positive_fields = required_fields(required_fields ~= "viscous_damping");
for index = 1:numel(positive_fields)
    if plant_params.(positive_fields(index)) <= 0
        error("twsbr:simulation:invalid_plant_params", ...
            "Plant parameters must satisfy their physical bounds.");
    end
end
if plant_params.viscous_damping < 0
    error("twsbr:simulation:invalid_plant_params", ...
        "Plant parameters must satisfy their physical bounds.");
end
end

function validate_config(config)
required_fields = ["sample_time", "plant_step", "theta_reference_limit"];
if ~isstruct(config) || ~isscalar(config) || ...
        ~all(isfield(config, required_fields))
    error("twsbr:simulation:invalid_config", ...
        "Experiment configuration is missing required fields.");
end
for index = 1:numel(required_fields)
    value = config.(required_fields(index));
    if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ...
            ~isfinite(value) || value <= 0
        error("twsbr:simulation:invalid_config", ...
            "Experiment timing and limits must be positive finite scalars.");
    end
end
step_ratio = config.sample_time / config.plant_step;
if abs(config.sample_time - 0.01) > 1e-15 || ...
        abs(config.plant_step - 0.001) > 1e-15 || ...
        abs(step_ratio - round(step_ratio)) > 1e-12 || ...
        round(step_ratio) ~= 10
    error("twsbr:simulation:invalid_timing", ...
        "Simulation requires sample_time=0.01 and plant_step=0.001.");
end
end

function validate_scenario(scenario)
required_fields = ["name", "initial_state", "duration", ...
    "x_reference", "force_disturbance", "torque_disturbance", ...
    "measurement_noise_std"];
if ~isstruct(scenario) || ~isscalar(scenario) || ...
        ~all(isfield(scenario, required_fields))
    error("twsbr:simulation:invalid_scenario", ...
        "Scenario must be a scalar structure with all required fields.");
end
if ~(ischar(scenario.name) && isrow(scenario.name)) && ...
        ~(isstring(scenario.name) && isscalar(scenario.name) && ...
        ~ismissing(scenario.name))
    error("twsbr:simulation:invalid_scenario", ...
        "Scenario name must be a character row or string scalar.");
end
if ~isnumeric(scenario.initial_state) || ~isreal(scenario.initial_state) || ...
        ~isequal(size(scenario.initial_state), [4, 1])
    error("twsbr:simulation:invalid_scenario", ...
        "Initial state must be a real numeric 4-by-1 column vector.");
end
if ~isnumeric(scenario.duration) || ~isreal(scenario.duration) || ...
        ~isscalar(scenario.duration) || ~isfinite(scenario.duration) || ...
        scenario.duration <= 0
    error("twsbr:simulation:invalid_scenario", ...
        "Scenario duration must be a positive finite real scalar.");
end
signal_fields = ["x_reference", "force_disturbance", ...
    "torque_disturbance"];
for index = 1:numel(signal_fields)
    if ~isa(scenario.(signal_fields(index)), "function_handle")
        error("twsbr:simulation:invalid_scenario", ...
            "Scenario signals must be function handles.");
    end
end
noise_std = scenario.measurement_noise_std;
if ~isnumeric(noise_std) || ~isreal(noise_std) || ...
        ~isequal(size(noise_std), [4, 1]) || ...
        any(~isfinite(noise_std)) || any(noise_std < 0)
    error("twsbr:simulation:invalid_scenario", ...
        "Measurement noise standard deviation must be a finite nonnegative 4-by-1 column vector.");
end
end

function value = scalar_signal(signal_function, time, field_name)
value = signal_function(time);
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ...
        ~isfinite(value)
    error("twsbr:simulation:invalid_signal", ...
        "Scenario signal %s must return a finite real numeric scalar.", ...
        field_name);
end
end

function validate_control_contract(control)
required_fields = ["u_raw", "theta_reference", "diagnostics"];
if ~isstruct(control) || ~isscalar(control) || ...
        ~all(isfield(control, required_fields)) || ...
        ~isnumeric(control.u_raw) || ~isreal(control.u_raw) || ...
        ~isscalar(control.u_raw) || ...
        ~isnumeric(control.theta_reference) || ...
        ~isreal(control.theta_reference) || ...
        ~isscalar(control.theta_reference) || ...
        ~isstruct(control.diagnostics) || ~isscalar(control.diagnostics)
    error("twsbr:simulation:invalid_control_contract", ...
        "Controller output does not satisfy the common scalar contract.");
end
end

function [failed, reason] = state_failure(state)
failed = false;
reason = "";
if any(~isfinite(state))
    failed = true;
    reason = "nonfinite_state";
elseif abs(state(3)) > deg2rad(30)
    failed = true;
    reason = "tilt_limit";
elseif abs(state(1)) > 5
    failed = true;
    reason = "position_limit";
end
end
