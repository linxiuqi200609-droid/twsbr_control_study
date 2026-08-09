function simulation = run_simulink_open_loop(params, initial_state, stop_time)
%RUN_SIMULINK_OPEN_LOOP Run the generated nonlinear Simulink plant.

if nargin < 1
    params = twsbr_params();
else
    params = twsbr_params(params);
end
if nargin < 2
    initial_state = [0.0; 0.0; deg2rad(3.0); 0.0];
end
if nargin < 3
    stop_time = 2.0;
end

if ~isnumeric(initial_state) || numel(initial_state) ~= 4 || ...
        any(~isfinite(initial_state), "all")
    error("twsbr:simulink:invalid_initial_state", ...
        "Initial state must contain four finite numeric values.");
end
if ~isnumeric(stop_time) || ~isscalar(stop_time) || ...
        ~isfinite(stop_time) || stop_time <= 0
    error("twsbr:simulink:invalid_stop_time", ...
        "Stop time must be a positive finite scalar.");
end

model_path = build_twsbr_simulink(params);
model_name = "twsbr_plant";
load_system(model_path);
cleanup = onCleanup(@() close_loaded_model(model_name));

initial_state = initial_state(:);
set_param(model_name + "/nonlinear_plant/state_integrator", ...
    "InitialCondition", mat2str(initial_state, 17));
set_param(model_name, "StopTime", sprintf("%.17g", stop_time));
set_param(model_name, "SimulationCommand", "update");

simulation_output = sim(model_name, "ReturnWorkspaceOutputs", "on");
state_log = simulation_output.get("state_log");
time = state_log.time(:);
state = squeeze(state_log.signals.values);
if size(state, 1) == 4 && size(state, 2) == numel(time)
    state = state.';
elseif size(state, 2) ~= 4 || size(state, 1) ~= numel(time)
    error("twsbr:simulink:invalid_log_shape", ...
        "Expected a four state trajectory aligned with the time vector.");
end

simulation = struct();
simulation.time = time;
simulation.state = state;
simulation.input = zeros(size(time));

clear cleanup;
close_system(model_name, 0);
end

function close_loaded_model(model_name)
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
end
