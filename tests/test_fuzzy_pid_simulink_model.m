function tests = test_fuzzy_pid_simulink_model
%TEST_FUZZY_PID_SIMULINK_MODEL Behavioral tests for the generated fuzzy PID model.
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function teardownOnce(test_case)
close_if_loaded("twsbr_fuzzy_pid");
close_if_loaded("twsbr_plant");
path(test_case.TestData.original_path);
end

% Mutation caught: returning a relative/wrong artifact path or not saving the model.
function test_builder_creates_expected_project_model(test_case)
[plant, params] = fuzzy_fixture();
paths = setup_project();
model_path = build_fuzzy_pid_simulink(plant, params);
verifyEqual(test_case, string(model_path), ...
    string(fullfile(paths.model_directory, "twsbr_fuzzy_pid.slx")));
verifyTrue(test_case, isfile(model_path));
verifyTrue(test_case, is_absolute_path(model_path));
end

% Mutation caught: removing a required sampling, fuzzy-controller, logging, or plant block.
function test_model_has_required_fuzzy_pid_blocks(test_case)
[model_name, cleanup] = load_built_model(); %#ok<ASGLU>
required = ["position_reference"; "reference_sampling"; "controller_reset"; ...
    "reset_sampling"; "state_sampling"; "fuzzy_controller"; ...
    "actuator_saturation"; "nonlinear_plant"; "disturbance_force"; ...
    "disturbance_torque"; "state_demux"; "state"; "kp_theta_log"; ...
    "ki_theta_log"; "kd_theta_log"; "u_raw_log"];
blocks = string(get_param(find_system(model_name, "SearchDepth", 1, ...
    "Type", "Block"), "Name"));
verifyTrue(test_case, all(ismember(required, blocks)));
end

% Mutation caught: changing the controller interface, state persistence, or shared inference call.
function test_controller_chart_compiles_with_expected_interface(test_case)
[model_name, cleanup] = load_built_model(); %#ok<ASGLU>
controller = model_name + "/fuzzy_controller";
ports = get_param(controller, "PortHandles");
verifyEqual(test_case, numel(ports.Inport), 3);
verifyEqual(test_case, numel(ports.Outport), 9);
chart = find_chart(controller);
verifyNotEmpty(test_case, regexp(string(chart.Script), ...
    "persistent\s+position_integral_state\s+theta_integral_state", "once"));
verifyNotEmpty(test_case, regexp(string(chart.Script), ...
    "fuzzy_pid_inference\s*\(", "once"));
verifyWarningFree(test_case, @() set_param(model_name, ...
    "SimulationCommand", "update"));
end

% Mutation caught: weakening the shared sample/solver timing contract.
function test_model_uses_required_timing_and_solver(test_case)
[model_name, cleanup] = load_built_model(); %#ok<ASGLU>
[~, params] = fuzzy_fixture();
for block = ["reference_sampling", "reset_sampling", "state_sampling"]
    verifyEqual(test_case, str2double(get_param(model_name + "/" + block, ...
        "SampleTime")), params.sample_time, "AbsTol", 1e-15);
end
verifyEqual(test_case, string(get_param(model_name, "SolverType")), "Variable-step");
verifyEqual(test_case, string(get_param(model_name, "Solver")), "ode45");
verifyEqual(test_case, str2double(get_param(model_name, "RelTol")), 1e-9, "AbsTol", 1e-20);
verifyEqual(test_case, str2double(get_param(model_name, "AbsTol")), 1e-11, "AbsTol", 1e-22);
verifyEqual(test_case, str2double(get_param(model_name, "MaxStep")), 0.001, "AbsTol", 1e-15);
verifyEqual(test_case, str2double(get_param(model_name, "StopTime")), 8.0, "AbsTol", 1e-15);
verifyEqual(test_case, string(get_param(model_name, "ReturnWorkspaceOutputs")), "on");
end

% Mutation caught: bypassing the one shared saturation or applying raw control to the plant.
function test_model_routes_raw_control_through_shared_saturation(test_case)
[plant, ~] = fuzzy_fixture();
[model_name, cleanup] = load_built_model(); %#ok<ASGLU>
saturation = model_name + "/actuator_saturation";
plant_path = model_name + "/nonlinear_plant";
verifyEqual(test_case, str2double(get_param(saturation, "UpperLimit")), plant.u_max, "AbsTol", 1e-15);
verifyEqual(test_case, str2double(get_param(saturation, "LowerLimit")), -plant.u_max, "AbsTol", 1e-15);
verifyEqual(test_case, source_block(input_port(saturation, 1)), ...
    get_param(model_name + "/fuzzy_controller", "Handle"));
verifyEqual(test_case, source_block(input_port(plant_path, 1)), get_param(saturation, "Handle"));
end

% Mutation caught: missing deterministic workspace data/logs or unsafe names.
function test_model_has_workspace_inputs_logs_and_safe_identifiers(test_case)
[model_name, cleanup] = load_built_model(); %#ok<ASGLU>
model_workspace = get_param(model_name, "ModelWorkspace");
inputs = ["position_reference_data"; "controller_reset_data"; ...
    "disturbance_force_data"; "disturbance_torque_data"];
for name = inputs'
    verifyEqual(test_case, evalin(model_workspace, "exist('" + name + "','var')"), 1, name);
end
verifyEqual(test_case, evalin(model_workspace, "controller_reset_data"), ...
    [0.0, 1.0; 0.001, 0.0; 8.0, 0.0], "AbsTol", 1e-15);
required_logs = ["state_log"; "position_reference_log"; "position_error_log"; ...
    "theta_reference_log"; "theta_error_log"; "position_integral_log"; ...
    "theta_integral_log"; "kp_theta_log"; "ki_theta_log"; "kd_theta_log"; ...
    "u_raw_log"; "u_log"; "disturbance_force_log"; "disturbance_torque_log"];
sinks = find_system(model_name, "SearchDepth", 1, "BlockType", "ToWorkspace");
variables = string(get_param(sinks, "VariableName"));
verifyTrue(test_case, all(ismember(required_logs, variables)));
for index = 1:numel(sinks)
    verifyEqual(test_case, string(get_param(sinks{index}, "SaveFormat")), "Structure With Time");
end
assert_english_safe_names(test_case, model_name);
end

% Mutation caught: using stale integrals, omitting reset, or replacing shared fuzzy inference.
function test_short_simulation_has_bounded_online_gains_and_deterministic_reset(test_case)
[~, params] = fuzzy_fixture();
[model_name, cleanup] = load_built_model(); %#ok<ASGLU>
workspace = get_param(model_name, "ModelWorkspace");
assignin(workspace, "position_reference_data", [0.0, 0.25; 0.06, 0.25]);
assignin(workspace, "controller_reset_data", ...
    [0.0, 1.0; 0.001, 0.0; 0.03, 1.0; 0.031, 0.0; 0.06, 0.0]);
first = sim(model_name, "StopTime", "0.06");
second = sim(model_name, "StopTime", "0.06");
names = ["position_integral_log"; "theta_integral_log"; "kp_theta_log"; ...
    "ki_theta_log"; "kd_theta_log"; "u_raw_log"];
for name = names'
    [first_time, first_values] = workspace_log_data(first, name);
    [second_time, second_values] = workspace_log_data(second, name);
    verifyTrue(test_case, all(isfinite(first_values(:))), name);
    verifyEqual(test_case, first_time, second_time, "AbsTol", 1e-15);
    verifyEqual(test_case, first_values, second_values, "AbsTol", 1e-13);
end
verifyGreaterThanOrEqual(test_case, workspace_log_data_values(first, "kp_theta_log"), 0.0);
verifyLessThanOrEqual(test_case, workspace_log_data_values(first, "kp_theta_log"), params.kp_theta_max);
verifyGreaterThanOrEqual(test_case, workspace_log_data_values(first, "ki_theta_log"), 0.0);
verifyLessThanOrEqual(test_case, workspace_log_data_values(first, "ki_theta_log"), params.ki_theta_max);
verifyGreaterThanOrEqual(test_case, workspace_log_data_values(first, "kd_theta_log"), 0.0);
verifyLessThanOrEqual(test_case, workspace_log_data_values(first, "kd_theta_log"), params.kd_theta_max);
[time, position_integral] = workspace_log_data(first, "position_integral_log");
[theta_time, theta_integral] = workspace_log_data(first, "theta_integral_log");
reset_index = find(abs(time - 0.03) < 1e-12, 1);
verifyNotEmpty(test_case, reset_index);
verifyEqual(test_case, theta_time, time, "AbsTol", 1e-15);
verifyEqual(test_case, position_integral(reset_index), 0.0, "AbsTol", 1e-15);
verifyEqual(test_case, theta_integral(reset_index), 0.0, "AbsTol", 1e-15);
[state_time, state] = workspace_log_data(first, "state_log");
sample_index = find(abs(state_time) < 1e-12, 1);
control = fuzzy_pid_step(struct("position_integral", 0.0, "theta_integral", 0.0), ...
    reshape(state(:, :, sample_index), [], 1), 0.25, params);
verifyEqual(test_case, workspace_log_data_value_at(first, "kp_theta_log", 0.0), ...
    control.kp_theta, "AbsTol", 1e-12);
verifyEqual(test_case, workspace_log_data_value_at(first, "ki_theta_log", 0.0), ...
    control.ki_theta, "AbsTol", 1e-12);
verifyEqual(test_case, workspace_log_data_value_at(first, "kd_theta_log", 0.0), ...
    control.kd_theta, "AbsTol", 1e-12);
verifyEqual(test_case, workspace_log_data_value_at(first, "u_raw_log", 0.0), ...
    control.u_raw, "AbsTol", 1e-12);
reset_state_index = find(abs(state_time - 0.03) < 1e-12, 1);
verifyNotEmpty(test_case, reset_state_index);
reset_control = fuzzy_pid_step( ...
    struct("position_integral", 0.0, "theta_integral", 0.0), ...
    reshape(state(:, :, reset_state_index), [], 1), 0.25, params);
verifyEqual(test_case, workspace_log_data_value_at(first, "kp_theta_log", 0.03), ...
    reset_control.kp_theta, "AbsTol", 1e-12);
verifyEqual(test_case, workspace_log_data_value_at(first, "ki_theta_log", 0.03), ...
    reset_control.ki_theta, "AbsTol", 1e-12);
verifyEqual(test_case, workspace_log_data_value_at(first, "kd_theta_log", 0.03), ...
    reset_control.kd_theta, "AbsTol", 1e-12);
verifyEqual(test_case, workspace_log_data_value_at(first, "u_raw_log", 0.03), ...
    reset_control.u_raw, "AbsTol", 1e-12);
end

% Mutation caught: accepting a controller rate outside the required 0.01 s contract.
function test_builder_rejects_nonstandard_timing(test_case)
[plant, params] = fuzzy_fixture();
params.sample_time = 0.02;
verifyError(test_case, @() build_fuzzy_pid_simulink(plant, params), ...
    "twsbr:fuzzy_pid_simulation:invalid_timing");
end

% Mutation caught: giving the controller a different actuator limit than the shared plant.
function test_builder_rejects_mismatched_actuator_limit(test_case)
[plant, params] = fuzzy_fixture();
params.u_max = plant.u_max * 0.5;
verifyError(test_case, @() build_fuzzy_pid_simulink(plant, params), ...
    "twsbr:fuzzy_pid_simulation:invalid_actuator_limit");
end

% Mutation caught: retaining nominal nonlinear dynamics after a supplied-plant rebuild.
function test_builder_embeds_custom_plant_and_restores_nominal_model(test_case)
plant = twsbr_params(struct("motor_force_gain", 13.0));
[~, params] = fuzzy_fixture(plant);
try
    model_path = build_fuzzy_pid_simulink(plant, params);
    load_system(model_path);
    chart = find_chart("twsbr_fuzzy_pid/nonlinear_plant/plant_dynamics");
    verifyNotEmpty(test_case, regexp(string(chart.Script), "motor_gain\s*=\s*13;", "once"));
catch cause
    rebuild_nominal_fuzzy_pid();
    rethrow(cause);
end
rebuild_nominal_fuzzy_pid();
end

function [plant, params] = fuzzy_fixture(plant)
if nargin < 1
    plant = twsbr_params();
end
config = experiment_config("quick");
vector = [log10([0.241, 0.000396, 0.193, 9.255, 0.05, 1.011]), 0.2, 0.2, 0.2];
params = decode_controller_vector("FUZZY_PID", vector, plant, config);
end

function [model_name, cleanup] = load_built_model()
[plant, params] = fuzzy_fixture();
model_path = build_fuzzy_pid_simulink(plant, params);
model_name = "twsbr_fuzzy_pid";
load_system(model_path);
cleanup = onCleanup(@() close_if_loaded(model_name));
end

function value = workspace_log_data_values(simulation_output, variable_name)
[~, value] = workspace_log_data(simulation_output, variable_name);
end

function value = workspace_log_data_value_at(simulation_output, variable_name, time)
[times, values] = workspace_log_data(simulation_output, variable_name);
index = find(abs(times - time) < 1e-12, 1);
value = values(index);
end

function [time, values] = workspace_log_data(simulation_output, variable_name)
log = simulation_output.get(char(variable_name));
time = log.time;
values = log.signals.values;
end

function port_handle = input_port(block_path, index)
ports = get_param(block_path, "PortHandles");
port_handle = ports.Inport(index);
end

function block_handle = source_block(port_handle)
line_handle = get_param(port_handle, "Line");
block_handle = get_param(line_handle, "SrcBlockHandle");
end

function chart = find_chart(block_path)
root = sfroot;
chart = root.find("-isa", "Stateflow.EMChart", "Path", char(block_path));
if isempty(chart)
    error("twsbr:test:chart_not_found", "MATLAB Function chart was not found: %s", block_path);
end
end

function assert_english_safe_names(test_case, model_name)
blocks = find_system(model_name, "Type", "Block");
for index = 1:numel(blocks)
    assert_safe_name(test_case, string(get_param(blocks{index}, "Name")));
end
ports = [find_system(model_name, "BlockType", "Inport"); find_system(model_name, "BlockType", "Outport")];
for index = 1:numel(ports)
    assert_safe_name(test_case, string(get_param(ports{index}, "Name")));
end
lines = find_system(model_name, "FindAll", "on", "Type", "line");
for index = 1:numel(lines)
    assert_safe_name(test_case, string(get_param(lines(index), "Name")));
end
end

function assert_safe_name(test_case, name)
verifyNotEmpty(test_case, regexp(name, "^[A-Za-z][A-Za-z0-9_]*$", "once"), name);
end

function rebuild_nominal_fuzzy_pid()
close_if_loaded("twsbr_fuzzy_pid");
close_if_loaded("twsbr_plant");
build_fuzzy_pid_simulink();
end

function absolute = is_absolute_path(file_path)
absolute = ~isempty(regexp(char(file_path), "^[A-Za-z]:[\\/]", "once")) || startsWith(file_path, "\\\\");
end

function close_if_loaded(model_name)
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
end
