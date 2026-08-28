function tests = test_lqi_simulink_model
%TEST_LQI_SIMULINK_MODEL Structural tests for the generated LQI model.
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function teardownOnce(test_case)
close_if_loaded("twsbr_lqi");
close_if_loaded("twsbr_plant");
path(test_case.TestData.original_path);
end

% Mutation caught: returning a relative/wrong artifact path or not saving the model.
function test_builder_creates_expected_project_model(test_case)
[plant, params] = lqi_fixture();
paths = setup_project();

model_path = build_lqi_simulink(plant, params);

verifyEqual(test_case, string(model_path), ...
    string(fullfile(paths.model_directory, "twsbr_lqi.slx")));
verifyTrue(test_case, isfile(model_path));
verifyTrue(test_case, is_absolute_path(model_path));
end

% Mutation caught: removing one required LQI control block or its sampling block.
function test_model_has_required_lqi_blocks(test_case)
[model_name, cleanup] = load_built_model(); %#ok<ASGLU>
required_blocks = ["position_reference"; "reference_sampling"; ...
    "controller_reset"; "reset_sampling"; "state_sampling"; "state_demux"; ...
    "position_error"; "position_error_integrator"; "state_gain"; ...
    "integral_gain"; "controller_sum"; "actuator_saturation"; ...
    "nonlinear_plant"; "disturbance_force"; "disturbance_torque"; "state"];
blocks = string(get_param(find_system(model_name, "SearchDepth", 1, ...
    "Type", "Block"), "Name"));
verifyTrue(test_case, all(ismember(required_blocks, blocks)));
verifyTrue(test_case, is_simulink_block_present(model_name, ...
    "position_error_integrator"));
end

% Mutation caught: reversing error polarity, dropping either feedback, or misrouting it.
function test_model_wires_lqi_feedback_with_required_gains(test_case)
[plant, params] = lqi_fixture();
[model_name, cleanup] = load_built_model(plant, params); %#ok<ASGLU>

verifyEqual(test_case, string(get_param(model_name + "/position_error", ...
    "Inputs")), "+-");
verifyEqual(test_case, string(get_param(model_name + "/state_gain", "Gain")), ...
    string(mat2str(-params.state_gain, 17)));
verifyEqual(test_case, string(get_param(model_name + "/state_gain", ...
    "Multiplication")), "Matrix(K*u)");
verifyEqual(test_case, string(get_param(model_name + "/integral_gain", ...
    "Gain")), string(mat2str(-params.integral_gain, 17)));
verifyEqual(test_case, string(get_param(model_name + "/controller_sum", ...
    "Inputs")), "++");

verifyEqual(test_case, source_block(input_port(model_name + "/controller_sum", 1)), ...
    get_param(model_name + "/state_gain", "Handle"));
verifyEqual(test_case, source_block(input_port(model_name + "/controller_sum", 2)), ...
    get_param(model_name + "/integral_gain", "Handle"));
verifyEqual(test_case, source_block(input_port(model_name + "/integral_gain", 1)), ...
    get_param(model_name + "/position_error_integrator", "Handle"));
verifyEqual(test_case, source_block(input_port(model_name + "/position_error_integrator", 1)), ...
    get_param(model_name + "/position_error", "Handle"));
verifyEqual(test_case, source_block(input_port(model_name + "/position_error", 1)), ...
    get_param(model_name + "/state_demux", "Handle"));
state_demux_ports = get_param(model_name + "/state_demux", "PortHandles");
verifyEqual(test_case, source_port(input_port(model_name + "/position_error", 1)), ...
    state_demux_ports.Outport(1));
verifyEqual(test_case, source_block(input_port(model_name + "/position_error", 2)), ...
    get_param(model_name + "/reference_sampling", "Handle"));
verifyEqual(test_case, source_block(input_port(model_name + "/state_gain", 1)), ...
    get_param(model_name + "/state_sampling", "Handle"));
end

% Mutation caught: gainval=sample_time (Ts squared), wrong sample time, reset,
% method, initial condition, or safety-limit configuration.
function test_integrator_applies_sample_time_exactly_once(test_case)
[plant, params] = lqi_fixture();
[model_name, cleanup] = load_built_model(plant, params); %#ok<ASGLU>
integrator = model_name + "/position_error_integrator";

verifyEqual(test_case, str2double(get_param(integrator, "gainval")), 1.0, ...
    "AbsTol", 1e-15);
verifyEqual(test_case, str2double(get_param(integrator, "SampleTime")), 0.01, ...
    "AbsTol", 1e-15);
dialog_parameters = get_param(integrator, "DialogParameters");
if isfield(dialog_parameters, "IntegratorMethod")
    verifyTrue(test_case, contains(string(get_param(integrator, ...
        "IntegratorMethod")), "Forward Euler"));
end
verifyEqual(test_case, str2double(get_param(integrator, "InitialCondition")), 0.0, ...
    "AbsTol", 1e-15);
verifyEqual(test_case, string(get_param(integrator, "ExternalReset")), "rising");
verifyEqual(test_case, string(get_param(integrator, "LimitOutput")), "on");
verifyEqual(test_case, str2double(get_param(integrator, "UpperSaturationLimit")), ...
    params.position_integral_limit, "AbsTol", 1e-15);
verifyEqual(test_case, str2double(get_param(integrator, "LowerSaturationLimit")), ...
    -params.position_integral_limit, "AbsTol", 1e-15);
verifyEqual(test_case, source_block(input_port(integrator, 2)), ...
    get_param(model_name + "/reset_sampling", "Handle"));
end

% Mutation caught: a nonzero integral initial condition or a broken reset path
% that makes repeated nonzero-reference simulations start from stale state.
function test_reset_pulse_makes_lqi_logs_reproducible(test_case)
[model_name, cleanup] = load_built_model(); %#ok<ASGLU>
model_workspace = get_param(model_name, "ModelWorkspace");
assignin(model_workspace, "position_reference_data", [0.0, 0.25; 0.05, 0.25]);
assignin(model_workspace, "controller_reset_data", ...
    [0.0, 1.0; 0.001, 0.0; 0.02, 1.0; 0.021, 0.0; 0.05, 0.0]);

first_run = sim(model_name, "StopTime", "0.05");
second_run = sim(model_name, "StopTime", "0.05");
[first_integral_time, first_integral_values] = workspace_log_data( ...
    first_run, "position_integral_log");
[second_integral_time, second_integral_values] = workspace_log_data( ...
    second_run, "position_integral_log");
[first_raw_time, first_raw_values] = workspace_log_data(first_run, "u_raw_log");
[second_raw_time, second_raw_values] = workspace_log_data(second_run, "u_raw_log");

verifyEqual(test_case, first_integral_time, second_integral_time, "AbsTol", 1e-15);
verifyEqual(test_case, first_integral_values, second_integral_values, "AbsTol", 1e-14);
verifyEqual(test_case, first_raw_time, second_raw_time, "AbsTol", 1e-15);
verifyEqual(test_case, first_raw_values, second_raw_values, "AbsTol", 1e-14);
verifyEqual(test_case, first_integral_values(1), 0.0, "AbsTol", 1e-15);
verifyEqual(test_case, second_integral_values(1), 0.0, "AbsTol", 1e-15);
reset_index = find(abs(first_integral_time - 0.02) < 1e-12, 1);
verifyNotEmpty(test_case, reset_index);
verifyEqual(test_case, first_integral_values(reset_index), 0.0, "AbsTol", 1e-15);
verifyEqual(test_case, second_integral_values(reset_index), 0.0, "AbsTol", 1e-15);
end

% Mutation caught: weakening the shared solver/timing contract.
function test_model_uses_required_solver_timing(test_case)
[model_name, cleanup] = load_built_model(); %#ok<ASGLU>
verifyEqual(test_case, string(get_param(model_name, "SolverType")), ...
    "Variable-step");
verifyEqual(test_case, string(get_param(model_name, "Solver")), "ode45");
verifyEqual(test_case, str2double(get_param(model_name, "RelTol")), 1e-9, ...
    "AbsTol", 1e-20);
verifyEqual(test_case, str2double(get_param(model_name, "AbsTol")), 1e-11, ...
    "AbsTol", 1e-22);
verifyEqual(test_case, str2double(get_param(model_name, "MaxStep")), 0.001, ...
    "AbsTol", 1e-15);
verifyEqual(test_case, str2double(get_param(model_name, "StopTime")), 8.0, ...
    "AbsTol", 1e-15);
verifyEqual(test_case, string(get_param(model_name, "ReturnWorkspaceOutputs")), "on");
end

% Mutation caught: bypassing saturation or composing against a non-shared plant.
function test_model_uses_saturation_and_shared_nonlinear_plant(test_case)
[plant, params] = lqi_fixture();
[model_name, cleanup] = load_built_model(plant, params); %#ok<ASGLU>
saturation = model_name + "/actuator_saturation";
plant_path = model_name + "/nonlinear_plant";
verifyEqual(test_case, str2double(get_param(saturation, "UpperLimit")), ...
    plant.u_max, "AbsTol", 1e-15);
verifyEqual(test_case, str2double(get_param(saturation, "LowerLimit")), ...
    -plant.u_max, "AbsTol", 1e-15);
verifyEqual(test_case, source_block(input_port(saturation, 1)), ...
    get_param(model_name + "/controller_sum", "Handle"));
verifyEqual(test_case, source_block(input_port(plant_path, 1)), ...
    get_param(saturation, "Handle"));
verifyEqual(test_case, string(get_param(plant_path, "BlockType")), "SubSystem");
verifyEqual(test_case, string(get_param(plant_path + "/state_integrator", ...
    "BlockType")), "Integrator");
end

% Mutation caught: missing deterministic workspace input/log or unsafe identifiers.
function test_model_has_workspace_inputs_logs_and_safe_identifiers(test_case)
[model_name, cleanup] = load_built_model(); %#ok<ASGLU>
model_workspace = get_param(model_name, "ModelWorkspace");
required_inputs = ["position_reference_data"; "controller_reset_data"; ...
    "disturbance_force_data"; "disturbance_torque_data"];
for index = 1:numel(required_inputs)
    exists = evalin(model_workspace, ...
        "exist('" + required_inputs(index) + "','var')");
    verifyEqual(test_case, exists, 1, required_inputs(index));
end
reset_data = evalin(model_workspace, "controller_reset_data");
verifyEqual(test_case, reset_data, [0.0, 1.0; 0.001, 0.0; 8.0, 0.0], ...
    "AbsTol", 1e-15);

required_logs = ["state_log"; "position_reference_log"; ...
    "position_error_log"; "position_integral_log"; "u_raw_log"; "u_log"; ...
    "disturbance_force_log"; "disturbance_torque_log"];
workspace_sinks = find_system(model_name, "SearchDepth", 1, ...
    "BlockType", "ToWorkspace");
logged_variables = string(get_param(workspace_sinks, "VariableName"));
verifyTrue(test_case, all(ismember(required_logs, logged_variables)));
for index = 1:numel(workspace_sinks)
    verifyEqual(test_case, string(get_param(workspace_sinks{index}, ...
        "SaveFormat")), "Structure With Time");
end

assert_english_safe_names(test_case, model_name);
end

% Mutation caught: accepting a controller rate that violates unified timing.
function test_builder_rejects_nonstandard_timing(test_case)
[plant, params] = lqi_fixture();
params.sample_time = 0.02;
verifyError(test_case, @() build_lqi_simulink(plant, params), ...
    "twsbr:lqi_simulation:invalid_timing");
end

% Mutation caught: reusing nominal plant dynamics when a supplied plant differs.
function test_builder_embeds_custom_plant_and_restores_nominal_model(test_case)
plant = twsbr_params(struct("motor_force_gain", 13.0));
config = experiment_config("quick");
params = decode_controller_vector("LQI", ...
    log10([10, 1, 200, 10, 100, 0.1]), plant, config);
try
    model_path = build_lqi_simulink(plant, params);
    load_system(model_path);
    chart = find_chart("twsbr_lqi/nonlinear_plant/plant_dynamics");
    verifyNotEmpty(test_case, regexp(string(chart.Script), ...
        "motor_gain\s*=\s*13;", "once"));
catch cause
    rebuild_nominal_lqi();
    rethrow(cause);
end
rebuild_nominal_lqi();
end

function [plant, params] = lqi_fixture()
plant = twsbr_params();
config = experiment_config("quick");
params = decode_controller_vector("LQI", ...
    log10([10, 1, 200, 10, 100, 0.1]), plant, config);
end

function [model_name, cleanup] = load_built_model(plant, params)
if nargin < 1
    [plant, params] = lqi_fixture();
end
model_path = build_lqi_simulink(plant, params);
model_name = "twsbr_lqi";
load_system(model_path);
cleanup = onCleanup(@() close_if_loaded(model_name));
end

function port_handle = input_port(block_path, index)
ports = get_param(block_path, "PortHandles");
port_handle = ports.Inport(index);
end

function block_handle = source_block(port_handle)
line_handle = get_param(port_handle, "Line");
block_handle = get_param(line_handle, "SrcBlockHandle");
end

function port_handle = source_port(input_handle)
line_handle = get_param(input_handle, "Line");
port_handle = get_param(line_handle, "SrcPortHandle");
end

function [time, values] = workspace_log_data(simulation_output, variable_name)
log = simulation_output.get(char(variable_name));
time = log.time;
values = log.signals.values;
end

function assert_english_safe_names(test_case, model_name)
blocks = find_system(model_name, "Type", "Block");
for index = 1:numel(blocks)
    assert_safe_name(test_case, string(get_param(blocks{index}, "Name")));
end
ports = [find_system(model_name, "BlockType", "Inport"); ...
    find_system(model_name, "BlockType", "Outport")];
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

function chart = find_chart(block_path)
stateflow_root = sfroot;
chart = stateflow_root.find( ...
    "-isa", "Stateflow.EMChart", "Path", char(block_path));
if isempty(chart)
    error("twsbr:test:chart_not_found", ...
        "MATLAB Function chart was not found: %s", block_path);
end
end

function rebuild_nominal_lqi()
close_if_loaded("twsbr_lqi");
close_if_loaded("twsbr_plant");
build_lqi_simulink();
end

function present = is_simulink_block_present(model_name, block_name)
present = getSimulinkBlockHandle(model_name + "/" + block_name) > 0;
end

function absolute = is_absolute_path(file_path)
file_path = char(file_path);
absolute = ~isempty(regexp(file_path, "^[A-Za-z]:[\\/]", "once")) || ...
    startsWith(file_path, "\\\\");
end

function close_if_loaded(model_name)
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
end
