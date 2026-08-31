function tests = test_lqr_simulink_model
%TEST_LQR_SIMULINK_MODEL Structural tests for the generated LQR model.
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function teardownOnce(test_case)
close_if_loaded("twsbr_lqr");
path(test_case.TestData.original_path);
end

function test_builder_creates_expected_project_model(test_case)
[plant, params] = lqr_fixture();
paths = setup_project();

model_path = build_lqr_simulink(plant, params);

verifyEqual(test_case, string(model_path), ...
    string(fullfile(paths.model_directory, "twsbr_lqr.slx")));
verifyTrue(test_case, isfile(model_path));
verifyTrue(test_case, is_absolute_path(model_path));
end

function test_model_has_sampled_full_state_lqr_structure(test_case)
[plant, params] = lqr_fixture();
[model_name, cleanup] = load_built_model(plant, params); %#ok<ASGLU>

required_blocks = ["position_reference"; "reference_sampling"; ...
    "state_sampling"; "reference_position_rate"; ...
    "reference_tilt"; "reference_angular_rate"; "reference_state"; ...
    "state_error"; "lqr_gain"; "actuator_saturation"; ...
    "nonlinear_plant"; "disturbance_force"; "disturbance_torque"; ...
    "state_log"; "position_reference_log"; "u_raw_log"; "u_log"; ...
    "disturbance_force_log"; "disturbance_torque_log"; "state"];
blocks = string(get_param(find_system(model_name, "SearchDepth", 1, ...
    "Type", "Block"), "Name"));
verifyTrue(test_case, all(ismember(required_blocks, blocks)));

verifyEqual(test_case, string(get_param( ...
    model_name + "/reference_sampling", "BlockType")), "ZeroOrderHold");
verifyEqual(test_case, string(get_param( ...
    model_name + "/state_sampling", "BlockType")), "ZeroOrderHold");
verifyEqual(test_case, str2double(get_param( ...
    model_name + "/reference_sampling", "SampleTime")), ...
    params.sample_time, "AbsTol", 1e-15);
verifyEqual(test_case, str2double(get_param( ...
    model_name + "/state_sampling", "SampleTime")), ...
    params.sample_time, "AbsTol", 1e-15);
config = experiment_config("quick");
verifyEqual(test_case, str2double(get_param(model_name, "MaxStep")), ...
    config.plant_step, "AbsTol", 1e-15);
verifyEqual(test_case, string(get_param(model_name, "SolverType")), "Variable-step");
verifyEqual(test_case, string(get_param(model_name, "Solver")), "ode45");
verifyEqual(test_case, str2double(get_param(model_name, "RelTol")), 1e-9);
verifyEqual(test_case, str2double(get_param(model_name, "AbsTol")), 1e-11);
verifyEqual(test_case, string(get_param(model_name, "ReturnWorkspaceOutputs")), "on");
verifyEqual(test_case, str2double(get_param(model_name, "StopTime")), 8);

verifyEqual(test_case, string(get_param( ...
    model_name + "/reference_state", "BlockType")), "Mux");
verifyEqual(test_case, string(get_param( ...
    model_name + "/state_error", "Inputs")), "+-");
verifyEqual(test_case, string(get_param( ...
    model_name + "/lqr_gain", "Gain")), string(mat2str(-params.gain, 17)));
verifyEqual(test_case, string(get_param( ...
    model_name + "/lqr_gain", "Multiplication")), "Matrix(K*u)");

saturation_path = model_name + "/actuator_saturation";
plant_path = model_name + "/nonlinear_plant";
verifyEqual(test_case, str2double(get_param(saturation_path, "UpperLimit")), ...
    plant.u_max, "AbsTol", 1e-15);
verifyEqual(test_case, str2double(get_param(saturation_path, "LowerLimit")), ...
    -plant.u_max, "AbsTol", 1e-15);
plant_ports = get_param(plant_path, "PortHandles");
verifyEqual(test_case, source_block(plant_ports.Inport(1)), ...
    get_param(saturation_path, "Handle"));
verifyEqual(test_case, string(get_param(plant_path, "BlockType")), "SubSystem");
verifyEqual(test_case, string(get_param( ...
    plant_path + "/state_integrator", "BlockType")), "Integrator");
end

function test_builder_rejects_nonstandard_timing(test_case)
[plant, params] = lqr_fixture();
params.sample_time = 0.02;

verifyError(test_case, @() build_lqr_simulink(plant, params), ...
    "twsbr:lqr_simulation:invalid_timing");
end

function test_builder_embeds_supplied_plant_dynamics(test_case)
plant = twsbr_params(struct("motor_force_gain", 13.0));
config = experiment_config("quick");
params = decode_controller_vector("LQR", ...
    log10([10, 1, 200, 10, 0.1]), plant, config);

try
    model_path = build_lqr_simulink(plant, params);
    load_system(model_path);
    chart = find_chart("twsbr_lqr/nonlinear_plant/plant_dynamics");

    verifyNotEmpty(test_case, regexp(string(chart.Script), ...
        "motor_gain\s*=\s*13;", "once"));
catch cause
    rebuild_nominal_lqr();
    rethrow(cause);
end
rebuild_nominal_lqr();
end

function test_model_has_workspace_inputs_logs_and_english_identifiers(test_case)
[plant, params] = lqr_fixture();
[model_name, cleanup] = load_built_model(plant, params); %#ok<ASGLU>

model_workspace = get_param(model_name, "ModelWorkspace");
required_inputs = ["position_reference_data"; "disturbance_force_data"; ...
    "disturbance_torque_data"];
for index = 1:numel(required_inputs)
    exists = evalin(model_workspace, ...
        "exist('" + required_inputs(index) + "','var')");
    verifyEqual(test_case, exists, 1, required_inputs(index));
end
source_blocks = find_system(model_name, "SearchDepth", 1, ...
    "BlockType", "FromWorkspace");
source_variables = string(get_param(source_blocks, "VariableName"));
verifyTrue(test_case, all(ismember(required_inputs, source_variables)));

required_logs = ["state_log"; "position_reference_log"; "u_raw_log"; ...
    "u_log"; "disturbance_force_log"; "disturbance_torque_log"];
workspace_sinks = find_system(model_name, "SearchDepth", 1, ...
    "BlockType", "ToWorkspace");
logged_variables = string(get_param(workspace_sinks, "VariableName"));
verifyTrue(test_case, all(ismember(required_logs, logged_variables)));
for index = 1:numel(workspace_sinks)
    verifyEqual(test_case, string(get_param(workspace_sinks{index}, ...
        "SaveFormat")), "Structure With Time");
end

blocks = find_system(model_name, "Type", "Block");
for index = 1:numel(blocks)
    name = string(get_param(blocks{index}, "Name"));
    verifyNotEmpty(test_case, regexp(name, ...
        "^[A-Za-z][A-Za-z0-9_]*$", "once"), name);
end
ports = [find_system(model_name, "BlockType", "Inport"); ...
    find_system(model_name, "BlockType", "Outport")];
for index = 1:numel(ports)
    name = string(get_param(ports{index}, "Name"));
    verifyNotEmpty(test_case, regexp(name, ...
        "^[A-Za-z][A-Za-z0-9_]*$", "once"), name);
end
line_handles = find_system(model_name, "FindAll", "on", "Type", "line");
for index = 1:numel(line_handles)
    name = string(get_param(line_handles(index), "Name"));
    verifyNotEmpty(test_case, regexp(name, ...
        "^[A-Za-z][A-Za-z0-9_]*$", "once"), name);
end
end

function [plant, params] = lqr_fixture()
plant = twsbr_params();
config = experiment_config("quick");
params = decode_controller_vector("LQR", ...
    log10([10, 1, 200, 10, 0.1]), plant, config);
end

function [model_name, cleanup] = load_built_model(plant, params)
model_path = build_lqr_simulink(plant, params);
model_name = "twsbr_lqr";
load_system(model_path);
cleanup = onCleanup(@() close_if_loaded(model_name));
end

function block_handle = source_block(port_handle)
line_handle = get_param(port_handle, "Line");
block_handle = get_param(line_handle, "SrcBlockHandle");
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

function rebuild_nominal_lqr()
close_if_loaded("twsbr_lqr");
close_if_loaded("twsbr_plant");
build_lqr_simulink();
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
