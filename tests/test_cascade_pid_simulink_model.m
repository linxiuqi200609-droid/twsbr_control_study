function tests = test_cascade_pid_simulink_model
%TEST_CASCADE_PID_SIMULINK_MODEL Structural tests for the cascade model.
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function teardownOnce(test_case)
close_if_loaded("twsbr_cascade_pid");
path(test_case.TestData.original_path);
end

function test_builder_creates_model_at_project_model_path(test_case)
paths = setup_project();
expected_path = fullfile(paths.model_directory, "twsbr_cascade_pid.slx");

model_path = build_cascade_pid_simulink();

verifyEqual(test_case, string(model_path), string(expected_path));
verifyTrue(test_case, isfile(model_path));
verifyTrue(test_case, is_absolute_path(model_path));
end

function test_model_has_discrete_cascade_and_shared_actuator(test_case)
[model_name, cleanup] = load_built_model(); %#ok<ASGLU>
params = cascade_pid_params();

verifyEqual(test_case, string( ...
    get_param(model_name + "/reference_sampling", "BlockType")), ...
    "ZeroOrderHold");
verifyEqual(test_case, ...
    str2double(get_param(model_name + "/reference_sampling", "SampleTime")), ...
    params.sample_time, "AbsTol", 1e-15);
verifyEqual(test_case, ...
    str2double(get_param(model_name + "/state_sampling", "SampleTime")), ...
    params.sample_time, "AbsTol", 1e-15);
verifyEqual(test_case, ...
    str2double(get_param(model_name + "/reset_sampling", "SampleTime")), ...
    params.sample_time, "AbsTol", 1e-15);

saturation_path = model_name + "/actuator_saturation";
verifyEqual(test_case, string(get_param(saturation_path, "BlockType")), ...
    "Saturate");
verifyEqual(test_case, str2double(get_param(saturation_path, "UpperLimit")), ...
    params.u_max, "AbsTol", 1e-15);
verifyEqual(test_case, str2double(get_param(saturation_path, "LowerLimit")), ...
    -params.u_max, "AbsTol", 1e-15);

controller_path = model_name + "/cascade_controller";
controller_chart = find_chart(controller_path);
script = string(controller_chart.Script);
verifyNotEmpty(test_case, regexp(script, ...
    "persistent\s+position_integral_state", "once"));
verifyNotEmpty(test_case, regexp(script, ...
    "reset\s*~=\s*0", "once"));
verifyNotEmpty(test_case, regexp(script, ...
    "position_integral_state\s*=\s*0\.0", "once"));
verifyNotEmpty(test_case, regexp(script, ...
    "position_integral_state\s*=\s*position_integral_state\s*\+", "once"));

plant_path = model_name + "/nonlinear_plant";
verifyEqual(test_case, string(get_param(plant_path, "BlockType")), ...
    "SubSystem");
verifyEqual(test_case, string( ...
    get_param(plant_path + "/state_integrator", "BlockType")), ...
    "Integrator");
plant_script = string(find_chart(plant_path + "/plant_dynamics").Script);
verifyNotEmpty(test_case, regexp(plant_script, ...
    "theta_dot\^2\s*\*\s*sin\(theta\)", "once"));
verifyNotEmpty(test_case, regexp(plant_script, ...
    "determinant\s*=\s*mass_sum", "once"));

controller_ports = get_param(controller_path, "PortHandles");
saturation_ports = get_param(saturation_path, "PortHandles");
plant_ports = get_param(plant_path, "PortHandles");
verifyEqual(test_case, source_block(saturation_ports.Inport), ...
    get_param(controller_path, "Handle"));
verifyEqual(test_case, source_block(plant_ports.Inport(1)), ...
    get_param(saturation_path, "Handle"));
verifyEqual(test_case, numel(controller_ports.Outport), 6);
end

function test_model_names_ports_lines_and_logs_in_english(test_case)
[model_name, cleanup] = load_built_model(); %#ok<ASGLU>

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

root_lines = find_system(model_name, "SearchDepth", 1, ...
    "FindAll", "on", "Type", "line");
line_names = strings(numel(root_lines), 1);
for index = 1:numel(root_lines)
    line_names(index) = string(get_param(root_lines(index), "Name"));
    verifyNotEmpty(test_case, regexp(line_names(index), ...
        "^[A-Za-z][A-Za-z0-9_]*$", "once"), line_names(index));
end
required_signals = ["position_reference", "position", "tilt", ...
    "theta_reference", "angular_rate", "u_raw", "u", ...
    "position_integral", "disturbance_force"];
verifyTrue(test_case, all(ismember(required_signals, line_names)));

required_logs = ["state_log", "position_reference_log", ...
    "position_error_log", "theta_reference_raw_log", ...
    "theta_reference_log", "theta_error_log", "u_raw_log", "u_log", ...
    "position_integral_log", "disturbance_force_log"];
workspace_sinks = find_system(model_name, "BlockType", "ToWorkspace");
logged_variables = string(get_param(workspace_sinks, "VariableName"));
verifyTrue(test_case, all(ismember(required_logs, logged_variables)));
end

function test_model_uses_explicit_model_workspace_inputs(test_case)
[model_name, cleanup] = load_built_model(); %#ok<ASGLU>
model_workspace = get_param(model_name, "ModelWorkspace");
required_variables = ["position_reference_data", "controller_reset_data", ...
    "disturbance_force_data", "disturbance_torque_data"];
for index = 1:numel(required_variables)
    exists = evalin(model_workspace, ...
        "exist('" + required_variables(index) + "','var')");
    verifyEqual(test_case, exists, 1, required_variables(index));
end

source_blocks = find_system(model_name, "BlockType", "FromWorkspace");
source_variables = string(get_param(source_blocks, "VariableName"));
verifyTrue(test_case, all(ismember(required_variables, source_variables)));
end

function [model_name, cleanup] = load_built_model()
model_path = build_cascade_pid_simulink();
model_name = "twsbr_cascade_pid";
load_system(model_path);
cleanup = onCleanup(@() close_if_loaded(model_name));
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

function block_handle = source_block(port_handle)
line_handle = get_param(port_handle, "Line");
block_handle = get_param(line_handle, "SrcBlockHandle");
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
