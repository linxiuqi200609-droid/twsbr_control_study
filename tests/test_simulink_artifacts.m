function tests = test_simulink_artifacts
%TEST_SIMULINK_ARTIFACTS Verify generated models use the configured directory.
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function teardownOnce(test_case)
close_if_loaded("twsbr_attitude_pid");
close_if_loaded("twsbr_plant");
path(test_case.TestData.original_path);
end

function test_plant_builder_saves_to_configured_model_directory(test_case)
paths = setup_project();
expected_path = fullfile(paths.model_directory, "twsbr_plant.slx");

model_path = build_twsbr_simulink();

verifyEqual(test_case, string(model_path), expected_path);
verifyTrue(test_case, isfile(expected_path));
end

function test_attitude_builder_saves_to_configured_model_directory(test_case)
paths = setup_project();
expected_path = fullfile(paths.model_directory, "twsbr_attitude_pid.slx");

model_path = build_attitude_pid_simulink();

verifyEqual(test_case, string(model_path), expected_path);
verifyTrue(test_case, isfile(expected_path));
end

function close_if_loaded(model_name)
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
end
