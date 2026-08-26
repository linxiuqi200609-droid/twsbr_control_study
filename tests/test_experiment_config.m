function tests = test_experiment_config
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
project_root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(project_root, "-begin");
setup_project();
end

function teardownOnce(test_case)
path(test_case.TestData.original_path);
end

function test_quick_configuration_is_frozen(test_case)
config = experiment_config("quick");
verifyEqual(test_case, config.mode, "quick");
verifyEqual(test_case, config.controller_names, ...
    ["ATTITUDE_PID"; "CASCADE_PID"; "FUZZY_PID"; "LQR"; "LQI"]);
verifyEqual(test_case, config.sample_time, 0.01, "AbsTol", 1e-15);
verifyEqual(test_case, config.plant_step, 0.001, "AbsTol", 1e-15);
verifyEqual(test_case, config.population_size, 24);
verifyEqual(test_case, config.evaluation_budget, 240);
verifyEqual(test_case, config.tuning_seeds, 0);
verifyEqual(test_case, config.monte_carlo_runs, 10);
end

function test_full_configuration_is_frozen(test_case)
config = experiment_config("full");
verifyEqual(test_case, config.population_size, 40);
verifyEqual(test_case, config.evaluation_budget, 3200);
verifyEqual(test_case, config.tuning_seeds, 0:9);
verifyEqual(test_case, config.monte_carlo_runs, 200);
end

function test_unknown_mode_is_rejected(test_case)
verifyError(test_case, @() experiment_config("draft"), ...
    "twsbr:study:invalid_mode");
end
