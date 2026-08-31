function tests = test_controller_runtime
%TEST_CONTROLLER_RUNTIME Lifecycle timing excludes plant samples and setup.
tests = functiontests(localfunctions);
end

function setupOnce(test_case)
test_case.TestData.original_path = path;
addpath(fileparts(fileparts(mfilename("fullpath"))));
setup_project();
end

function teardownOnce(test_case)
path(test_case.TestData.original_path);
end

function test_schedule_counts_controller_updates_not_plant_samples(test_case)
simulation = run_fixture(zeros(4,1));
verifyEqual(test_case, numel(simulation.time), 41);
verifyEqual(test_case, simulation.controller_evaluation_count, 5);
verifyEqual(test_case, simulation.controller_completed_count, 5);
verifyGreaterThanOrEqual(test_case, simulation.controller_runtime_seconds, 0);
verifyLessThanOrEqual(test_case, simulation.controller_runtime_seconds, simulation.runtime_seconds);
end

function test_truncated_run_counts_only_attempted_updates(test_case)
simulation = run_fixture([6;0;0;0]);
verifyFalse(test_case, simulation.success);
verifyEqual(test_case, numel(simulation.time), 1);
verifyEqual(test_case, simulation.controller_evaluation_count, 1);
verifyEqual(test_case, simulation.controller_completed_count, 1);
end

function test_no_attempt_is_distinct_from_a_failed_first_attempt(test_case)
simulation = run_fixture([NaN;0;0;0]);
verifyEqual(test_case, simulation.controller_evaluation_count, 0);
verifyEqual(test_case, simulation.controller_completed_count, 0);
verifyEqual(test_case, simulation.controller_runtime_seconds, 0);
metrics = calculate_control_metrics(simulation, fixture_scenario(), twsbr_params());
verifyTrue(test_case, isnan(metrics.mean_step_runtime_us));
verifyFalse(test_case, metrics.success);

failed = run_fixture([0;0;0;realmax]);
verifyEqual(test_case, failed.failure_reason, "nonfinite_control");
verifyEqual(test_case, failed.controller_evaluation_count, 1);
verifyEqual(test_case, failed.controller_completed_count, 0);
metrics = calculate_control_metrics(failed, fixture_scenario(), twsbr_params());
verifyEqual(test_case, metrics.mean_step_runtime_us, failed.controller_runtime_seconds * 1e6);
end

function test_exact_metric_conversion_and_invalid_lifecycle_contract(test_case)
simulation = run_fixture(zeros(4,1));
simulation.runtime_seconds = 4;
simulation.controller_runtime_seconds = 0.002;
simulation.controller_evaluation_count = 5;
simulation.controller_completed_count = 5;
metrics = calculate_control_metrics(simulation, fixture_scenario(), twsbr_params());
verifyEqual(test_case, metrics.simulation_runtime_seconds, 4);
verifyEqual(test_case, metrics.controller_runtime_seconds, 0.002);
verifyEqual(test_case, metrics.mean_step_runtime_us, 400);
bad = {rmfield(simulation, "controller_evaluation_count"), ...
    setfield(simulation, "controller_evaluation_count", 1.5), ... %#ok<SFLD>
    setfield(simulation, "controller_completed_count", 6), ... %#ok<SFLD>
    setfield(simulation, "controller_runtime_seconds", 5)}; %#ok<SFLD>
for index = 1:numel(bad)
    verifyError(test_case, @() calculate_control_metrics(bad{index}, ...
        fixture_scenario(), twsbr_params()), "twsbr:metrics:invalid_input");
end
end

function simulation = run_fixture(initial_state)
scenario = fixture_scenario();
scenario.initial_state = initial_state;
simulation = simulate_control_system("LQR", log10([10,1,200,10,0.1]), ...
    twsbr_params(), experiment_config("quick"), scenario, 0);
end

function test_completed_updates_must_be_consistent_with_published_log(test_case)
simulation = run_fixture(zeros(4,1));
simulation.controller_evaluation_count = 42;
simulation.controller_completed_count = 42;
verifyError(test_case, @() calculate_control_metrics(simulation, ...
    fixture_scenario(), twsbr_params()), "twsbr:metrics:invalid_input");
simulation.controller_evaluation_count = 0;
simulation.controller_completed_count = 0;
simulation.controller_runtime_seconds = 0;
simulation.success = false;
simulation.failure_reason = "nonfinite_control";
verifyError(test_case, @() calculate_control_metrics(simulation, ...
    fixture_scenario(), twsbr_params()), "twsbr:metrics:invalid_input");
end

function test_zero_attempt_failure_survives_csv_and_workbook_roundtrip(test_case)
simulation = run_fixture([NaN;0;0;0]);
metrics = calculate_control_metrics(simulation, fixture_scenario(), twsbr_params());
data = struct2table(metrics);
directory = string(tempname);
mkdir(directory);
cleanup = onCleanup(@() rmdir(directory, "s"));
paths = write_results_tables(directory, data, data, data, data, data, data);
csv = readtable(paths.deterministic_csv);
workbook = readtable(paths.statistics_xlsx, "Sheet", "deterministic_raw");
for item = {csv, workbook}
    record = item{1};
    verifyEqual(test_case, height(record), 1);
    verifyFalse(test_case, logical(record.success));
    verifyEqual(test_case, string(record.failure_reason), "nonfinite_state");
    verifyEqual(test_case, record.controller_evaluation_count, 0);
    verifyEqual(test_case, record.controller_completed_count, 0);
    verifyEqual(test_case, record.controller_runtime_seconds, 0);
    verifyTrue(test_case, isnan(record.mean_step_runtime_us));
end
delete(cleanup)
end

function scenario = fixture_scenario()
scenarios = training_scenarios(3.2);
scenario = scenarios.T1_initial_tilt_5deg;
scenario.duration = 0.04;
end
