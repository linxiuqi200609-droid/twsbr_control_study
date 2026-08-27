function tests = test_control_objective
%TEST_CONTROL_OBJECTIVE Tests for the shared three-scenario objective.
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

function test_invalid_vector_returns_invalid_penalty(test_case)
config = experiment_config("quick");
[cost, details] = control_objective("ATTITUDE_PID", [NaN, 0, 0], ...
    twsbr_params(), config, objective_config(), training_scenarios(4), 0);

verifyEqual(test_case, cost, 1e6);
verifyEqual(test_case, details.failure_reason, ...
    "invalid_controller_vector");
verifyEqual(test_case, details.failed_index, 1);
verifyEqual(test_case, details.failed_scenario, ...
    "T1_initial_tilt_5deg");
verifyEqual(test_case, details.scenario_seeds, [0; 1; 2]);
end

function test_objective_is_deterministic_and_uses_three_scenarios(test_case)
config = experiment_config("quick");
vector = log10([1.9, 0.2, 0.18]);
scenarios = training_scenarios(4);
[first, first_details] = control_objective("ATTITUDE_PID", vector, ...
    twsbr_params(), config, objective_config(), scenarios, 12);
[second, second_details] = control_objective("ATTITUDE_PID", vector, ...
    twsbr_params(), config, objective_config(), scenarios, 12);

verifyEqual(test_case, second, first);
verifyEqual(test_case, second_details, first_details);
verifyEqual(test_case, numel(first_details.scenarios), 3);
verifyEqual(test_case, string({first_details.scenarios.name}).', ...
    ["T1_initial_tilt_5deg"; "T2_position_step_0p5m"; ...
    "T3_impulse_disturbance"]);
verifyEqual(test_case, first_details.scenario_seeds, [12; 13; 14]);
first_metrics = [first_details.scenarios.metrics];
verifyEqual(test_case, [first_metrics.seed].', [12; 13; 14]);
verifyEqual(test_case, first_details.failed_index, 0);
verifyEqual(test_case, first_details.failed_scenario, "");
end

function test_details_reproduce_objective_formula_and_penalty(test_case)
objective = objective_config();
scenarios = training_scenarios(4);
[cost, details] = control_objective("ATTITUDE_PID", ...
    log10([1.9, 0.2, 0.18]), twsbr_params(), ...
    experiment_config("quick"), objective, scenarios, 12);
scenario_names = ["T1_initial_tilt_5deg"; ...
    "T2_position_step_0p5m"; "T3_impulse_disturbance"];
expected_costs = zeros(3, 1);
has_failure = false;

for scenario_index = 1:3
    scenario = scenarios.(scenario_names(scenario_index));
    detail = details.scenarios(scenario_index);
    expected_values = zeros(numel(objective.metric_names), 1);
    for metric_index = 1:numel(objective.metric_names)
        metric_name = objective.metric_names(metric_index);
        expected_values(metric_index) = detail.metrics.(metric_name);
    end
    recovery_index = objective.metric_names == ...
        "disturbance_recovery_time";
    if scenario.disturbance_end == 0
        expected_values(recovery_index) = 0.0;
        verifyEqual(test_case, ...
            detail.contributions(recovery_index), 0.0);
    end
    expected_contributions = objective.weights .* ...
        (expected_values ./ objective.scales);
    expected_base = sum(expected_contributions);
    expected_penalty = 0.0;
    if ~detail.metrics.success
        has_failure = true;
        expected_penalty = objective.failure_penalty * ...
            (2 - detail.metrics.survived_time / scenario.duration);
    end
    expected_costs(scenario_index) = expected_base + expected_penalty;

    verifyEqual(test_case, detail.metric_values, expected_values, ...
        "AbsTol", 1e-12);
    verifyEqual(test_case, detail.contributions, ...
        expected_contributions, "AbsTol", 1e-12);
    verifyEqual(test_case, detail.base_cost, expected_base, ...
        "AbsTol", 1e-12);
    verifyEqual(test_case, detail.failure_penalty, expected_penalty, ...
        "AbsTol", 1e-12);
    verifyEqual(test_case, detail.scenario_cost, ...
        expected_costs(scenario_index), "AbsTol", 1e-12);
    verifyFalse(test_case, isfield(detail.metrics, ...
        "controller_runtime_seconds"));
    verifyFalse(test_case, isfield(detail.metrics, ...
        "mean_step_runtime_us"));
end

verifyTrue(test_case, has_failure);
verifyTrue(test_case, isfinite(cost));
verifyEqual(test_case, cost, mean(expected_costs), "AbsTol", 1e-12);
end

function test_seed_and_training_scenario_contracts(test_case)
plant = twsbr_params();
config = experiment_config("quick");
objective = objective_config();
scenarios = training_scenarios(4);
vector = log10([1.9, 0.2, 0.18]);
invalid_seeds = {-1, 0.5, 2^32 - 2};
for index = 1:numel(invalid_seeds)
    verifyError(test_case, @() control_objective("ATTITUDE_PID", ...
        vector, plant, config, objective, scenarios, ...
        invalid_seeds{index}), "twsbr:objective:invalid_seed");
end

bad_split = scenarios;
bad_split.T1_initial_tilt_5deg.split = "test";
bad_name = scenarios;
bad_name.T2_position_step_0p5m.name = "wrong_name";
renamed = rmfield(scenarios, "T3_impulse_disturbance");
renamed.S1_heldout = scenarios.T3_impulse_disturbance;
invalid_scenarios = {bad_split, bad_name, renamed};
for index = 1:numel(invalid_scenarios)
    verifyError(test_case, @() control_objective("ATTITUDE_PID", ...
        vector, plant, config, objective, invalid_scenarios{index}, 0), ...
        "twsbr:objective:invalid_scenarios");
end
end

function test_objective_configuration_is_frozen(test_case)
plant = twsbr_params();
config = experiment_config("quick");
scenarios = training_scenarios(4);
vector = log10([1.9, 0.2, 0.18]);
negative_weight = objective_config();
negative_weight.weights(1) = -0.1;
duplicate_metric = objective_config();
duplicate_metric.metric_names(2) = duplicate_metric.metric_names(1);
reordered_metrics = objective_config();
reordered_metrics.metric_names = flipud(reordered_metrics.metric_names);
small_invalid_penalty = objective_config();
small_invalid_penalty.invalid_penalty = ...
    2 * small_invalid_penalty.failure_penalty;
changed_weights = objective_config();
changed_weights.weights(1:2) = changed_weights.weights(1:2) + [0.01; -0.01];
changed_scales = objective_config();
changed_scales.scales(1) = changed_scales.scales(1) + 0.01;
changed_failure_penalty = objective_config();
changed_failure_penalty.failure_penalty = 501;
changed_invalid_penalty = objective_config();
changed_invalid_penalty.invalid_penalty = 1e6 + 1;
invalid_objectives = {negative_weight, duplicate_metric, ...
    reordered_metrics, small_invalid_penalty, changed_weights, ...
    changed_scales, changed_failure_penalty, changed_invalid_penalty};
for index = 1:numel(invalid_objectives)
    verifyError(test_case, @() control_objective("ATTITUDE_PID", ...
        vector, plant, config, invalid_objectives{index}, scenarios, 0), ...
        "twsbr:objective:invalid_config");
end
end

function test_programming_errors_rethrow(test_case)
plant = twsbr_params();
config = experiment_config("quick");
objective = objective_config();
scenarios = training_scenarios(4);
vector = log10([1.9, 0.2, 0.18]);

verifyError(test_case, @() control_objective("UNSUPPORTED", 0, ...
    plant, config, objective, scenarios, 0), ...
    "twsbr:controller:unsupported_name");
bad_config = config;
bad_config.sample_time = 0;
verifyError(test_case, @() control_objective("ATTITUDE_PID", vector, ...
    plant, bad_config, objective, scenarios, 0), ...
    "twsbr:simulation:invalid_config");
bad_scenarios = scenarios;
bad_scenarios.T1_initial_tilt_5deg = rmfield( ...
    bad_scenarios.T1_initial_tilt_5deg, "split");
verifyError(test_case, @() control_objective("ATTITUDE_PID", vector, ...
    plant, config, objective, bad_scenarios, 0), ...
    "twsbr:objective:invalid_scenarios");
end

function test_all_controller_starters_return_finite_three_scenario_cost(test_case)
[names, vectors] = controller_cases();
plant = twsbr_params();
config = experiment_config("quick");
objective = objective_config();
scenarios = training_scenarios(3.2);
expected_names = ["T1_initial_tilt_5deg"; ...
    "T2_position_step_0p5m"; "T3_impulse_disturbance"];

for index = 1:numel(names)
    [cost, details] = control_objective(names(index), vectors{index}, ...
        plant, config, objective, scenarios, 21);
    verifyTrue(test_case, isfinite(cost));
    verifyEqual(test_case, numel(details.scenarios), 3);
    verifyEqual(test_case, details.controller, names(index));
    verifyEqual(test_case, details.scenario_seeds, [21; 22; 23]);
    verifyEqual(test_case, string({details.scenarios.name}).', ...
        expected_names);
    scenario_metrics = [details.scenarios.metrics];
    verifyEqual(test_case, [scenario_metrics.seed].', ...
        [21; 22; 23]);
end
end

function [names, vectors] = controller_cases()
names = ["ATTITUDE_PID"; "CASCADE_PID"; "FUZZY_PID"; "LQR"; "LQI"];
vectors = { ...
    log10([1.9, 0.2, 0.18]), ...
    log10([0.24100028146267993, 0.0003962067755988572, ...
        0.1930824033173246, 9.254929149177556, ...
        1.0113335430173094]), ...
    [log10([0.24, 0.0004, 0.19, 9.25, 0.05, 1.01]), ...
        0.2, 0.2, 0.2], ...
    log10([10, 1, 200, 10, 0.1]), ...
    log10([10, 1, 200, 10, 100, 0.1])};
end