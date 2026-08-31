function tests = test_unified_plant_equivalence
%TEST_UNIFIED_PLANT_EQUIVALENCE Tests for the common nonlinear simulator.
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

function test_common_simulator_matches_existing_rk4_step_and_schema(test_case)
plant = twsbr_params();
config = experiment_config("quick");
scenario = make_scenario("one_step", [0; 0; deg2rad(5); 0], 0.01, ...
    zeros(4, 1));
vector = log10([0.1, 1e-4, 0.01]);

result = simulate_control_system("ATTITUDE_PID", vector, plant, ...
    config, scenario, 7);
expected = twsbr_rk4_step(scenario.initial_state, result.u(1), ...
    config.plant_step, plant, result.force_disturbance(1), ...
    result.torque_disturbance(1));

verifyEqual(test_case, result.state(2, :).', expected, "AbsTol", 1e-14);
verifyEqual(test_case, fieldnames(result), { ...
    'controller_name'; 'scenario_name'; 'seed'; 'time'; 'state'; ...
    'position_reference'; 'theta_reference'; 'u_raw'; 'u'; ...
    'saturated'; 'force_disturbance'; 'torque_disturbance'; ...
    'diagnostics'; 'success'; 'failure_reason'; 'survived_time'; ...
    'runtime_seconds'; 'controller_runtime_seconds'; ...
    'controller_evaluation_count'; 'controller_completed_count'; 'controller_parameters'});
verifyEqual(test_case, result.time, (0:10).' * 0.001, "AbsTol", 0);
verifyEqual(test_case, numel(result.diagnostics), numel(result.time));
end

function test_local_noise_stream_is_deterministic_and_isolates_global_rng(test_case)
plant = twsbr_params();
config = experiment_config("quick");
vector = log10([1.9, 0.2, 0.18]);
scenario = make_scenario("noisy", zeros(4, 1), 0.02, ...
    [0; 0; 0.002; 0.01]);
original_rng = rng;
rng_cleanup = onCleanup(@() rng(original_rng));

rng(2468, "twister");
expected_global_draw = rand(1, 5);
rng(2468, "twister");
first = simulate_control_system("ATTITUDE_PID", vector, plant, ...
    config, scenario, 19);
actual_global_draw = rand(1, 5);
second = simulate_control_system("ATTITUDE_PID", vector, plant, ...
    config, scenario, 19);
different = simulate_control_system("ATTITUDE_PID", vector, plant, ...
    config, scenario, 20);

verifyEqual(test_case, actual_global_draw, expected_global_draw, "AbsTol", 0);
verifyEqual(test_case, second.state, first.state, "AbsTol", 0);
verifyEqual(test_case, second.u_raw, first.u_raw, "AbsTol", 0);
verifyGreaterThan(test_case, max(abs(different.u_raw - first.u_raw)), 0);
end

function test_seed_domain_rejects_noncanonical_values(test_case)
plant = twsbr_params();
config = experiment_config("quick");
vector = log10([0.1, 1e-4, 0.01]);
scenario = make_scenario("seed_domain", zeros(4, 1), 0.01, ...
    zeros(4, 1));

invalid_seeds = {-1, 0.5, 2^32};
for index = 1:numel(invalid_seeds)
    verifyError(test_case, @() simulate_control_system("ATTITUDE_PID", ...
        vector, plant, config, scenario, invalid_seeds{index}), ...
        "twsbr:simulation:invalid_seed");
end
end

function test_zero_noise_is_exactly_seed_independent(test_case)
plant = twsbr_params();
config = experiment_config("quick");
scenario = make_scenario("zero_noise", [0; 0; 0.05; 0], 0.02, ...
    zeros(4, 1));
vector = log10([1.9, 0.2, 0.18]);

first = simulate_control_system("ATTITUDE_PID", vector, plant, ...
    config, scenario, 1);
second = simulate_control_system("ATTITUDE_PID", vector, plant, ...
    config, scenario, 999);

verifyEqual(test_case, second.state, first.state, "AbsTol", 0);
verifyEqual(test_case, second.u_raw, first.u_raw, "AbsTol", 0);
verifyEqual(test_case, second.u, first.u, "AbsTol", 0);
end

function test_raw_applied_and_diagnostics_are_held_for_ten_steps(test_case)
plant = twsbr_params();
config = experiment_config("quick");
scenario = make_scenario("saturated_hold", [0; 0; 0.2; 0], 0.011, ...
    zeros(4, 1));
vector = [2, -4, -2];

result = simulate_control_system("ATTITUDE_PID", vector, plant, ...
    config, scenario, 3);

verifyGreaterThan(test_case, result.u_raw(1), plant.u_max);
verifyEqual(test_case, result.u_raw(1:10), repmat(result.u_raw(1), 10, 1));
verifyEqual(test_case, result.u(1:10), plant.u_max * ones(10, 1));
verifyTrue(test_case, all(result.saturated(1:10)));
verifyEqual(test_case, result.theta_reference(1:10), zeros(10, 1));
verifyEqual(test_case, result.diagnostics(1:10), ...
    repmat(result.diagnostics(1), 10, 1));
verifyEqual(test_case, numel(result.diagnostics), numel(result.time));
end

function test_fixed_limits_truncate_at_the_logged_failure_sample(test_case)
plant = twsbr_params(struct("theta_fail_deg", 40, "x_limit", 8));
config = experiment_config("quick");
vector = log10([0.1, 1e-4, 0.01]);
position_scenario = make_scenario("position_limit", ...
    [5.001; 0; 0; 0], 0.01, zeros(4, 1));
tilt_scenario = make_scenario("tilt_limit", ...
    [0; 0; deg2rad(30.01); 0], 0.01, zeros(4, 1));

position_result = simulate_control_system("ATTITUDE_PID", vector, ...
    plant, config, position_scenario, 0);
tilt_result = simulate_control_system("ATTITUDE_PID", vector, ...
    plant, config, tilt_scenario, 0);

verifyFalse(test_case, position_result.success);
verifyEqual(test_case, position_result.failure_reason, "position_limit");
verifyEqual(test_case, position_result.time, 0);
verifyEqual(test_case, position_result.survived_time, 0);
verifyEqual(test_case, position_result.state, position_scenario.initial_state.');
verifyFalse(test_case, tilt_result.success);
verifyEqual(test_case, tilt_result.failure_reason, "tilt_limit");
verifyEqual(test_case, tilt_result.time, 0);
verifyEqual(test_case, tilt_result.survived_time, 0);
end

function test_nonfinite_control_stops_without_publishing_partial_sample(test_case)
plant = twsbr_params();
config = experiment_config("quick");
scenario = make_scenario("nonfinite_control", ...
    [0; 0; 0; realmax], 0.01, zeros(4, 1));
vector = [2, -4, 1.5];

result = simulate_control_system("ATTITUDE_PID", vector, plant, ...
    config, scenario, 5);

verifyFalse(test_case, result.success);
verifyEqual(test_case, result.failure_reason, "nonfinite_control");
verifyEmpty(test_case, result.time);
verifySize(test_case, result.state, [0, 4]);
verifyEmpty(test_case, result.diagnostics);
verifyEqual(test_case, result.survived_time, 0);
end

function test_near_singular_plant_returns_truncated_failure(test_case)
plant = twsbr_params(struct( ...
    "body_mass", 1e-6, ...
    "wheel_mass_equiv", 1e-6, ...
    "com_length", 1e-6, ...
    "body_inertia", 1e-13));
config = experiment_config("quick");
scenario = make_scenario("near_singular_plant", zeros(4, 1), ...
    0.01, zeros(4, 1));
vector = log10([0.1, 1e-4, 0.01]);

result = simulate_control_system("ATTITUDE_PID", vector, plant, ...
    config, scenario, 0);

verifyFalse(test_case, result.success);
verifyEqual(test_case, result.failure_reason, "singular_mass_matrix");
verifyEqual(test_case, result.time, 0);
verifyEqual(test_case, result.state, scenario.initial_state.');
verifyEqual(test_case, result.survived_time, 0);
verifyEqual(test_case, numel(result.diagnostics), 1);
end

function test_inputs_are_validated_before_simulation(test_case)
plant = twsbr_params();
config = experiment_config("quick");
vector = log10([0.1, 1e-4, 0.01]);
scenario = make_scenario("valid", zeros(4, 1), 0.01, zeros(4, 1));

bad_shape = scenario;
bad_shape.initial_state = zeros(1, 4);
verifyError(test_case, @() simulate_control_system("ATTITUDE_PID", ...
    vector, plant, config, bad_shape, 0), ...
    "twsbr:simulation:invalid_scenario");
bad_duration = scenario;
bad_duration.duration = 0.0105;
verifyError(test_case, @() simulate_control_system("ATTITUDE_PID", ...
    vector, plant, config, bad_duration, 0), ...
    "twsbr:simulation:invalid_duration");
bad_handle = scenario;
bad_handle.force_disturbance = 0;
verifyError(test_case, @() simulate_control_system("ATTITUDE_PID", ...
    vector, plant, config, bad_handle, 0), ...
    "twsbr:simulation:invalid_scenario");
bad_config = config;
bad_config.sample_time = 0.02;
verifyError(test_case, @() simulate_control_system("ATTITUDE_PID", ...
    vector, plant, bad_config, scenario, 0), ...
    "twsbr:simulation:invalid_timing");
nonscalar_config = config;
nonscalar_config.plant_step = [0.001, 0.001];
verifyError(test_case, @() simulate_control_system("ATTITUDE_PID", ...
    vector, plant, nonscalar_config, scenario, 0), ...
    "twsbr:simulation:invalid_config");
verifyError(test_case, @() simulate_control_system("ATTITUDE_PID", ...
    vector, rmfield(plant, "u_max"), config, scenario, 0), ...
    "twsbr:simulation:invalid_plant_params");
verifyError(test_case, @() simulate_control_system("ATTITUDE_PID", ...
    vector, plant, config, scenario, NaN), ...
    "twsbr:simulation:invalid_seed");
end

function scenario = make_scenario(name, initial_state, duration, noise_std)
scenario = struct( ...
    "name", name, ...
    "duration", duration, ...
    "initial_state", initial_state, ...
    "x_reference", @(time) 0.25 .* double(time >= 0.005), ...
    "force_disturbance", @(time) 2.0 .* double(time < 0.001), ...
    "torque_disturbance", @(time) 0.1 .* double(time < 0.001), ...
    "measurement_noise_std", noise_std);
end
