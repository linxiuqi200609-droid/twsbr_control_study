function tests = test_monte_carlo_pairing
%TEST_MONTE_CARLO_PAIRING Tests for deterministic paired MC generation.
local_functions = localfunctions;
function_names = string(cellfun(@func2str, local_functions, ...
    "UniformOutput", false));
tests = functiontests(local_functions(~endsWith( ...
    function_names, "starter_vectors_for_test")));
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

function test_same_run_is_repeatable_and_preserves_global_rng(test_case)
config = monte_carlo_config("quick");
nominal = twsbr_params();
original_rng = rng;
    test_case.TestData.rng_cleanup = onCleanup(@() rng(original_rng));
rng(7142, "twister");
expected_draw = rand(1, 8);
rng(7142, "twister");

[first_scenario, first_plant, first_metadata] = ...
    generate_monte_carlo_scenario(7, 9000, config, nominal);
actual_draw = rand(1, 8);
[second_scenario, second_plant, second_metadata] = ...
    generate_monte_carlo_scenario(7, 9000, config, nominal);

verifyEqual(test_case, actual_draw, expected_draw, "AbsTol", 0);
verifyEqual(test_case, second_plant, first_plant);
verifyEqual(test_case, second_metadata, first_metadata);
verify_scenarios_equal(test_case, second_scenario, first_scenario);
end

function test_generated_scenario_obeys_ranges_parity_and_half_open_timing(test_case)
config = monte_carlo_config("full");
nominal = twsbr_params();

[odd, ~, odd_metadata] = ...
    generate_monte_carlo_scenario(3, 1234, config, nominal);
[~, ~, even_metadata] = ...
    generate_monte_carlo_scenario(4, 1234, config, nominal);

verifyEqual(test_case, odd.name, "S4_monte_carlo");
verifyEqual(test_case, odd.split, "test");
verifyEqual(test_case, odd.duration, config.duration);
verifyEqual(test_case, odd.measurement_noise_std, config.noise_std);
verifyEqual(test_case, odd.initial_state([1, 2, 4]), zeros(3, 1));
verifyGreaterThanOrEqual(test_case, odd_metadata.initial_theta_deg, 2.0);
verifyLessThanOrEqual(test_case, odd_metadata.initial_theta_deg, 8.0);
verifyEqual(test_case, odd.initial_state(3), ...
    deg2rad(odd_metadata.initial_theta_deg), "AbsTol", 1e-15);
verifyGreaterThanOrEqual(test_case, odd_metadata.reference_amplitude, 0.25);
verifyLessThanOrEqual(test_case, odd_metadata.reference_amplitude, 1.0);
verifyEqual(test_case, odd.reference_start, 1.0);
verifyEqual(test_case, odd.x_reference([0.999, 1.0]), ...
    [0, odd_metadata.reference_amplitude]);
verifyGreaterThanOrEqual(test_case, abs(odd_metadata.force_amplitude), 4.0);
verifyLessThanOrEqual(test_case, abs(odd_metadata.force_amplitude), 9.0);
verifyLessThan(test_case, odd_metadata.force_amplitude, 0);
verifyGreaterThan(test_case, even_metadata.force_amplitude, 0);
verifyGreaterThanOrEqual(test_case, odd_metadata.force_start, 3.0);
verifyLessThanOrEqual(test_case, odd_metadata.force_start, 5.0);
verifyGreaterThanOrEqual(test_case, odd_metadata.force_duration, 0.10);
verifyLessThanOrEqual(test_case, odd_metadata.force_duration, 0.30);
verifyGreaterThanOrEqual(test_case, odd_metadata.constant_force, -0.5);
verifyLessThanOrEqual(test_case, odd_metadata.constant_force, 0.5);
verifyEqual(test_case, odd.force_amplitude, odd_metadata.force_amplitude);
verifyEqual(test_case, odd.force_start, odd_metadata.force_start);
verifyEqual(test_case, odd.force_duration, odd_metadata.force_duration);
verifyEqual(test_case, odd.constant_force, odd_metadata.constant_force);
verifyEqual(test_case, odd.disturbance_end, ...
    odd_metadata.force_start + odd_metadata.force_duration, ...
    "AbsTol", 1e-15);
verifyEqual(test_case, odd.force_disturbance(odd.force_start - eps(odd.force_start)), ...
    odd.constant_force);
verifyEqual(test_case, odd.force_disturbance(odd.force_start), ...
    odd.constant_force + odd.force_amplitude);
verifyEqual(test_case, odd.force_disturbance( ...
    odd.force_start + odd.force_duration / 2), ...
    odd.constant_force + odd.force_amplitude);
verifyEqual(test_case, odd.force_disturbance(odd.disturbance_end), ...
    odd.constant_force);
verifyEqual(test_case, odd.torque_amplitude, 0);
verifyEqual(test_case, odd.torque_disturbance([0, 4, 10]), [0, 0, 0]);
end

function test_parameter_perturbations_are_bounded_and_do_not_cross_talk(test_case)
config = monte_carlo_config("quick");
nominal = twsbr_params(struct( ...
    "wheel_mass_equiv", 0.37, ...
    "wheel_radius", 0.061, ...
    "viscous_damping", 0.073, ...
    "gravity", 9.79, ...
    "motor_force_gain", 11.2, ...
    "u_max", 0.92, ...
    "theta_fail_deg", 28.0, ...
    "x_limit", 4.4));

[~, perturbed, metadata] = ...
    generate_monte_carlo_scenario(8, 321, config, nominal);

delta_fields = ["delta_body_mass"; "delta_com_length"; ...
    "delta_body_inertia"];
plant_fields = ["body_mass"; "com_length"; "body_inertia"];
for index = 1:numel(delta_fields)
    delta = metadata.(delta_fields(index));
    verifyGreaterThanOrEqual(test_case, delta, -config.parameter_rho);
    verifyLessThanOrEqual(test_case, delta, config.parameter_rho);
    verifyEqual(test_case, perturbed.(plant_fields(index)), ...
        nominal.(plant_fields(index)) * (1 + delta), "RelTol", 1e-15);
end

unchanged_fields = setdiff(fieldnames(nominal), cellstr(plant_fields), ...
    "stable");
for index = 1:numel(unchanged_fields)
    field_name = unchanged_fields{index};
    verifyEqual(test_case, perturbed.(field_name), nominal.(field_name));
end
end

function test_metadata_is_complete_and_run_identity_changes_draws(test_case)
config = monte_carlo_config("quick");
nominal = twsbr_params();

[~, ~, first] = generate_monte_carlo_scenario(1, 700, config, nominal);
[~, ~, second] = generate_monte_carlo_scenario(2, 700, config, nominal);

expected_fields = [ ...
    "run_index"; "seed"; "rho"; ...
    "delta_body_mass"; "delta_com_length"; "delta_body_inertia"; ...
    "initial_theta_deg"; "reference_amplitude"; "force_amplitude"; ...
    "force_start"; "force_duration"; "constant_force"; ...
    "measurement_noise_seed"];
verifyEqual(test_case, string(fieldnames(first)), expected_fields);
verifyEqual(test_case, first.run_index, 1);
verifyEqual(test_case, first.seed, 701);
verifyEqual(test_case, first.measurement_noise_seed, 701);
verifyEqual(test_case, first.rho, config.parameter_rho);
verifyEqual(test_case, second.run_index, 2);
verifyEqual(test_case, second.seed, 702);
verifyNotEqual(test_case, second, first);
end

function test_integer_numeric_inputs_preserve_fractional_draws(test_case)
config = monte_carlo_config("quick");
config.parameter_rho = 0.5;
config.initial_tilt_deg_range = int32([2, 8]);
config.reference_range = int32([0, 1]);
config.force_range = int32([4, 9]);
config.force_start_range = int32([3, 5]);
config.force_duration_range = int32([1, 2]);
config.constant_force_range = int32([-1, 1]);
config.reference_start = int32(1);
config.duration = int32(8);
config.noise_std = int32(zeros(4, 1));
nominal = twsbr_params();
nominal.body_mass = int32(1);
nominal.com_length = int32(1);
nominal.body_inertia = int32(1);
[~, perturbed, metadata] = generate_monte_carlo_scenario(7, 9000, config, nominal);
verifyTrue(test_case, metadata.initial_theta_deg ~= floor(metadata.initial_theta_deg));
verifyTrue(test_case, metadata.reference_amplitude ~= floor(metadata.reference_amplitude));
verifyTrue(test_case, perturbed.body_mass ~= nominal.body_mass);
end

function test_invalid_indices_seeds_config_and_nominal_have_stable_errors(test_case)
config = monte_carlo_config("quick");
nominal = twsbr_params();

invalid_run_indices = {-1, 1.5, NaN, Inf, 1 + 1i, [1, 2], "1"};
for index = 1:numel(invalid_run_indices)
    verifyError(test_case, @() generate_monte_carlo_scenario( ...
        invalid_run_indices{index}, 0, config, nominal), ...
        "twsbr:scenario:invalid_run_index");
end

invalid_base_seeds = {-1, 1.5, NaN, Inf, 1 + 1i, [1, 2], "1"};
for index = 1:numel(invalid_base_seeds)
    verifyError(test_case, @() generate_monte_carlo_scenario( ...
        0, invalid_base_seeds{index}, config, nominal), ...
        "twsbr:scenario:invalid_base_seed");
end
verifyError(test_case, @() generate_monte_carlo_scenario( ...
    1, 2^32 - 1, config, nominal), "twsbr:scenario:invalid_seed");

invalid_configs = {rmfield(config, "parameter_rho"), ...
    setfield(config, "parameter_rho", -0.1), ... %#ok<SFLD>
    setfield(config, "noise_std", [-1; 0; 0; 0]), ... %#ok<SFLD>
    setfield(config, "force_start_range", [5, 3]), ... %#ok<SFLD>
    setfield(config, "duration", 4.0)}; %#ok<SFLD>
for index = 1:numel(invalid_configs)
    verifyError(test_case, @() generate_monte_carlo_scenario( ...
        0, 0, invalid_configs{index}, nominal), ...
        "twsbr:scenario:invalid_config");
end

invalid_nominals = {rmfield(nominal, "body_mass"), ...
    setfield(nominal, "body_mass", -1), struct(), [1, 2]}; %#ok<SFLD>
for index = 1:numel(invalid_nominals)
    verifyError(test_case, @() generate_monte_carlo_scenario( ...
        0, 0, config, invalid_nominals{index}), ...
        "twsbr:scenario:invalid_nominal");
end
end

function test_monte_carlo_rows_share_conditions_by_run_index(test_case)
[vectors, config] = starter_vectors_for_test();
config.monte_carlo_runs = 2;

table_out = run_monte_carlo(vectors, twsbr_params(), config, ...
    monte_carlo_config("quick"), 20260);

verifyEqual(test_case, height(table_out), 10);
verifyEqual(test_case, table_out.controller, ...
    repmat(config.controller_names, 2, 1));
for run_index = 0:1
    rows = table_out(table_out.run_index == run_index, :);
    verifyEqual(test_case, height(rows), 5);
    verifyEqual(test_case, numel(unique(rows.delta_body_mass)), 1);
    verifyEqual(test_case, numel(unique(rows.force_amplitude)), 1);
    verifyEqual(test_case, numel(unique(rows.measurement_noise_seed)), 1);
end
failure_row = table_out.run_index == 1 & ...
    table_out.controller == "ATTITUDE_PID";
verifyEqual(test_case, sum(failure_row), 1);
verifyFalse(test_case, table_out.simulation_success(failure_row));
verifyFalse(test_case, table_out.success(failure_row));
verifyEqual(test_case, table_out.failure_reason(failure_row), ...
    "position_limit");
end

function verify_scenarios_equal(test_case, actual, expected)
verifyEqual(test_case, rmfield(actual, ...
    ["x_reference", "force_disturbance", "torque_disturbance"]), ...
    rmfield(expected, ...
    ["x_reference", "force_disturbance", "torque_disturbance"]));
probe_times = [0, 0.999, 1.0, 3.5, 5.5, actual.duration, ...
    actual.force_start, actual.force_start + actual.force_duration / 2];
verifyEqual(test_case, actual.x_reference(probe_times), ...
    expected.x_reference(probe_times));
verifyEqual(test_case, actual.force_disturbance(probe_times), ...
    expected.force_disturbance(probe_times));
verifyEqual(test_case, actual.torque_disturbance(probe_times), ...
    expected.torque_disturbance(probe_times));
end

function [vectors, config] = starter_vectors_for_test()
config = experiment_config("quick");
vectors = struct();
vectors.ATTITUDE_PID = log10([1.9,0.2,0.18]);
vectors.CASCADE_PID = log10([0.241,0.000396,0.193,9.255,1.011]);
vectors.FUZZY_PID = [log10([0.241,0.000396,0.193, ...
    9.255,0.05,1.011]),0.2,0.2,0.2];
vectors.LQR = log10([10,1,200,10,0.1]);
vectors.LQI = log10([10,1,200,10,100,0.1]);
end
