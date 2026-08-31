function tests = test_controller_design_isolation
%TEST_CONTROLLER_DESIGN_ISOLATION Nominal feedback with perturbed physics.
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

function test_lqr_and_lqi_design_stays_nominal_while_physics_changes(test_case)
nominal = twsbr_params();
physical = nominal;
physical.body_mass = 1.2 * nominal.body_mass;
physical.com_length = 0.85 * nominal.com_length;
physical.body_inertia = 1.1 * nominal.body_inertia;
config = experiment_config("quick");
scenarios = training_scenarios(3.2);
scenario = scenarios.T1_initial_tilt_5deg;
scenario.duration = 0.04;
vectors = {log10([10,1,200,10,0.1]), log10([10,1,200,10,100,0.1])};
names = ["LQR", "LQI"];
for index = 1:2
    name = names(index);
    vector = vectors{index};
    expected = decode_controller_vector(name, vector, nominal, config);
    base = simulate_control_system(name, vector, nominal, config, scenario, 0);
    perturbed = simulate_control_system(name, vector, physical, config, ...
        scenario, 0, nominal);
    legacy = simulate_control_system(name, vector, physical, config, scenario, 0);
    verifyEqual(test_case, base.controller_parameters, expected);
    verifyEqual(test_case, perturbed.controller_parameters, expected);
    verifyEqual(test_case, legacy.controller_parameters, ...
        decode_controller_vector(name, vector, physical, config));
    verifyNotEqual(test_case, legacy.controller_parameters, expected);
    verifyNotEqual(test_case, perturbed.state(2,:), base.state(2,:));
    verifyEqual(test_case, perturbed.u_raw(1), base.u_raw(1));
    if name == "LQR"
        verifyEqual(test_case, perturbed.controller_parameters.gain, expected.gain);
    else
        verifyEqual(test_case, perturbed.controller_parameters.state_gain, expected.state_gain);
        verifyEqual(test_case, perturbed.controller_parameters.integral_gain, expected.integral_gain);
    end
end
end

function test_monte_carlo_uses_nominal_design_not_perturbed_redesign(test_case)
plant = twsbr_params();
config = experiment_config("quick");
config.controller_names = ["LQR"; "LQI"];
config.monte_carlo_runs = 1;
vectors = struct("LQR", log10([10,1,200,10,0.1]), ...
    "LQI", log10([10,1,200,10,100,0.1]));
uncertainty = monte_carlo_config("quick");
actual = run_monte_carlo(vectors, plant, config, uncertainty, 2026);
[scenario, physical, metadata] = generate_monte_carlo_scenario(0, 2026, uncertainty, plant);
verifyNotEqual(test_case, physical.body_mass, plant.body_mass);
for index = 1:2
    name = config.controller_names(index);
    nominal = simulate_control_system(name, vectors.(name), physical, ...
        config, scenario, metadata.measurement_noise_seed, plant);
    redesigned = simulate_control_system(name, vectors.(name), physical, ...
        config, scenario, metadata.measurement_noise_seed);
    expected = calculate_control_metrics(nominal, scenario, physical);
    verifyEqual(test_case, actual.theta_itae(index), expected.theta_itae);
    verifyNotEqual(test_case, nominal.state, redesigned.state);
end
end
