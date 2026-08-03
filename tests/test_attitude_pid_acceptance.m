function tests = test_attitude_pid_acceptance
%TEST_ATTITUDE_PID_ACCEPTANCE Acceptance scenarios and result plots.
tests = functiontests(localfunctions);
end

function test_scenario_definitions_are_frozen(test_case)
scenarios = attitude_pid_scenarios();

verifyEqual(test_case, string(fieldnames(scenarios)), ...
    ["zero_state"; "positive_tilt"; "negative_tilt"; ...
    "large_tilt"; "torque_impulse"]);
verifyEqual(test_case, scenarios.zero_state.duration, 2.0);
verifyEqual(test_case, scenarios.positive_tilt.duration, 5.0);
verifyEqual(test_case, scenarios.negative_tilt.duration, 5.0);
verifyEqual(test_case, scenarios.large_tilt.duration, 5.0);
verifyEqual(test_case, scenarios.torque_impulse.duration, 5.0);
verifyEqual(test_case, rad2deg(scenarios.positive_tilt.initial_state(3)), ...
    5.0, "AbsTol", 1e-12);
verifyEqual(test_case, rad2deg(scenarios.negative_tilt.initial_state(3)), ...
    -5.0, "AbsTol", 1e-12);
verifyEqual(test_case, rad2deg(scenarios.large_tilt.initial_state(3)), ...
    8.0, "AbsTol", 1e-12);
verifyEqual(test_case, scenarios.torque_impulse.torque_disturbance(0.999), 0.0);
verifyEqual(test_case, scenarios.torque_impulse.torque_disturbance(1.000), 0.05);
verifyEqual(test_case, scenarios.torque_impulse.torque_disturbance(1.050), 0.05);
verifyEqual(test_case, scenarios.torque_impulse.torque_disturbance(1.051), 0.0);

scenario_names = fieldnames(scenarios);
for index = 1:numel(scenario_names)
    scenario = scenarios.(scenario_names{index});
    verifyEqual(test_case, scenario.theta_reference(0.5), 0.0);
    verifyEqual(test_case, scenario.force_disturbance(0.5), 0.0);
end
end

function test_all_matlab_scenarios_meet_acceptance(test_case)
plant_params = twsbr_params();
pid_params = attitude_pid_params(struct(), plant_params);
scenarios = attitude_pid_scenarios();
scenario_names = fieldnames(scenarios);

for index = 1:numel(scenario_names)
    scenario_name = scenario_names{index};
    simulation = simulate_attitude_pid( ...
        plant_params, pid_params, scenarios.(scenario_name));
    verifyTrue(test_case, simulation.success, scenario_name);
    verifyLessThanOrEqual(test_case, max(abs(simulation.u)), ...
        pid_params.u_max + 1e-12, scenario_name);
    verifyLessThanOrEqual(test_case, max(abs(simulation.integral_error)), ...
        pid_params.integral_limit + 1e-12, scenario_name);
    verifyTrue(test_case, all(isfinite(simulation.state), "all"), scenario_name);

    switch scenario_name
        case "zero_state"
            verifyLessThan(test_case, max(abs(simulation.state), [], "all"), ...
                1e-10);
        case {"positive_tilt", "negative_tilt"}
            verifyLessThanOrEqual(test_case, ...
                simulation.metrics.settling_time, 2.0, scenario_name);
            verifyLessThan(test_case, ...
                simulation.metrics.final_abs_tilt_deg, 0.5, scenario_name);
        case "large_tilt"
            verifyLessThan(test_case, ...
                simulation.metrics.final_abs_tilt_deg, 0.5, scenario_name);
        case "torque_impulse"
            verifyLessThanOrEqual(test_case, ...
                simulation.metrics.settling_time - 1.05, 2.0, scenario_name);
    end
end
end

function test_result_figure_has_four_panels(test_case)
scenarios = attitude_pid_scenarios();
simulation = simulate_attitude_pid(twsbr_params(), ...
    attitude_pid_params(), scenarios.positive_tilt);
figure_handle = plot_attitude_pid_results(simulation);
cleanup = onCleanup(@() close(figure_handle));

verifyEqual(test_case, string(figure_handle.Visible), "off");
axes_handles = findall(figure_handle, "Type", "axes");
verifyNumElements(test_case, axes_handles, 4);
end
