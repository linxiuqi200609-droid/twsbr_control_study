function tests = test_cascade_pid_acceptance
%TEST_CASCADE_PID_ACCEPTANCE Frozen cascade PID acceptance criteria.
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

function test_all_frozen_scenarios_meet_common_acceptance(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct(), plant_params);
scenarios = cascade_pid_scenarios();
scenario_names = fieldnames(scenarios);

for index = 1:numel(scenario_names)
    scenario_name = scenario_names{index};
    simulation = simulate_cascade_pid( ...
        plant_params, params, scenarios.(scenario_name));

    verifyTrue(test_case, simulation.success, scenario_name);
    verifyEqual(test_case, simulation.failure_reason, "", scenario_name);
    verify_public_numerics_are_finite(test_case, simulation, scenario_name);
    verifyLessThanOrEqual(test_case, ...
        max(abs(rad2deg(simulation.state(:, 3)))), 30.0, scenario_name);
    verifyLessThanOrEqual(test_case, ...
        max(abs(rad2deg(simulation.theta_reference))), ...
        12.0 + 1e-12, scenario_name);
    verifyLessThanOrEqual(test_case, max(abs(simulation.u)), ...
        plant_params.u_max + 1e-12, scenario_name);
    verifyLessThanOrEqual(test_case, max(abs(simulation.state(:, 1))), ...
        plant_params.x_limit, scenario_name);
end
end

function test_positive_position_step_meets_tracking_acceptance(test_case)
simulation = run_frozen_scenario("positive_position_step");

verifyLessThan(test_case, simulation.metrics.final_abs_position_error, 0.05);
verifyLessThanOrEqual(test_case, ...
    simulation.metrics.position_settling_time, 5.0);
verifyLessThan(test_case, simulation.metrics.final_abs_tilt_deg, 0.5);
end

function test_negative_position_step_meets_tracking_acceptance(test_case)
simulation = run_frozen_scenario("negative_position_step");

verifyLessThan(test_case, simulation.metrics.final_abs_position_error, 0.05);
verifyLessThanOrEqual(test_case, ...
    simulation.metrics.position_settling_time, 5.0);
verifyLessThan(test_case, simulation.metrics.final_abs_tilt_deg, 0.5);
end

function test_initial_tilt_returns_to_accepted_state(test_case)
simulation = run_frozen_scenario("initial_tilt");

verifyLessThan(test_case, simulation.metrics.final_abs_position_error, 0.05);
verifyLessThan(test_case, simulation.metrics.final_abs_tilt_deg, 0.5);
end

function test_force_impulse_recovers_within_three_seconds(test_case)
simulation = run_frozen_scenario("force_impulse");

verifyLessThanOrEqual(test_case, ...
    simulation.metrics.disturbance_recovery_time, 3.0);
end

function simulation = run_frozen_scenario(scenario_name)
plant_params = twsbr_params();
params = cascade_pid_params(struct(), plant_params);
scenarios = cascade_pid_scenarios();
simulation = simulate_cascade_pid( ...
    plant_params, params, scenarios.(scenario_name));
end

function verify_public_numerics_are_finite(test_case, simulation, diagnostic)
numeric_fields = {"time", "state", "position_reference", ...
    "position_error", "theta_reference", "theta_reference_raw", ...
    "theta_error", "u_raw", "u", "disturbance_force", ...
    "position_integral", "saturated"};
for index = 1:numel(numeric_fields)
    values = simulation.(numeric_fields{index});
    verifyTrue(test_case, all(isfinite(values), "all"), ...
        sprintf("%s: %s", diagnostic, numeric_fields{index}));
end

metric_names = fieldnames(simulation.metrics);
for index = 1:numel(metric_names)
    value = simulation.metrics.(metric_names{index});
    verifyTrue(test_case, isnumeric(value) && isscalar(value) && ...
        isfinite(value), sprintf("%s: metrics.%s", ...
        diagnostic, metric_names{index}));
end
end
