function tests = test_simulate_cascade_pid
%TEST_SIMULATE_CASCADE_PID Tests for the nonlinear cascade PID simulation.
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

function test_zero_state_remains_an_exact_equilibrium(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct(), plant_params);
scenario = cascade_pid_scenarios().zero_state;
simulation = simulate_cascade_pid(plant_params, params, scenario);

verifyTrue(test_case, simulation.success);
verifyEqual(test_case, simulation.failure_reason, "");
verifyEqual(test_case, simulation.time, (0:0.001:2.0).', "AbsTol", 1e-15);
verifyEqual(test_case, simulation.state, zeros(2001, 4), "AbsTol", 1e-12);
verifyEqual(test_case, simulation.position_reference, zeros(2001, 1));
verifyEqual(test_case, simulation.position_error, zeros(2001, 1));
verifyEqual(test_case, simulation.theta_reference, zeros(2001, 1));
verifyEqual(test_case, simulation.theta_reference_raw, zeros(2001, 1));
verifyEqual(test_case, simulation.theta_error, zeros(2001, 1));
verifyEqual(test_case, simulation.u_raw, zeros(2001, 1));
verifyEqual(test_case, simulation.u, zeros(2001, 1));
verifyEqual(test_case, simulation.disturbance_force, zeros(2001, 1));
verifyEqual(test_case, simulation.position_integral, zeros(2001, 1));
verifyFalse(test_case, any(simulation.saturated));
verifyEqual(test_case, simulation.metrics.position_settling_time, 0.0);
verifyEqual(test_case, simulation.metrics.disturbance_recovery_time, 0.0);
verifyEqual(test_case, simulation.metrics.position_iae, 0.0, "AbsTol", 1e-15);
verifyEqual(test_case, simulation.metrics.tilt_iae, 0.0, "AbsTol", 1e-15);
end

function test_logs_have_aligned_dimensions_and_required_fields(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct(), plant_params);
scenario = make_scenario("dimensions", zeros(4, 1), 0.021, ...
    @(time) 0.4 .* ones(size(time)), @(time) zeros(size(time)), 0.0, 0.0);
simulation = simulate_cascade_pid(plant_params, params, scenario);
sample_count = 22;

verifySize(test_case, simulation.time, [sample_count, 1]);
verifySize(test_case, simulation.state, [sample_count, 4]);
vector_fields = {"position_reference", "position_error", ...
    "theta_reference", "theta_reference_raw", "theta_error", ...
    "u_raw", "u", "disturbance_force", "position_integral", "saturated"};
for index = 1:numel(vector_fields)
    verifySize(test_case, simulation.(vector_fields{index}), [sample_count, 1]);
end
verifyEqual(test_case, simulation.time(end), scenario.duration, "AbsTol", 1e-15);
verifyTrue(test_case, all(isfinite(simulation.state), "all"));
end

function test_controller_diagnostics_are_held_for_ten_plant_steps(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct(), plant_params);
scenario = make_scenario("multirate", zeros(4, 1), 0.021, ...
    @(time) 0.4 .* ones(size(time)), @(time) zeros(size(time)), 0.0, 0.0);
simulation = simulate_cascade_pid(plant_params, params, scenario);

held_fields = {"position_error", "theta_reference_raw", ...
    "theta_reference", "theta_error", "u_raw", "u", ...
    "position_integral", "saturated"};
for index = 1:numel(held_fields)
    values = simulation.(held_fields{index});
    verifyEqual(test_case, values(1:10), repmat(values(1), 10, 1));
    verifyEqual(test_case, values(11:20), repmat(values(11), 10, 1));
end
verifyGreaterThan(test_case, abs(simulation.u(1) - simulation.u(11)), 1e-10);
verifyEqual(test_case, simulation.position_integral(1:10), zeros(10, 1));
verifyEqual(test_case, simulation.position_integral(11:20), ...
    0.004 .* ones(10, 1), "AbsTol", 1e-12);
end

function test_repeated_runs_are_deterministic(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct(), plant_params);
scenario = cascade_pid_scenarios().initial_tilt;

first = simulate_cascade_pid(plant_params, params, scenario);
second = simulate_cascade_pid(plant_params, params, scenario);
verifyEqual(test_case, second, first);
end

function test_fixed_timing_and_duration_grid_are_enforced(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct(), plant_params);
bad_duration = make_scenario("bad_duration", zeros(4, 1), 0.0015, ...
    @(time) zeros(size(time)), @(time) zeros(size(time)), 0.0, 0.0);
verifyError(test_case, ...
    @() simulate_cascade_pid(plant_params, params, bad_duration), ...
    "twsbr:cascade_simulation:invalid_duration");

wrong_plant_step = cascade_pid_params(struct("plant_step", 0.002), plant_params);
verifyError(test_case, @() simulate_cascade_pid( ...
    plant_params, wrong_plant_step, cascade_pid_scenarios().zero_state), ...
    "twsbr:cascade_simulation:invalid_timing");
wrong_sample_time = cascade_pid_params( ...
    struct("sample_time", 0.02), plant_params);
verifyError(test_case, @() simulate_cascade_pid( ...
    plant_params, wrong_sample_time, cascade_pid_scenarios().zero_state), ...
    "twsbr:cascade_simulation:invalid_timing");
end

function test_controller_limits_are_respected_at_the_boundaries(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct(), plant_params);
scenario = make_scenario("limits", zeros(4, 1), 0.001, ...
    @(time) 100.0 .* ones(size(time)), @(time) zeros(size(time)), 0.0, 0.0);
simulation = simulate_cascade_pid(plant_params, params, scenario);

verifyTrue(test_case, simulation.success);
verifyEqual(test_case, abs(simulation.theta_reference(1)), ...
    deg2rad(12.0), "AbsTol", 1e-12);
verifyEqual(test_case, abs(simulation.u(1)), plant_params.u_max, "AbsTol", 1e-12);
verifyLessThanOrEqual(test_case, max(abs(simulation.theta_reference)), ...
    params.theta_reference_limit + 1e-12);
verifyLessThanOrEqual(test_case, max(abs(simulation.u)), plant_params.u_max + 1e-12);
end

function test_state_and_signal_failures_are_detected(test_case)
plant_params = twsbr_params();
params = cascade_pid_params(struct(), plant_params);

tilt_failure = make_scenario("tilt_failure", ...
    [0.0; 0.0; deg2rad(30.01); 0.0], 0.001, ...
    @(time) zeros(size(time)), @(time) zeros(size(time)), 0.0, 0.0);
tilt_result = simulate_cascade_pid(plant_params, params, tilt_failure);
verifyFalse(test_case, tilt_result.success);
verifyEqual(test_case, tilt_result.failure_reason, "tilt_limit");

position_failure = make_scenario("position_failure", ...
    [plant_params.x_limit + 0.001; 0.0; 0.0; 0.0], 0.001, ...
    @(time) zeros(size(time)), @(time) zeros(size(time)), 0.0, 0.0);
position_result = simulate_cascade_pid(plant_params, params, position_failure);
verifyFalse(test_case, position_result.success);
verifyEqual(test_case, position_result.failure_reason, "position_limit");

nonfinite_state = make_scenario("nonfinite_state", [NaN; 0.0; 0.0; 0.0], ...
    0.001, @(time) zeros(size(time)), @(time) zeros(size(time)), 0.0, 0.0);
state_result = simulate_cascade_pid(plant_params, params, nonfinite_state);
verifyFalse(test_case, state_result.success);
verifyEqual(test_case, state_result.failure_reason, "nonfinite_state");

nonfinite_reference = make_scenario("nonfinite_reference", zeros(4, 1), ...
    0.001, @(time) NaN(size(time)), @(time) zeros(size(time)), 0.0, 0.0);
reference_result = simulate_cascade_pid(plant_params, params, nonfinite_reference);
verifyFalse(test_case, reference_result.success);
verifyEqual(test_case, reference_result.failure_reason, "nonfinite_signal");
end

function test_reference_and_actuator_limit_failures_are_detected(test_case)
plant_params = twsbr_params();
reference_params = cascade_pid_params( ...
    struct("theta_reference_limit", deg2rad(13.0)), plant_params);
large_reference = make_scenario("reference_limit", zeros(4, 1), 0.001, ...
    @(time) 100.0 .* ones(size(time)), @(time) zeros(size(time)), 0.0, 0.0);
reference_result = simulate_cascade_pid( ...
    plant_params, reference_params, large_reference);
verifyFalse(test_case, reference_result.success);
verifyEqual(test_case, reference_result.failure_reason, "theta_reference_limit");

actuator_params = cascade_pid_params(struct("u_max", 2.0), plant_params);
large_command = make_scenario("actuator_limit", ...
    [0.0; 0.0; deg2rad(20.0); 0.0], 0.001, ...
    @(time) zeros(size(time)), @(time) zeros(size(time)), 0.0, 0.0);
actuator_result = simulate_cascade_pid( ...
    plant_params, actuator_params, large_command);
verifyFalse(test_case, actuator_result.success);
verifyEqual(test_case, actuator_result.failure_reason, "actuator_limit");
end

function test_iae_uses_trapezoids_and_saturation_counts_intervals(test_case)
plant_params = twsbr_params();
zero_gain_params = cascade_pid_params(struct( ...
    "kp_x", 0.0, "ki_x", 0.0, "kd_x", 0.0, ...
    "kp_theta", 0.0, "kd_theta", 0.0), plant_params);
constant_error = make_scenario("constant_error", zeros(4, 1), 0.003, ...
    @(time) 0.5 .* ones(size(time)), @(time) zeros(size(time)), 0.0, 0.0);
error_result = simulate_cascade_pid( ...
    plant_params, zero_gain_params, constant_error);
verifyEqual(test_case, error_result.metrics.position_iae, 0.0015, ...
    "AbsTol", 1e-15);
verifyEqual(test_case, error_result.metrics.tilt_iae, 0.0, "AbsTol", 1e-15);

saturated_params = cascade_pid_params(struct("u_max", 0.01), plant_params);
saturated_scenario = make_scenario("saturated", ...
    [0.0; 0.0; 0.1; 0.0], 0.003, ...
    @(time) zeros(size(time)), @(time) zeros(size(time)), 0.0, 0.0);
saturated_result = simulate_cascade_pid( ...
    plant_params, saturated_params, saturated_scenario);
verifyTrue(test_case, all(saturated_result.saturated));
verifyEqual(test_case, saturated_result.metrics.saturation_duration, ...
    0.003, "AbsTol", 1e-15);
end

function scenario = make_scenario(name, initial_state, duration, ...
    x_reference, force_disturbance, reference_start, disturbance_end)
scenario = struct();
scenario.name = name;
scenario.initial_state = initial_state;
scenario.duration = duration;
scenario.x_reference = x_reference;
scenario.force_disturbance = force_disturbance;
scenario.torque_disturbance = @(time) zeros(size(time));
scenario.reference_start = reference_start;
scenario.disturbance_end = disturbance_end;
end
