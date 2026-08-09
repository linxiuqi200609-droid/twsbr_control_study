function tests = test_attitude_pid_simulation
%TEST_ATTITUDE_PID_SIMULATION Tests for RK4 and closed loop simulation.
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

function test_rk4_preserves_zero_equilibrium(test_case)
next_state = twsbr_rk4_step(zeros(4, 1), 0.0, 0.001, ...
    twsbr_params(), 0.0, 0.0);

verifyEqual(test_case, next_state, zeros(4, 1), "AbsTol", 1e-12);
end

function test_rk4_agrees_with_ode45_for_short_step(test_case)
plant_params = twsbr_params();
state = [0.0; 0.1; deg2rad(2.0); -0.05];
u = 0.2;
step_size = 0.001;
rk4_state = twsbr_rk4_step(state, u, step_size, plant_params, 0.0, 0.0);
derivative = @(time, value) twsbr_dynamics( ...
    time, value, u, plant_params, 0.0, 0.0);
options = odeset("RelTol", 1e-12, "AbsTol", 1e-13);
[~, ode_state] = ode45(derivative, [0.0, step_size], state, options);

verifyEqual(test_case, rk4_state, ode_state(end, :).', "AbsTol", 1e-10);
end

function test_zero_state_remains_at_equilibrium(test_case)
simulation = simulate_attitude_pid( ...
    twsbr_params(), attitude_pid_params(), make_scenario(0.0, 1.0));

verifyTrue(test_case, simulation.success);
verifyLessThan(test_case, max(abs(simulation.state), [], "all"), 1e-10);
verifyEqual(test_case, simulation.metrics.settling_time, 0.0, "AbsTol", 1e-12);
end

function test_positive_tilt_stabilizes_within_two_seconds(test_case)
simulation = simulate_attitude_pid( ...
    twsbr_params(), attitude_pid_params(), make_scenario(5.0, 5.0));

verifyTrue(test_case, simulation.success);
verifyLessThanOrEqual(test_case, simulation.metrics.settling_time, 2.0);
verifyLessThan(test_case, simulation.metrics.final_abs_tilt_deg, 0.5);
end

function test_negative_tilt_stabilizes_within_two_seconds(test_case)
simulation = simulate_attitude_pid( ...
    twsbr_params(), attitude_pid_params(), make_scenario(-5.0, 5.0));

verifyTrue(test_case, simulation.success);
verifyLessThanOrEqual(test_case, simulation.metrics.settling_time, 2.0);
verifyLessThan(test_case, simulation.metrics.final_abs_tilt_deg, 0.5);
end

function test_logs_align_and_respect_limits(test_case)
pid_params = attitude_pid_params();
simulation = simulate_attitude_pid( ...
    twsbr_params(), pid_params, make_scenario(5.0, 1.0));
sample_count = numel(simulation.time);

verifySize(test_case, simulation.state, [sample_count, 4]);
verifySize(test_case, simulation.u, [sample_count, 1]);
verifySize(test_case, simulation.u_raw, [sample_count, 1]);
verifySize(test_case, simulation.integral_error, [sample_count, 1]);
verifyLessThanOrEqual(test_case, max(abs(simulation.u)), pid_params.u_max + 1e-12);
verifyLessThanOrEqual(test_case, max(abs(simulation.integral_error)), ...
    pid_params.integral_limit + 1e-12);
verifyTrue(test_case, all(isfinite(simulation.state), "all"));
end

function scenario = make_scenario(initial_tilt_deg, duration)
scenario = struct();
scenario.name = "test_scenario";
scenario.initial_state = [0.0; 0.0; deg2rad(initial_tilt_deg); 0.0];
scenario.duration = duration;
scenario.theta_reference = @(time) 0.0 * time;
scenario.force_disturbance = @(time) 0.0 * time;
scenario.torque_disturbance = @(time) 0.0 * time;
end
