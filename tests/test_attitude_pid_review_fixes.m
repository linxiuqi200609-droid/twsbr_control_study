function tests = test_attitude_pid_review_fixes
%TEST_ATTITUDE_PID_REVIEW_FIXES Regression tests from final code review.
tests = functiontests(localfunctions);
end

function test_position_drift_is_not_matlab_failure(test_case)
plant_params = twsbr_params();
scenario = make_position_drift_scenario(plant_params, 0.02);
simulation = simulate_attitude_pid( ...
    plant_params, attitude_pid_params(), scenario);

verifyTrue(test_case, simulation.success);
verifyGreaterThan(test_case, ...
    simulation.metrics.max_abs_position, plant_params.x_limit);
end

function test_position_drift_is_not_simulink_failure(test_case)
plant_params = twsbr_params();
scenario = make_position_drift_scenario(plant_params, 0.02);
simulation = run_attitude_pid_simulink( ...
    plant_params, attitude_pid_params(), scenario);

verifyTrue(test_case, simulation.success);
verifyGreaterThan(test_case, ...
    simulation.metrics.max_abs_position, plant_params.x_limit);
end

function test_saturation_duration_counts_intervals_not_samples(test_case)
scenario = make_position_drift_scenario(twsbr_params(), 0.01);
scenario.name = "saturation_duration";
scenario.initial_state = [0.0; 0.0; deg2rad(20.0); 0.0];
pid_params = attitude_pid_params(struct("kp", 10.0));
simulation = simulate_attitude_pid( ...
    twsbr_params(), pid_params, scenario);

verifyTrue(test_case, all(simulation.saturated));
verifyEqual(test_case, simulation.metrics.saturation_duration, ...
    scenario.duration, "AbsTol", 1e-12);
end

function test_root_simulink_signals_have_english_names(test_case)
model_path = build_attitude_pid_simulink();
model_name = "twsbr_attitude_pid";
load_system(model_path);
cleanup = onCleanup(@() close_if_loaded(model_name));
line_handles = find_system(model_name, "FindAll", "on", ...
    "SearchDepth", 1, "Type", "line");

for index = 1:numel(line_handles)
    signal_name = string(get_param(line_handles(index), "Name"));
    verifyNotEmpty(test_case, regexp(signal_name, ...
        "^[A-Za-z][A-Za-z0-9_]*$", "once"));
end
end

function scenario = make_position_drift_scenario(plant_params, duration)
zero_signal = @(time) zeros(size(time));
scenario = struct();
scenario.name = "position_drift";
scenario.initial_state = [plant_params.x_limit + 1.0; 0.0; 0.0; 0.0];
scenario.duration = duration;
scenario.theta_reference = zero_signal;
scenario.force_disturbance = zero_signal;
scenario.torque_disturbance = zero_signal;
end

function close_if_loaded(model_name)
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
end
