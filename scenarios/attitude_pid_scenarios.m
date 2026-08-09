function scenarios = attitude_pid_scenarios()
%ATTITUDE_PID_SCENARIOS Return frozen upright control scenarios.

zero_signal = @(time) zeros(size(time));
zero_state = zeros(4, 1);

scenarios = struct();
scenarios.zero_state = make_scenario( ...
    "zero_state", zero_state, 2.0, zero_signal, zero_signal, zero_signal);
scenarios.positive_tilt = make_scenario( ...
    "positive_tilt", [0.0; 0.0; deg2rad(5.0); 0.0], ...
    5.0, zero_signal, zero_signal, zero_signal);
scenarios.negative_tilt = make_scenario( ...
    "negative_tilt", [0.0; 0.0; deg2rad(-5.0); 0.0], ...
    5.0, zero_signal, zero_signal, zero_signal);
scenarios.large_tilt = make_scenario( ...
    "large_tilt", [0.0; 0.0; deg2rad(8.0); 0.0], ...
    5.0, zero_signal, zero_signal, zero_signal);
torque_impulse = @(time) 0.05 .* double(time >= 1.00 & time <= 1.05);
scenarios.torque_impulse = make_scenario( ...
    "torque_impulse", zero_state, 5.0, ...
    zero_signal, zero_signal, torque_impulse);
end

function scenario = make_scenario(name, initial_state, duration, ...
    theta_reference, force_disturbance, torque_disturbance)
scenario = struct();
scenario.name = name;
scenario.initial_state = initial_state;
scenario.duration = duration;
scenario.theta_reference = theta_reference;
scenario.force_disturbance = force_disturbance;
scenario.torque_disturbance = torque_disturbance;
end
