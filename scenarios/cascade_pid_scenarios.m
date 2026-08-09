function scenarios = cascade_pid_scenarios()
%CASCADE_PID_SCENARIOS Return the five frozen cascade PID scenarios.

zero_signal = @(time) zeros(size(time));
positive_step = @(time) 0.5 .* double(time >= 1.0);
negative_step = @(time) -0.5 .* double(time >= 1.0);
impulse = @(time) 5.0 .* double(time >= 4.0 & time < 4.2);
zero_state = zeros(4, 1);

scenarios = struct();
scenarios.zero_state = make_scenario( ...
    "zero_state", zero_state, 2.0, zero_signal, zero_signal, ...
    zero_signal, 0.0, 0.0);
scenarios.positive_position_step = make_scenario( ...
    "positive_position_step", zero_state, 8.0, positive_step, ...
    zero_signal, zero_signal, 1.0, 0.0);
scenarios.negative_position_step = make_scenario( ...
    "negative_position_step", zero_state, 8.0, negative_step, ...
    zero_signal, zero_signal, 1.0, 0.0);
scenarios.initial_tilt = make_scenario( ...
    "initial_tilt", [0.0; 0.0; deg2rad(5.0); 0.0], 5.0, ...
    zero_signal, zero_signal, zero_signal, 0.0, 0.0);
scenarios.force_impulse = make_scenario( ...
    "force_impulse", zero_state, 8.0, positive_step, impulse, ...
    zero_signal, 1.0, 4.2);
end

function scenario = make_scenario(name, initial_state, duration, ...
    x_reference, force_disturbance, torque_disturbance, ...
    reference_start, disturbance_end)
scenario = struct();
scenario.name = name;
scenario.initial_state = initial_state;
scenario.duration = duration;
scenario.x_reference = x_reference;
scenario.force_disturbance = force_disturbance;
scenario.torque_disturbance = torque_disturbance;
scenario.reference_start = reference_start;
scenario.disturbance_end = disturbance_end;
end
