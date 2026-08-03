function tests = test_twsbr_simulation
%TEST_TWSBR_SIMULATION Tests for MATLAB open loop simulation.
tests = functiontests(localfunctions);
end

function test_zero_state_remains_at_equilibrium(test_case)
simulation = simulate_open_loop(twsbr_params(), zeros(4, 1), 0.5, false);

verifyLessThan(test_case, max(abs(simulation.state), [], "all"), 1e-10);
verifySize(test_case, simulation.state, [numel(simulation.time), 4]);
verifyEqual(test_case, simulation.input, zeros(size(simulation.time)), ...
    "AbsTol", 1e-12);
end

function test_three_degree_tilt_falls_without_control(test_case)
initial_tilt = deg2rad(3.0);
initial_state = [0.0; 0.0; initial_tilt; 0.0];
simulation = simulate_open_loop(twsbr_params(), initial_state, 2.0, false);

verifyGreaterThan(test_case, max(abs(simulation.state(:, 3))), ...
    deg2rad(20.0));
verifyEmpty(test_case, simulation.figure_handle);
end
