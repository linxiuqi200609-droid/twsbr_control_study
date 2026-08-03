# Two Wheeled Self Balancing Robot Plant

This project provides matching MATLAB and Simulink models of the longitudinal dynamics of a two wheeled self balancing robot. The plant is an equivalent nonlinear cart pole model intended for later PID and cascade PID controller development.

## State and inputs

The state order is:

```text
[x; x_dot; theta; theta_dot]
```

- `x`: wheel axle position in meters.
- `x_dot`: wheel axle velocity in meters per second.
- `theta`: body tilt from the upright equilibrium in radians.
- `theta_dot`: body angular velocity in radians per second.
- `u`: normalized motor command converted to horizontal force by `motor_force_gain`.
- `force_disturbance`: external horizontal force in newtons.
- `torque_disturbance`: external body torque in newton meters.

The upright equilibrium is `theta = 0`. The plant does not clip `u`; actuator saturation belongs outside the plant so all future controllers share the same limit.

## Main files

- `twsbr_params.m`: default physical parameters and validation.
- `twsbr_dynamics.m`: nonlinear continuous state equation.
- `twsbr_linear_model.m`: analytical upright linearization.
- `twsbr_numerical_linearize.m`: central difference Jacobian verification.
- `simulate_open_loop.m`: MATLAB open loop simulation and response plot.
- `build_twsbr_simulink.m`: reproducible Simulink model builder.
- `run_simulink_open_loop.m`: Simulink simulation interface with `N by 4` state output.
- `run_project.m`: one command build, test, simulation, and export workflow.
- `twsbr_plant.slx`: generated nonlinear Simulink plant.
- `tests`: MATLAB unit and integration tests.

## Quick start

Open MATLAB and run:

```matlab
cd("D:/Research/srtp")
run_project
```

To generate results without running the test suite first:

```matlab
summary = run_project(false);
```

## Generated outputs

The workflow creates:

```text
results/open_loop_results.mat
results/open_loop_response.png
twsbr_plant.slx
```

The MAT file contains default parameters, analytical state space matrices, the controllability matrix and rank, a zero state equilibrium simulation, a 3 degree MATLAB open loop simulation, and the matching Simulink simulation.

## Model verification

The automated tests verify:

- zero state and zero input form an equilibrium;
- positive tilt produces falling angular acceleration;
- positive motor input produces corrective negative angular acceleration;
- analytical and numerical linearization agree;
- the linearized plant has controllability rank four;
- an uncontrolled 3 degree initial tilt diverges from upright;
- MATLAB and Simulink state trajectories agree;
- the Simulink model can be rebuilt without warnings;
- the expected result files are generated.

## Future controller connection

A future attitude PID uses `theta` and `theta_dot`. A cascade controller can use `x_dot` or `x` in its outer loop and produce a tilt reference for the attitude inner loop. Both controllers should produce an unsaturated command `u_raw`; one shared actuator block should apply the limit from `params.u_max` before the signal enters `twsbr_plant`.
