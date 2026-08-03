# Two Wheeled Self Balancing Robot Plant and Attitude PID

This project provides matching MATLAB and Simulink models of the longitudinal dynamics of a two wheeled self balancing robot. It includes a discrete basic attitude PID that keeps the body upright while leaving wheel position and velocity uncontrolled. The plant is an equivalent nonlinear cart pole model intended for later cascade PID controller development.

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

The upright equilibrium is `theta = 0`. The plant does not clip `u`; actuator saturation belongs outside the plant so all controllers share the same limit.

## Main files

- `twsbr_params.m`: default physical parameters and validation.
- `twsbr_dynamics.m`: nonlinear continuous state equation.
- `twsbr_linear_model.m`: analytical upright linearization.
- `twsbr_numerical_linearize.m`: central difference Jacobian verification.
- `simulate_open_loop.m`: MATLAB open loop simulation and response plot.
- `build_twsbr_simulink.m`: reproducible Simulink plant model builder.
- `run_simulink_open_loop.m`: Simulink simulation interface with `N by 4` state output.
- `run_project.m`: one command plant build, test, simulation, and export workflow.
- `twsbr_plant.slx`: generated nonlinear Simulink plant.
- `attitude_pid_params.m`: validated discrete attitude PID parameters.
- `attitude_pid_step.m`: one deterministic PID update with conditional anti windup.
- `simulate_attitude_pid.m`: fixed step nonlinear MATLAB closed loop simulation.
- `build_attitude_pid_simulink.m`: reproducible closed loop Simulink builder.
- `run_attitude_pid_simulink.m`: Simulink closed loop simulation interface.
- `run_attitude_pid.m`: one command attitude PID verification and export workflow.
- `twsbr_attitude_pid.slx`: generated basic attitude PID Simulink model.
- `tests`: MATLAB unit and integration tests.

## Quick start

Open MATLAB and run the plant workflow:

```matlab
cd("D:/Research/srtp")
run_project
```

To generate plant results without running the test suite first:

```matlab
summary = run_project(false);
```

To build and verify the basic attitude PID, run:

```matlab
cd("D:/Research/srtp")
summary = run_attitude_pid;
```

For a faster PID result export without rerunning all tests:

```matlab
summary = run_attitude_pid(false);
```

## Basic attitude PID

The controller uses only body tilt and angular velocity:

```text
theta_error = theta - theta_reference
u_raw = kp * theta_error + ki * integral_error + kd * theta_dot
u = clip(u_raw, -u_max, u_max)
```

The default upright reference is zero. The discrete controller runs every `0.01 s`; the MATLAB nonlinear plant uses a `0.001 s` RK4 step. The default gains are `kp = 1.90`, `ki = 0.20`, and `kd = 0.18`. Conditional integration prevents the integral state from increasing actuator saturation, and both the actuator and integral state have symmetric limits.

The frozen acceptance scenarios are zero state, positive 5 degree tilt, negative 5 degree tilt, positive 8 degree tilt, and a `0.05 N m` torque impulse. MATLAB and Simulink use the same gains, sign convention, sample time, actuator limit, plant parameters, and scenarios.

## Generated outputs

The workflows create:

```text
results/open_loop_results.mat
results/open_loop_response.png
twsbr_plant.slx
results/attitude_pid_results.mat
results/attitude_pid_response.png
twsbr_attitude_pid.slx
```

The open loop MAT file contains default parameters, analytical state space matrices, the controllability matrix and rank, equilibrium and 3 degree simulations, and the matching Simulink simulation. The attitude PID MAT file contains plant and PID parameters, all MATLAB scenario trajectories and metrics, the positive tilt Simulink trajectory, acceptance decisions, and the MATLAB versus Simulink comparison.

## Model verification

The automated tests verify:

- zero state and zero input form an equilibrium;
- positive tilt produces falling angular acceleration;
- positive motor input produces corrective negative angular acceleration;
- analytical and numerical linearization agree;
- the linearized plant has controllability rank four;
- an uncontrolled 3 degree initial tilt diverges from upright;
- MATLAB and Simulink open loop state trajectories agree;
- the plant model can be rebuilt without warnings;
- PID proportional, integral, derivative, saturation, and anti windup behavior;
- all five basic attitude PID acceptance scenarios;
- applied control and integral state limits;
- warning free Simulink attitude PID reconstruction and English block names;
- MATLAB and Simulink closed loop tilt agreement within 0.2 degree;
- generation of the expected MAT, PNG, and SLX artifacts.

## Current limitation and future controller connection

The basic attitude PID deliberately uses only `theta` and `theta_dot`. It keeps the body upright but does not regulate wheel position or wheel velocity, so position drift is expected and is recorded rather than treated as a failure. A future cascade controller can use `x_dot` or `x` in its outer loop and produce a tilt reference for this attitude inner loop. Both controllers produce an unsaturated command `u_raw`; one shared actuator block applies the limit from `params.u_max` before the signal enters the plant.
