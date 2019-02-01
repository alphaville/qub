%CAR_RUNNER is a script that simulates the closed-loop behaviour of a
%PID-controlled car.
%
%The user specifies the initial state of the car (position and
%orientation), the simulation time, the PID parameters (Kp, Kd and Ki), the
%set-point as well as possible disturbances.
%
%Syntax:
% car_runner;
%
%See also
% car_kinematics, car_move, car_simulate, make_car, make_pid_controller, 
% plot_controlled_car

% Initial position and pose; state = (x_position, y_position, theta)
x_init = [0; 2.5; 0];

% System parameters (all in SI units)
car = make_car();

% Simulation time (in seconds)
Tsim = 10.0;

% PID controller parameters (Kp, Kd, Ki)
controller = make_pid_controller(0.7, 0.3, 0.0, car);

% Disturbances
car.u_bias = deg2rad(10);
car.x_bias = [0.0; 0.0; 0.0];

% Define set-point
set_point = 0.0;

% Perform simulations
[x_cache, u_cache] = car_simulate(car, controller, x_init, set_point, Tsim);

% Plot time profiles
plot_controlled_car(x_cache, u_cache, car)