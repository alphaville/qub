% Controlling the robot with a PID-controller

% Initial position and pose; state = (x_position, y_position, theta)
x_init = [0; 2.5; 0];

% System parameters (all in SI units)
car = make_car();

% Simulation time
Tsim = 10.0;

% PID controller parameters
controller = make_pid_controller(0.35, 0.25, 0.1, car);

% Disturbances
car.u_bias = deg2rad(10);
car.x_bias = [0.0; 0.0; 0.0];

[x_cache, u_cache] = car_simulate(car, controller, x_init, Tsim);


% Plot time profiles
plot_controlled_car(x_cache, u_cache, car)