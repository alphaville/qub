function [X, U] = car_simulate(car, controller, x_init, set_point, Tsim)
%CAR_SIMULATE performs a simulation of the PID-controlled car
%
%Syntax:
% [X, U] = car_simulate(car, controller, x_init, Tsim)
%
%Input arguments:
% car           a structure carrying the car parameters (create one using 
%               `make_car`). The user may provide two additional (optional) 
%               fields in `car`:
%                 - u_bias: it is possible that the control command (the
%                   steering angle) is affected by a disturbance and while
%                   the controller commands an angle `u`, the actual
%                   steering angle is `u + u_bias`. Recall that all
%                   variables are assumed to be in SI units, so `u_bias` is
%                   in radians (not degrees)
%                 - x_bias: it is likely that the model we use (the bicycle
%                   model; see `car_kinematics`) is not accurate. 
%               Overall, the system dynamics in discrete time is described
%               by:
%                  x = car_move(x, u + u_bias, car) + x_bias
%               where `u_bias` and `x_bias` are the (constant) disturbances 
%               described above.
% controller    a PID controller structure (create one using
%               `make_pid_controller`)
% x_init        initial state of the car; a three-dimensional vector x_init
%               = (px_init, py_init, theta_init), where (px_init, py_init)
%               is the initial position of the car on the (x,y) plane and
%               theta_init is the car's initial heading
% Tsim          simulation time in seconds
%
%Output arguments:
% X  is a 3-by-(Nsim+1) matrix; its `k`-th column is a 3-dimensional vector
%    with the system state at time `(k-1)*Ts`, where `Ts` is the sampling 
%    time (see `car.Ts`)
% U  is a 5-by-Nsim matrix; U(1, k) is the control action at time
%    `(k-1)*Ts`, U(2, k) stores the action of the P mode at time
%    `(k-1)*Ts`, U(3, k) and U(4, k) store the actions of the D and I modes
%    respectively and U(5, k) stores the sum of errors up to the discrete
%    time instant `k-1`.
%
%Note: Use plot_controlled_car to generate plots of the car's controlled
%trajectories.
%
%See also:
% make_car, make_pid_controller, plot_controlled_car, car_runner

if isfield(car, 'u_bias'), u_bias = car.u_bias; else, u_bias = 0.; end
if isfield(car, 'x_bias'), x_bias = car.x_bias; else, x_bias = 0.; end

Nsim = ceil(Tsim/car.Ts);                        % num. simulation points

% initialise variables
U = zeros(5, Nsim-1); % [u, up, ud, ui]
x = x_init; X = zeros(3, Nsim); X(:, 1) = x;
measurement = x(2); sum_error = 0;

% Control loop
previous_error = set_point - x(2);
for k = 1:Nsim
    ctrl_error = set_point - measurement;        % error
    delta_error = ctrl_error - previous_error;   % error difference
    previous_error = ctrl_error;                 % store previous error
    up = controller.Kp * ctrl_error;             % P
    ui = controller.Ki * sum_error;              % I
    ud = controller.Kd * delta_error;            % D
    
    u = up + ud + ui;                            % control action (P+I+D)
    
    x = car_move(x, u + u_bias, car) + x_bias;   % apply contr. to system
    measurement = x(2);                          % obtain measurement
    
    X(:, k+1) = x;                               % store variables
    U(:, k) = [u; up; ud; ui; sum_error];        %  for plotting
    
    sum_error = sum_error + ctrl_error;          % update the sum of errors
end