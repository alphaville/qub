function [X, U] = car_simulate(car, controller, x_init, Tsim)
%CAR_SIMULATE performs a simulation of the PID-controlled car
%
%Syntax:
% [X, U] = car_simulate(car, controller, x_init, Tsim)
%
%Input arguments:
% car           a structure carrying the car parameters (create one using 
%               `make_car`)
% controller    a PID controller structure (create one using
%               `make_pid_controller`)
% x_init        initial state of the car; a three-dimensional vector x_init
%               = (px_init, py_init, theta_init), where (px_init, py_init)
%               is the initial position of the car on the (x,y) plane and
%               theta_init is the car's initial heading
% Tsim          simulation time in seconds
%
%Output arguments:
% X
% U
%
%See also:
% make_car, make_pid_controller

if isfield(car, 'u_bias'), u_bias = car.u_bias; else, u_bias = 0.; end
if isfield(car, 'x_bias'), x_bias = car.x_bias; else, x_bias = 0.; end

Nsim = ceil(Tsim/car.Ts);                        % num. simulation points

% initialise variables
set_point = 0.0;
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