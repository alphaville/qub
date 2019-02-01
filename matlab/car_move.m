function x_next = car_move(x_init, u, car)
%CAR_MOVE computes the position and orientation of a vehicle at the end of
%the sampling period (at time Ts), given its initial position and
%orientation upon the action of a continuous steering angle.
%
%Syntax:
% x_next = car_move(x_init, u, car)
%
%Input arguments:
% x_init    three-dimensional vector with components x_init = (px_init,
%           py_init, theta_init), where (px_init, py_init) is the initial
%           position of the car on the (x, y)-plane and theta_init is the
%           initial orientation of the car (at time 0).
% u         (constant) steering angle; the angle is maintained constant
%           between sampling time instants.
% car       a structure with the parameters of the car (this is generated
%           using `make_car`). The sampling period, Ts, is specified in
%           this structure.
%
%Output arguments:
% x_next    three-dimensional vectors with components (px_next, py_next,
%           theta_next) with the position and orientation of the car at
%           time Ts.
%
%See also
% make_car, car_runner

% In our case, the exact discretisation is available in closed form:
L_over_tan_u = car.L/tan(u);
theta_plus = x_init(3) + (car.v/L_over_tan_u)*car.Ts;
x_plus = x_init(1) + L_over_tan_u * (sin(theta_plus) - sin(x_init(3)));
y_plus = x_init(2) - L_over_tan_u * (cos(theta_plus) - cos(x_init(3)));
x_next = [x_plus; y_plus; theta_plus];

% Numerical integration:
%  sysfun = @(t, z) car_kinematics(z, u, car);
%  [T, X] = ode45(sysfun, [0, car.Ts], x_init);
%  x_next = X(end, :)';

% NOTE: in general, we might not be able to derive the solution of a
% nonlinear dynamical system in closed form. We then need to resort to
% numerical methods. MATLAB comes with a number of numerical solvers for
% ordinary differential equations (ODEs). A standard choice is ODE45 - type
% `help ode45` for more information.