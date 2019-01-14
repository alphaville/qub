function df = car_kinematics(x, u, car)
%CAR_KINEMATICS defines the kinematic equations of a car, using a simple
%bicycle model
%
%Syntax:
%df = car_kinematics(x, u, car)
%
%Input arguments:
% x    a vector with the position and pose of the car, that is, 
%      x = (px, py, theta), where (px, py) is the (x,y)-position of the car
%      and theta is its heading angle
% u    steering angle
% car  a structure with the car parameters (use `make_car` to construct
%      such a structure)
%
%Output arguments:
% df   the gradient 
%
%The bicycle model is described by the following system of equations:
%
% p_x' = v * cos(θ),
% p_y' = v * sin(θ),
% θ' = (v/L) * tan(u),
%
%See also
%car_move, make_car

theta = x(3);

v = car.v;
L = car.L;

x_dot = v * cos(theta);
y_dot = v * sin(theta);
theta_dot = (v/L) * tan(u);

df = [x_dot; y_dot; theta_dot];