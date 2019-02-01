function controller = make_pid_controller(Kp, Kd, Ki, car)
%MAKE_PID_CONTROLLER constructs a discrete-time PID controller with given 
%(continuous-time) parameters.
%
%The derivative of the error is approximated using the backward successive
%differences formula and the integral is approximated using the rectangle
%approximation rule.
%
%Syntax:
%controller = make_pid_controller(Kp, Kd, Ki, car)
%
%Input arguments:
% Kp   proportional gain 
% Kd   derivative gain
% Ki   integral gain
% car  structure with car parameters
%
%See also
% make_car, car_simulate, car_kinematics, car_move, car_runner


controller.Kp = Kp;
controller.Kd = Kd / car.Ts;
controller.Ki = Ki * car.Ts;