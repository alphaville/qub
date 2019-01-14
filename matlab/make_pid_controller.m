function controller = make_pid_controller(Kp, Kd, Ki, car)
%MAKE_PID_CONTROLLER constructs a PID controller with given parameters
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
% make_car, car_simulate, car_kinematics, car_move


controller.Kp = Kp;
controller.Kd = Kd / car.Ts;
controller.Ki = Ki * car.Ts;