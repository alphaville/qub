function car = make_car()
%MAKE_CAR returns a structure that contains the constant parameters of a
%car. In particular, it returns a structure with the following fields:
%
% - L            length of the car in m
% - w            width of the car in m
% - wheel_size   diameter of the car's wheels in m
% - v            velocity of the car in m/s
% - Ts           sampling time in s
%
%See also
%car_kinematics, car_move, car_runner

car.L = 0.9;               % car length (m)                           90 cm
car.w = car.L/2;           % car width (m)                            45 cm
car.wheel_size = car.L/4;  % diameter of wheels (m)                 22.5 cm
car.v = 5;                 % velocity (constant) (m/s)                5 m/s
car.Ts = 1/40;             % sampling time (s)                        25 ms
