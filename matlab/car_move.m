function [x_next, X, T] = car_move(x_init, u, car)

sysfun = @(t, z) car_kinematics(z, u, car);
[T, X] = ode45(sysfun, [0, car.Ts], x_init);
x_next = X(end, :)';
