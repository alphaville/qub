function plot_controlled_car(x_cache, u_cache, car)
%PLOT_CONTROLLED_CAR plots 

Nsim = length(u_cache);
x_max = 1.05 * max(x_cache(1,:));
y_max = 1.05 * max(abs(x_cache(2,:)));
cmap = colormap('lines');
color = cmap(1, :);

subplot(4,4,1:12)
plot([-car.L x_max], [0 0], 'Color', [0.7 0.7 0.7]); hold on; % plot line
plot(x_cache(1,:), x_cache(2, :), 'Color', color, 'linewidth', 2);
axis ([-car.L x_max -y_max y_max]);
hold on;

subplot(4,4,[13 14])
plot(car.Ts*(1:Nsim), rad2deg(u_cache(1,1:Nsim)),'linewidth', 2);
u_max = rad2deg(max(abs(u_cache(1,:))));
axis ([0 Nsim*car.Ts -1.1*u_max 1.1*u_max]); ylabel('steering (deg)');
xlabel('time (s)')
grid;
hold on;

subplot(4,4,[15 16])
plot(car.Ts*(1:Nsim), x_cache(2,1:Nsim), 'linewidth', 2);
y_max = max(abs(x_cache(2,:)));
axis ([0 Nsim*car.Ts -1.1*y_max 1.1*y_max]); ylabel('y (m)');
xlabel('time (s)'); grid;
hold on;