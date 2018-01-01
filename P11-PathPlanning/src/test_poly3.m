clear all;
clc;

points;

figure(1);
subplot(1,3,1);
plot(points(:,1),points(:,2), 'r');
xlabel('t');
ylabel('m/s')
grid on;
hold on;
title('longitudinal speed');
hold off;

subplot(1,3,2);
plot(points(:,1),points(:,3), 'r');
xlabel('t');
ylabel('m/s^2')
grid on;
hold on;
title('longitudinal acceleration');
hold off;

subplot(1,3,3);
plot(points(:,1),points(:,4), 'r');
xlabel('t');
ylabel('m/s^3')
grid on;
hold on;
title('longitudinal jerk');
hold off;
%-----------------------------------------------------
