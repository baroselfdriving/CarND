clear all;
clc;

points;

figure(1);
subplot(2,2,1);
plot(points(:,1),points(:,2), 'r');
grid on;
hold on;
title('longitudinal position');
hold off;

subplot(2,2,2);
plot(points(:,1),points(:,3), 'r');
grid on;
hold on;
title('longitudinal speed');
hold off;

subplot(2,2,3);
plot(points(:,1),points(:,4), 'r');
grid on;
hold on;
title('x evolution');
hold off;

subplot(2,2,4);
plot(points(:,1),points(:,5), 'r');
grid on;
hold on;
title('y evolution');
hold off;
%-----------------------------------------------------
