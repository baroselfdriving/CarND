clear all;
clc;

points;

figure(1);
subplot(2,2,1);
plot(points1(:,1),points1(:,2), 'r--');
grid on;
hold on;
plot(points2(:,1),points2(:,2), 'b.');
title('position');
hold off;

subplot(2,2,2);
plot(points1(:,1),points1(:,3), 'r--');
grid on;
hold on;
plot(points2(:,1),points2(:,3), 'b.');
title('speed');
hold off;

subplot(2,2,3);
plot(points1(:,1),points1(:,4), 'r--');
grid on;
hold on;
plot(points2(:,1),points2(:,4), 'b.');
title('acceleration');
hold off;

subplot(2,2,4);
plot(points1(:,1),points1(:,5), 'r--');
grid on;
hold on;
plot(points2(:,1),points2(:,5), 'b.');
title('jerk');
hold off;
