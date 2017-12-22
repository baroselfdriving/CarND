clear all;
clc;

points;

figure(1);
subplot(2,2,1);
plot(points0(:,1),points0(:,2), 'r--');
grid on;
hold on;
plot(points1(:,1),points1(:,2), 'b.');
plot(points2(:,1),points2(:,2), 'g.');
plot(points3(:,1),points3(:,2), 'k.');
plot(points4(:,1),points4(:,2), 'r..');
plot(points5(:,1),points5(:,2), 'b..');
title('longitudinal position');
hold off;

subplot(2,2,2);
plot(points0(:,1),points0(:,3), 'r--');
grid on;
hold on;
plot(points1(:,1),points1(:,3), 'b.');
plot(points2(:,1),points2(:,3), 'g.');
plot(points3(:,1),points3(:,3), 'k.');
plot(points4(:,1),points4(:,3), 'r..');
plot(points5(:,1),points5(:,3), 'b..');
title('longitudinal speed');
hold off;

subplot(2,2,3);
plot(points0(:,1),points0(:,4), 'r--');
grid on;
hold on;
plot(points1(:,1),points1(:,4), 'b.');
plot(points2(:,1),points2(:,4), 'g.');
plot(points3(:,1),points3(:,4), 'k.');
plot(points4(:,1),points4(:,4), 'r..');
plot(points5(:,1),points5(:,4), 'b..');
title('longitudinal acceleration');
hold off;

subplot(2,2,4);
plot(points0(:,1),points0(:,5), 'r--');
grid on;
hold on;
plot(points1(:,1),points1(:,5), 'b.');
plot(points2(:,1),points2(:,5), 'g.');
plot(points3(:,1),points3(:,5), 'k.');
plot(points4(:,1),points4(:,5), 'r..');
plot(points5(:,1),points5(:,5), 'b..');
title('longitudinal jerk');
hold off;
%-----------------------------------------------------

figure(2);
subplot(2,2,1);
plot(points0(:,1),points0(:,6), 'r--');
grid on;
hold on;
plot(points1(:,1),points1(:,6), 'b.');
title('lateral position');
hold off;

subplot(2,2,2);
plot(points0(:,1),points0(:,7), 'r--');
grid on;
hold on;
plot(points1(:,1),points1(:,7), 'b.');
title('lateral speed');
hold off;

subplot(2,2,3);
plot(points0(:,1),points0(:,8), 'r--');
grid on;
hold on;
plot(points1(:,1),points1(:,8), 'b.');
title('lateral acceleration');
hold off;

subplot(2,2,4);
plot(points0(:,1),points0(:,9), 'r--');
grid on;
hold on;
plot(points1(:,1),points1(:,9), 'b.');
title('lateral jerk');
hold off;
