clear all;
clc;

points;

sz = size(points);
nSwitches = 1;
for i = 2:sz(1)
  if points(i,1) < points(i-1,1)
    nSwitches = nSwitches + 1;
  end 
end

figure(1);
subplot(2,2,1);
plot(points(:,1),points(:,2), 'r--');
grid on;
hold on;
title('longitudinal position');
hold off;

subplot(2,2,2);
plot(points(:,1),points(:,3), 'r--');
grid on;
hold on;
title('longitudinal speed');
hold off;

subplot(2,2,3);
plot(points(:,1),points(:,4), 'r--');
grid on;
hold on;
title('longitudinal acceleration');
hold off;

subplot(2,2,4);
plot(points(:,1),points(:,5), 'r--');
grid on;
hold on;
title('longitudinal jerk');
hold off;
%-----------------------------------------------------

figure(2);
subplot(2,2,1);
plot(points(:,1),points(:,6), 'r--');
grid on;
hold on;
title('lateral position');
hold off;

subplot(2,2,2);
plot(points(:,1),points(:,7), 'r--');
grid on;
hold on;
title('lateral speed');
hold off;

subplot(2,2,3);
plot(points(:,1),points(:,8), 'r--');
grid on;
hold on;
title('lateral acceleration');
hold off;

subplot(2,2,4);
plot(points(:,1),points(:,9), 'r--');
grid on;
hold on;
title('lateral jerk');
hold off;

%-----------------------------------------------------
figure(3);
plot(points(:,10),points(:,11), 'r--');
grid on;
hold on;
title('cartesian position');
hold off;
