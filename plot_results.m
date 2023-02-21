% disturbance at time approx. 10s and 20s
load tube5
close all
figure
plot(out.u(:,1),out.u(:,2), 'LineWidth', 1)
figure, hold on
plot(out.y(:,1),out.y(:,2), 'LineWidth', 1)
plot(out.y(:,1),out.y(:,3), 'LineWidth', 1)
%tube3 - small offset
%tube4 - nice but -68% not feasible
%tube5 - small offset
%tube6 - quite nice 