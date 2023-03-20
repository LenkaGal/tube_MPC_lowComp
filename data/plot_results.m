% disturbance at time approx. 10s and 20s
load tube16
close all

fs = 12;
lw = 1.5;
red = [204 0 0]/255;
green = [50,205,50]/255;
blue = [[0 0.4470 0.7410]];
set(0,'defaulttextinterpreter','latex')

figure, hold on
plot(out.u(:,1),out.u(:,2),'Color', blue,'LineWidth', lw)
plot([0 25],[0 0],'r--','Color', red,'LineWidth', lw)
plot([0 25],[100 100],'r--','Color', red,'LineWidth', lw)
set(gca, 'TickLabelInterpreter','latex','FontSize', fs)
xlabel('time [s]')
ylabel('fan speed [\%]')
ylim([-10 110])
legend('$u(t)$','$u_\mathrm{min}, u_\mathrm{max}$', 'Interpreter','latex', 'FontSize', fs)

figure, hold on
plot(out.y(:,1),out.y(:,3),'Color', green,'LineWidth', lw)
plot(out.y(:,1),out.y(:,2),'Color', blue,'LineWidth', lw)

plot([0 25],[0 0],'--','Color', red, 'LineWidth', lw)
plot([0 25],[100 100],'--','Color', red,'LineWidth', lw)
set(gca, 'TickLabelInterpreter','latex','FontSize', fs)
xlabel('time [s]')
ylabel('flex sensor bend [\%]')
ylim([-10 110])
legend('$y^\mathrm{s}$', '$y(t)$', '$y_\mathrm{min}, y_\mathrm{max}$', 'Interpreter','latex', 'FontSize', fs)

%tube3 - small offset
%tube4 - nice but -68% not feasible
%tube5 - small offset
%tube6 - quite nice 
%tube7 - wrong delta-u setup 
%tube8 - wrong delta-u setup 
%tube9 - ugly beginning 
%tube10 - quite nice but small disturbance upwards
%tube11 - quite nice but no disturbance downwards
%tube12 - quite nice, but u_prev is initialized wrongly...all the previous data wrong
%-------------u_prev correct-------------
%tube13 - offset after disturbance, deltaU = [-60;60] 
%tube14 - quite nice, deltaU = [-60;60] 
%tube15 - quite nice, small offset, deltaU = [-60;60] 
%tube16 - quite nice, deltaU = [-55;55] 