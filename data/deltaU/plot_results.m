% disturbance at time approx. 10s and 20s
load tube19
close all

fs = 12;
lw = 1.5;
red = [204 0 0]/255;
green = [50,205,50]/255;
blue = [[0 0.4470 0.7410]];
set(0,'defaulttextinterpreter','latex')

figure, hold on
plot(out.u(:,1),out.u(:,2),'Color', blue,'LineWidth', lw)
plot([0 15],[0 0],'r--','Color', red,'LineWidth', lw)
plot([0 15],[100 100],'r--','Color', red,'LineWidth', lw)
set(gca, 'TickLabelInterpreter','latex','FontSize', fs)
xlabel('time [s]')
ylabel('fan speed [\%]')
axis([0 15 -10 110])
legend('$v(t)$','$v_\mathrm{min}, v_\mathrm{max}$', 'Interpreter','latex', 'FontSize', fs)

figure, hold on
plot(out.y(:,1),out.y(:,3),'Color', green,'LineStyle', '--','LineWidth', lw)
plot(out.y(:,1),out.y(:,2),'Color', blue,'LineWidth', lw)
plot([0 15],[0 0],'--','Color', red, 'LineWidth', lw)
plot([0 15],[100 100],'--','Color', red,'LineWidth', lw)
set(gca, 'TickLabelInterpreter','latex','FontSize', fs)
xlabel('time [s]')
ylabel('flex sensor bend [\%]')
axis([0 15 -10 110])
legend('$b^\mathrm{s}$', '$b(t)$', '$b_\mathrm{min}, b_\mathrm{max}$', 'Interpreter','latex', 'FontSize', fs)
%% Cut data
close all
to1 = 251; from2 = 751;
to2 = 1251; from3 = 1751;
uplot = [out.u(1:to1,2);out.u(from2:to2,2);out.u(from3:end,2)]
yplot = [out.y(1:to1,2);out.y(from2:to2,2);out.y(from3:end,2)]
y_refplot = [out.y(1:to1,3);out.y(from2:to2,3);out.y(from3:end,3)]
tplot = out.u(1:length(uplot),1)

figure, hold on
plot(tplot,uplot,'Color', blue,'LineWidth', lw)
plot([0 15],[0 0],'r--','Color', red,'LineWidth', lw)
plot([0 15],[100 100],'r--','Color', red,'LineWidth', lw)
set(gca, 'TickLabelInterpreter','latex','FontSize', fs)
xlabel('time [s]')
ylabel('fan speed [\%]')
axis([0 15 -10 110])
legend('$v(t)$','$v_\mathrm{min}, v_\mathrm{max}$', 'Interpreter','latex', 'FontSize', fs)

figure, hold on
plot(tplot,y_refplot,'Color', green,'LineWidth', lw)
plot(tplot,yplot,'Color', blue,'LineWidth', lw)
plot([0 15],[0 0],'--','Color', red, 'LineWidth', lw)
plot([0 15],[100 100],'--','Color', red,'LineWidth', lw)
set(gca, 'TickLabelInterpreter','latex','FontSize', fs)
xlabel('time [s]')
ylabel('flex sensor bend [\%]')
axis([0 15 -10 110])
legend('$b^\mathrm{s}$', '$b(t)$', '$b_\mathrm{min}, b_\mathrm{max}$', 'Interpreter','latex', 'FontSize', fs)

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
%tube17 - generated disturbance on inputs
%tube18 - generated disturbance on inputs
%tube19 - generated disturbance on inputs, the prettiest