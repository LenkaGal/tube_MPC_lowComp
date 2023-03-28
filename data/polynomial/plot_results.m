
load poly3
yp = out.y(:,2);
up = out.u(:,2);

load opti7
close all
yo = out.y(:,2);
uo = out.u(:,2);

fs = 12;
lw = 1.5;
red = [204 0 0]/255;
green = [50,205,50]/255;
blue = [[0 0.4470 0.7410]];
yellow = [0.9290 0.6940 0.1250];
set(0,'defaulttextinterpreter','latex')

figure, hold on
plot(out.u(:,1),uo,'Color', yellow,'LineWidth', lw)
plot(out.u(:,1),up,'Color', blue,'LineWidth', lw)
plot([0 15],[0 0],'r--','Color', red,'LineWidth', lw)
plot([0 15],[100 100],'r--','Color', red,'LineWidth', lw)
set(gca, 'TickLabelInterpreter','latex','FontSize', fs)
xlabel('time [s]')
ylabel('fan speed [\%]')
axis([0 15 -10 110])
legend('$v_\mathrm{o}(t)$', '$v_\mathrm{o}(t)$', '$v_\mathrm{min}, v_\mathrm{max}$', 'Interpreter','latex', 'FontSize', fs)

figure, hold on
plot(out.y(:,1),out.y(:,3),'Color', green, 'LineStyle', '--', 'LineWidth', lw)
plot(out.y(:,1),yo,'Color', yellow,'LineWidth', lw)
plot(out.y(:,1),yp,'Color', blue, 'LineWidth', lw)
plot([0 15],[0 0],'--','Color', red, 'LineWidth', lw)
plot([0 15],[100 100],'--','Color', red,'LineWidth', lw)
set(gca, 'TickLabelInterpreter','latex','FontSize', fs)
xlabel('time [s]')
ylabel('flex sensor bend [\%]')
axis([0 15 -10 110])
legend('$b^\mathrm{s}$', '$b_\mathrm{o}(t)$', '$b_\mathrm{p}(t)$', '$b_\mathrm{min}, b_\mathrm{max}$', 'Interpreter','latex', 'FontSize', fs)

%% Optimal 1-norm controller
%opti1 - no disturbance (25sec. measurement)
%opti2 - second dist. delayed (25sec. measurement)
%opti3 - both dist. delayed 
%opti4 - first dis. small
%opti5 - ok, little bit oscillating
%opti6 - most pretty, both dist. delayed 
%opti7 - generated dist. on input
%% Polynomial approximation
%poly1 - no disturbance
%poly2 - "our" disturbance on output
%poly3 - generated dist. on input
