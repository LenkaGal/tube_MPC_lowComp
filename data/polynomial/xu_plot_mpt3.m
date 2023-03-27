function xu_plot_mpt3(ctrl,aopt,XUset,plotSaturated)
% plot the approximated solution
if nargin < 3, error('Not enought inputs'); end
if nargin < 4, plotSaturated = 0; end

order = length(aopt)-1;
umax = ctrl.model.u.max;
umin = ctrl.model.u.min;

if ctrl.nx == 1
    x = ctrl.partition.Domain.grid(300);
else
      error('Currently onty 1D partitions can be plotted.');
end
y_sat = zeros(1, size(x, 1));
y = zeros(1, size(x, 1));
z = zeros(1, size(x, 1));
for j = 1:size(x, 1)
    y_sat(j) = xu_poly_eval(aopt, x(j, :)', order, umin, umax);
    y(j) = xu_poly_eval(aopt, x(j, :)', order);
    z(j) = ctrl.evaluate(x(j, :)');
end


figure
hold on
% XUset.plot('color','y','linestyle','--','linewidth', 2)
XUset.plot('color','y','linewidth', 2)
plot(x, z, 'b--', 'LineWidth', 3);
if plotSaturated
    plot(x, y, 'g', 'LineWidth', 3);
    plot(x, y_sat, 'r--', 'LineWidth', 3);
else
    plot(x, y, 'r--', 'LineWidth', 3);
end
axis([min(x)*1.1 max(x)*1.1 min(y_sat)*1.1 max(y_sat)*1.1])
if plotSaturated, s = '(saturated)'; else, s = ''; end
title(sprintf('Approximation order: %d %s', order, s));       
hold off

end