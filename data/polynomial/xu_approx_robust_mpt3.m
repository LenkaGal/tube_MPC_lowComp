function [ctrl, runtime, aopt] = xu_approx_robust_mpt3(ctrl, XUset, order, polya_degree, strip_bounds)

if nargin < 4
    strip_bounds = false;
end
usecost = true;

yalmip('clear');

% umax = ctrl.sysStruct.umax;
% umin = ctrl.sysStruct.umin;
% Pn = ctrl.Pn;
% if strip_bounds
%     XU = xu_strip_u_bounds(ctrl.details.XU, ctrl.details.dims.nx);
% else
%     XU = ctrl.details.XU;
% end
XU = XUset.XU;


%% find polynomial u(x)=a'*x s.t. [x; u] \in XU(i) \forall x \in P(i), \forall i

nx = ctrl.nx;
a = sdpvar(repmat(1, 1, order+1), repmat(nx, 1, order+1));
a{end} = 0;

E = {};
maxp = 0;
for i = 1:ctrl.nr
    E{i} = ctrl.partition.Set(i).V;
end

objrob = 0;
Frob = set([]);

fprintf('robustifying constraints...\n');
tic
for i = 1:ctrl.nr
    obj = 0;
    F = [];
    w = sdpvar(size(E{i}, 1), 1);
    F = F + [uncertain(w)];
    F = F + [sum(w)==1; w >= 0];
    x = 0;
    for j = 1:size(E{i}, 1)
        x = x + E{i}(j, :)'*w(j);
    end
    u = xu_poly_eval(a, x, order);
    F = F + [ismember([x; u], XU(i))];
%     x0 = E{i};
    x0 = grid(ctrl.partition.Set(i), 10);
% x0 = extreme(XU(i));
% x0 = unique(x0(:, 1:nx), 'rows');
    for ii = 1:size(x0, 1)
        u_opt = ctrl.evaluate(x0(ii, :)');
%         if strip_bounds
%             %u_aprx = xu_poly_eval(a, x0(i, :)', order, umin, umax);
%             u_aprx = xu_poly_eval(a, x0(i, :)', order);
%         else
            u_aprx = xu_poly_eval(a, x0(ii, :)', order);
%         end
        obj = obj + norm(u_opt - u_aprx, 1);
    end
    [FF, OO] = robustify(F, obj, sdpsettings('robust.polya', polya_degree));
    Frob = Frob + FF;
    objrob = objrob + OO;
end
if isa(objrob, 'double') || ~usecost
    objrob = [];
end
runtime.robustify = toc
fprintf('\nsolving the problem...\n');
tic
sol = solvesdp(Frob, objrob, sdpsettings('robust.polya', polya_degree))
runtime.lp = toc
runtime.sol = sol;

% extract optimal values of the parameters of the approximation polynomial
aopt = cell(1, length(a));
for i = 1:length(a),
    aopt{i} = double(a{i});
end

% % store the approximation into the controller object
% ctrl = struct(ctrl);
% % ctrl.details = rmfield(ctrl.details, 'XU');
% % ctrl.details = rmfield(ctrl.details, 'XUproj');
% % ctrl.details = rmfield(ctrl.details, 'XUtime');
% ctrl.details.u_aprox = aopt;
% ctrl.details.u_aprox_saturated = strip_bounds;
% ctrl = mptctrl(ctrl);




end
