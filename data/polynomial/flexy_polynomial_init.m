yalmip('clear')
clear all
close all

%% Model data
% Z = 0.6342;
Z = 3; % vieme ist so zosilnenim aj vyssie, napr. Z=10 kvoli sklonu PWA zak.riad. ale potom to crashne neskor pri tych XU 
T = 0.2926;

Ts = 0.01;
Ac = -1/T;
Bc = Z/T;
Cc = 1;
Dc = 0;
syss = ss(Ac,Bc,Cc,Dc);

syssd = c2d(syss,Ts);
A = syssd.A;
B = syssd.B;
C = syssd.C;
D = syssd.D;

us = [40];
ys = [68];

%% Initial state
x0 = 0-ys;

%% Physical constraints
u_min = 0-us;
u_max = 100-us;

x_min = [0-ys];
x_max = [100-ys];

%% Problem size
nx = size(A,1); % Number of states
nu = size(B,2); % Number of inputs

%% MPC data
Q = diag(10); % State penalty
R = 1*eye(nu); % Input penalty
N = 10; % Prediction horizon

%% MPC stability
% LTI system
model = LTISystem('A', A, 'B', B);
% Input constraints
model.u.min = u_min;
model.u.max = u_max;
% State constraints
model.x.min = x_min;
model.x.max = x_max;
% Penalty functions
model.x.penalty = OneNormFunction(Q);
model.u.penalty = OneNormFunction(R);
% % Terminal set
% Tset = model.LQRSet;
% model.x.with('terminalSet');
% model.x.terminalSet = Tset;
% % Terminal penalty
% P = model.LQRPenalty;
% model.x.with('terminalPenalty');
% model.x.terminalPenalty = P;
global empc
mpc = MPCController(model, N)
empc = mpc.toExplicit()


%% 
options = struct('dV_eps', 1e-6);
[ctrl_xu,XUset] = mpt_XUctrl_mpt3(empc, options);

% LTI system is assumed: i.e. no ULTI or hybrid dynamics
% MPT3 creates PWQ obj function oif terminal set is set
% extract the XU stabilizing sety
% plot the set
% close all
% XUset.XU.plot('color','y');
% hold on
% empc.partition.plot;

% plot the optimal control input
figure, hold on
% plotu(ctrl);
empc.feedback.fplot();
% axis([-4 4 -1.2 1.2])
% hold off

% degree of the approximation polynomial
aprx_degree = 3;
% degree of the polya polynomial
polya_degree = 1;

% compute the approximation
% aprx_ctrl = xu_approx_robust_mpt3(empc, XUset, aprx_degree, polya_degree);
global a
[~, ~, a] = xu_approx_robust_mpt3(empc, XUset, aprx_degree, polya_degree);
  
% plot the approximation
xu_plot_mpt3(empc,a,XUset.XU)
%% Disturbance on the input
dist = zeros(1501,1); 
dist(501:551) = 30;
dist(1001:1051) = -30;
dist = [[0:0.01:15]', dist]
%% calculate the control input
% x = 10;
% order = length(a)-1;
% 
% u = sum(a{end});
% for i = 1:order
%     u = u + a{i}*x.^(order+1-i);
% end
% 
% u = 0.5*(0.5*(u + u_max - abs(u - u_max)) + u_min + abs(0.5*(u + u_max - abs(u - u_max)) - u_min));
