yalmip('clear')
clear all
close all

%% Model data
Z = 0.6342;
Z = 3;
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
% Terminal set
% Tset = Polyhedron('lb',-5,'ub',5)
% model.x.with('terminalSet');
% model.x.terminalSet = Tset;
% % Terminal penalty
% P = model.LQRPenalty;
% model.x.with('terminalPenalty');
% model.x.terminalPenalty = P;
global empc
mpc = MPCController(model, N)
empc = mpc.toExplicit()

figure, empc.feedback.fplot

