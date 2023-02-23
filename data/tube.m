% clc, clear all;
% D = 0.0864;
K = 0.6342;
T = 0.2926;

Ts = 0.01;
A = -1/T;
B = K/T;
C = 1;
D = 0;
syss = ss(A,B,C,D);

syssd = c2d(syss,Ts);
Ad = syssd.A;
Bd = syssd.B;
Cd = syssd.C;
Dd = syssd.D;

ny = size(C,1);
nx = size(A,1);
nu = size(B,2);

us = [40];
ys = [68];
% x0 = [ys-ys; 0];

model = ULTISystem('A', Ad , 'B', Bd, 'E', [1]);
model.x.min = [0-ys];
model.x.max = [100-ys];
model.u.min = 0-us;
model.u.max = 100-us;
model.d.min = [-1]; 
model.d.max = [ 1];

R = 1*eye(nu);
model.u.penalty = QuadFunction(R);

Q = diag(20);
model.x.penalty = QuadFunction(Q);

% Terminal set
model.x.with('terminalSet');
model.x.terminalSet = model.LQRSet;

% Terminal penalty
model.x.with('terminalPenalty');
model.x.terminalPenalty = QuadFunction(model.LQRPenalty);

% Prediction horizon
N = 25;

% Enable construction of the explicit controller
option = {'eMPC',1};

% TMPC controller construction
global iMPC
[iMPC,eMPC,sol] = TMPCController(model,N,option)
%%
% TMPC evaluation
% x0 = -10; % Initial condition
% u_implicit = iMPC.optimizer(x0) % Implicit MPC evaluation
% u_explicit = eMPC.evaluate(x0)  % Explicit MPC evaluation
% [u, feasible, openloop] = eMPC.evaluate(x0) % Enriched output

% loop = ClosedLoop(eMPC,model)
% Nsim = 100
% data = loop.simulate(x0, Nsim)

