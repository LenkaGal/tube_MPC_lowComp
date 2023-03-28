%% TUBE MPC DESIGN
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

% LTI system
model = ULTISystem('A', A, 'B', B, 'E', 1);
model.u.min = 0-us;
model.u.max = 100-us;
model.x.min = 0-ys; 
model.x.max = 100-ys;
model.d.min = -1; 
model.d.max = 1;
% Penalty functions
model.x.penalty = QuadFunction(diag([10]));
model.u.penalty = QuadFunction(diag([1]));
% Prediction horizon
N = 30;

model.u.with('deltaMin');
model.u.with('deltaMax');
model.u.deltaMin = -55;
model.u.deltaMax = 55;

option = {'soltype',1,'LQRstability',1};
% option = {'soltype',0,'LQRstability',1};

iMPC = TMPCController(model,N,option);
% Explicit TMPC controller construction
eMPC = iMPC.toExplicit;

% TMPC evaluation
x0 = 0-ys; x0 = x0(1:eMPC.nx);
[ u, feasible ] = iMPC.evaluate(x0,'u.previous', 0) % Feasibility check
[ u, feasible ] = eMPC.evaluate(x0,'u.previous', 0) % Feasibility check