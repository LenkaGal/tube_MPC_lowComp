yalmip('clear')
clear all
close all

%% MPC Class
% mpc_setup.explicit = true; % Explicit
mpc_setup.explicit = false; % Implicit MPC

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

%% Disturbance/additive uncertainty
% w_min = [-0.15; -0.15];
% w_max = [0.15; 0.15];
w_min = [-1];
w_max = [1];
W = Polyhedron('lb', w_min, 'ub', w_max);

figure(10)
subplot(2,2,3), hold on, box on, title('Additive disturbance')
W.plot

%% Initial state
% x0 = [3; 1];
% x0 = [-7; -2];
% x0 = [-10; -1.5];
x0 = 0-ys;

%% Physical constraints
u_min = 0-us;
u_max = 100-us;
% x_min = [-5; -5];
% x_max = [5; 5];
x_min = [0-ys];
x_max = [100-ys];

%% Delta-u constraints
% model.u.with('deltaMin');
% model.u.with('deltaMax');
% model.u.deltaMin = -7;
% model.u.deltaMax = 7;
du_min = -55
du_max =  55

%% Problem size
nx = size(A,1); % Number of states
nu = size(B,2); % Number of inputs

%% MPC data
Q = diag(10); % State penalty
% R = 0.1*eye(nu); % Input penalty
R = 1*eye(nu); % Input penalty
% R = 1*eye(nu); % Input penalty
% N = 10; % Prediction horizon
N = 30; % Prediction horizon

%% Constraints handling
Sx = Polyhedron('lb',x_min,'ub',x_max);
Su = Polyhedron('lb',u_min,'ub',u_max);
figure(10)
subplot(2,2,1), hold on, box on, title('State constraints')
Sx.plot
subplot(2,2,2), hold on, box on, title('Input constraints')
Su.plot

% Delta-U
Sdu = Polyhedron('lb',du_min,'ub',du_max);


%% LQR controller
[Klqr, Plqr] = dlqr(A, B, Q, R);
global K
K = -Klqr;

%% Robust positive invariant set
Z = approx_minimum_robust_invariant_set(A,B,K,W);
figure(10)
subplot(2,2,4), hold on, box on, title('RPI set')
Z.plot

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
model.x.penalty = QuadFunction(Q);
model.u.penalty = QuadFunction(R);
% Terminal set
Tset = model.LQRSet;
model.x.with('terminalSet');
model.x.terminalSet = Tset;
% Terminal penalty
P = model.LQRPenalty;
model.x.with('terminalPenalty');
model.x.terminalPenalty = P;

%% Optimization problem
u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
u_prev = sdpvar(nu,1);

%% State constraints robustification
Xc = Sx - Z;
figure(10)
subplot(2,2,1), hold on
Xc.plot('Color','b')

%% Input constraints robustification
UC = ( -K*Z );
Uc = Su - UC;
figure(10)
subplot(2,2,2), hold on
Uc.plot('Color','b')

%% Delta-U Input constraints robustification
UdC = -K*Z;
Udc = Sdu - UdC;
% Sdu = Polyhedron('lb',du_min,'ub',du_max);
figure(100), hold on, box on, title('Delta-U constraints')
Sdu.plot('Color','r')
Udc.plot('Color','b')

%% Terminal constraint
figure(10)
subplot(2,2,2), hold on, box on, title('Terminal set')
Tset.plot('Color','r')

%% Optimization
constraints = [];
objective = 0;
% Initial condition
X0 = sdpvar(nx,1);

%% Constraints for OPTIMIZER
constraints = [ x{1} == X0 ]; % to generate an OPTIMIZER object
constraints = [ constraints, x_min <= X0 <= x_max ]; 
constraints = [ Z.A*( X0 - x{1} ) <= Z.b ];
% constraints = [ -Z.A* x{1} <= ( Z.b - Z.A*X0) ];
constraints = [constraints, Sdu.A*(u{1}+ K*( X0 - x{1} ) - u_prev) <= Sdu.b ];

% Constraints for k = 0,...,N-1
for k = 1:N
    % Objective
    % objective = objective + norm(Q*x{k},2) + norm(R*u{k},2);
    objective = objective + x{k}'*Q*x{k} + u{k}'*R*u{k};
    % LTI model
    constraints = [ constraints, x{k+1} == A*x{k} + B*u{k} ];
    % Input and state constraints
    % constraints = [constraints, u_min <= u{k} <= u_max, x_min <= x{k} <= x_max];
    constraints = [constraints, Xc.A*x{k} <= Xc.b ];
    constraints = [constraints, Uc.A*u{k} <= Uc.b ];

    % Delta-U
    if(k >= 2)
        constraints = [constraints, Udc.A*(u{k} - u{k-1}) <= Udc.b ];
    end
end
% Terminal penalty
objective = objective + x{k+1}'*P.H*x{k+1};

% Terminal constraint
constraints = [constraints, Tset.A*x{N+1} <= Tset.b ];

% opt_settings = sdpsettings('verbose',0, 'solver', 'gurobi');
opt_settings = sdpsettings('verbose',0);
% sol = optimize([constraints, X0 == x0 ], objective, opt_settings)
sol = optimize([constraints], objective, opt_settings)


%% Optimizer
if( mpc_setup.explicit == true )
    %% Expliccit MPC
    disp(sprintf(' Explicit MPC design:'))
    [ mpsol, diagnostics, aux, Valuefunction, ExplicitOptimizer ] = solvemp( constraints, objective, opt_settings, [ X0 ], [u{1}; x{1}] )
    [ mpsol ] = get_explicit_partition( mpsol );
else
    %% Implicit MPC
    global ImplicitOptimizer
    disp(sprintf(' Implicit MPC design:'))
    ImplicitOptimizer = optimizer(constraints, objective, opt_settings, [X0; u_prev], [u{1}; x{1}]);
end

%% Closed loop simulation
Nsim = 15; % Number of simulation steps
X = x0;
Unominal = [];
Xnominal = [ x0 ];
Udata = [];
Xdata = [ X ];
Utube = 0-us;
for k = 1 : Nsim
    X = Xdata(:,k);
    if( mpc_setup.explicit == true )
        %% Expliccit MPC
        [ U_opt ] = ExplicitOptimizer{X};
    else
        %% Implicit MPC
        [ U_opt, error_code(k,1) ] = ImplicitOptimizer{[X; Utube]};
    end
  
    %% Extract variables
    U = U_opt(1:nu);
    X_opt = U_opt(nu+1 : nu+nx);
    % X = A*X + B*U;
    Utube = U + K*( X - X_opt );

    % Xdata(:,k+1) = A*X + B*( Utube ); % Nominal LTI system
    w = W.randomPoint; % Uncertainty
    Xdata(:,k+1) = A*X + B*( Utube ) + w; % Uncertain (noisy) LTI system

    Unominal = [Unominal, U];
    Xnominal = [Xnominal, X_opt];
    Udata = [Udata, Utube];

end

% Figures
figure(2), 
subplot(2,1,1), box on, hold on, xlabel('k'), ylabel('x')
stairs([0 , size(Xdata,2)-1], [0,0], 'k:')
stairs([0 : size(Xdata,2)-1], Xdata(1,:))
% stairs([0 : size(Xdata,2)-1], Xdata(2,:))
subplot(2,1,2), box on, hold on, xlabel('k'), ylabel('u')
stairs([0 : size(Udata,2)-1],Udata(1,:))
stairs([0 , size(Udata,2)-1], [ u_min, u_min], 'k--' )
stairs([0 , size(Udata,2)-1], [ u_max, u_max], 'k--' )

% figure(3), hold on, box on, title('State trajectory'), xlabel('x_1'), ylabel('x_2')
% Show Feasibility set
% mpsol{1}.Pfinal.plot('Color',[0.95, 0.95, 0.95]) % NOTE: Feasibility set for the conservative Tube MPC
% plot( Tset, 'Color',[0.9, 0.9, 0.9] )
% for k = 1 : size(Xnominal,2)
    % plot( Z + [ Xdata(:,k) ], 'Color',[0.7, 0.7, 0.7] )
%     plot( Z + [ Xnominal(:,k) ], 'Color',[0.7, 0.7, 0.7] )
% end
% plot(Xnominal(1,:),Xnominal(2,:),'kx:','LineWidth', 2, 'MarkerSize', 6)
% plot(Xdata(1,:),Xdata(2,:),'k*--','LineWidth', 2, 'MarkerSize', 6)
