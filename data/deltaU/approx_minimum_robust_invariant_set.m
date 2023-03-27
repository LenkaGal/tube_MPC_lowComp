function F_alpha = approx_minimum_robust_invariant_set(A,B,K,W,tol)
%% Invariant Approximations of the Minimal Robust Positively Invariant Set
% by S. V. Rakovic, E. C. Kerrigan, K. I. Kouramas, D. Q. Mayne
% IEEE TRANSACTIONS ON AUTOMATIC CONTROL, VOL. 50, NO. 3, MARCH 2005
% URL: https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=1406138

if (nargin <= 4) 
    tol = 1e-4; % Default tolerance 
end % if 
if ( tol <= 0 )
	tol = 1e-4; % Default tolerance 
end % if 

% Closed-loop/autonomous system matrix 
Acl = A + B*K;

% Problem size
nx = size(Acl, 1); 

% Auxiliary variable
I = eye(nx);

% Initial values
s = 0; % "s" is integer going toward zero: s -> 0
alpha = 1e6; % Initialize "alpha"
Ms = 1000;
MaxIter = 1000;
mss = zeros(2*nx,1); % Initialize evaluation of Eq(13)

while( alpha > tol/( tol + Ms ) ) & ( s < MaxIter )
    % Increment "s"
    s = s + 1
    
    % Eq(10): (Acl^s)*W \subseteq alpha*W == h_W( (Acl^(s))'*f_i ) <= alpha*g_i
    % Eq(11): alpha(s) = max_{i \in I} ( ( h_W( (Acl^(s))'*f_i ) ) / g_i )
    alpha = max(W.support(Acl^(s)*(W.A)')./W.b);
    
    % Eq(13): M(s) = max_{j=1,...,n} ( sum_{i=0}^{s-1} ( h_W( (Acl^(i))'*e_j ) ), sum_{i=0}^{s-1} ( h_W( -(Acl^(i))'*e_j ) ) )
    mss = mss + W.support([Acl^(s), -Acl^(s)]);
    Ms = max(mss);
end

% Eq(2): Fs = Minkowski_Sum_{i=0}^{s-1}( Acl^(i)*W ), where F_0 = {0}
Fs = W; % for s = 0

for i = 1 : (s - 1)
    [i, s]
    Fs = Fs + Acl^(i)*W;
    Fs.minVRep();
end
F_alpha = (1 - alpha)^(-1)*Fs;

end % function