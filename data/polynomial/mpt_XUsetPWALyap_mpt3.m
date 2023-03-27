function XUset = mpt_XUsetPWALyap_mpt3(ctrl,Options,LyapFcn);
%MPT_XUSETPWALYAP computes the stabilizing control set on the basis of a given PWA Lyapunov function
%
% XUset = mpt_XUsetPWALyap(ctrl,Options);
% XUset = mpt_XUsetPWALyap(ctrl,Options,LyapFcn); [NOT IMPLEMENTED YET!!]
%
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% computes the stabilizing controller set U(x) on the basis of a given PWA
% Lyapunov function
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% ctrl                    - Explicit controller (MPTCTRL object)
%
% LyapFcn                 - PWA Lyapunov Function (if not provided: the value
%                           function in ctrl is assumed to be Lyapunov for the
%                           controlled system)
%                           NOTE: The LyapFcn needs to be defined over the
%                                 same polyhedral partition as ctrl.Pn!!
%                                 (e.g. MPT_LYAPUNOV creates such a LyapFcn)
%  LyapFcn.Pn             - Polyhedral partition of the state-space
%  LyapFcn.Bi, LyapFcn.Ci - cell arrays containing the PWA Lyapunov function
%
% Options.gamma           - enforced Lyapunov decay rate (default gamma = 1e-5)
%                           for regions around the origin:
%                            V(x(k+1)) - V(x(k)) <= gamma * ||x(k)||_p
% Options.norm_p          - norm p (= 1 or inf) used in the enforced Lyapunov
%                           decay rate (default p = inf)
% Options.dV_eps          - enforced Lyapunov decay rate (default dV_eps = 1e-6)
%                           for regions not around the origin:
%                            V(x(k+1)) - V(x(k)) <= -dV_eps
% Options.merge           - =0 (default) no intermediate merging of the XUset
%                           =1 simple intermediate greedy merging
%                           =2 intermediate optimal merging
% Options.convex          - =0 don't try to find a convex representation
%                              of polytope arrays in intermediate steps
%                           =1 (default) find a convex representation of
%                              for polytope arrays in intermediate steps
% Options.psolvers        - which projection method to use
%                           (default is [3 1 5 2 4 0], see help
%                           polytope/projection for more details)
%
%
% ---------------------------------------------------------------------------
% OUTPUT
% ---------------------------------------------------------------------------
% XUset                   - Polyhedral partition of the state+input-space,
%                           where [x' U(x)']' \in XUset
%
%
% see also MPT_LYAPUNOV, MPT_OPTINFCONTROL

% Copyright is with the following author(s):
%
% (C) 2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch
% (C) 2005 Frank J. Christophersen, Automatic Control Laboratory, ETH Zurich,
%          fjc@control.ee.ethz.ch

% ---------------------------------------------------------------------------
% Legal note:
%          This program is free software; you can redistribute it and/or
%          modify it under the terms of the GNU General Public
%          License as published by the Free Software Foundation; either
%          version 2.1 of the License, or (at your option) any later version.
%
%          This program is distributed in the hope that it will be useful,
%          but WITHOUT ANY WARRANTY; without even the implied warranty of
%          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%          General Public License for more details.
%
%          You should have received a copy of the GNU General Public
%          License along with this library; if not, write to the
%          Free Software Foundation, Inc.,
%          59 Temple Place, Suite 330,
%          Boston, MA  02111-1307  USA
%
% ---------------------------------------------------------------------------


error(nargchk(1,3,nargin));

global mptOptions
if ~isstruct(mptOptions),
    mpt_error;
end

% define an empty Polyhedron
% Pe = Polyhedron([eye(1);-eye(1)],[-1;-1]);
Pe = [];


% if isa(ctrl, 'mptctrl')
%     if ~isexplicit(ctrl),
%         error('Only explicit controllers supported by this function.');
%     end
%     %ctrl = struct(ctrl);
% elseif ~mpt_isValidCS(ctrl)
%     error('Input must be an MPTCTRL object or a valid Control Structure.');
% end
if ~strcmp(ctrl.getName,'Explicit MPC controller')
    error('Only explicit controllers are supported by this function.');
end

if nargin<=1,
    Options = [];
end

if nargin==3
    warning(['make sure that the lyapunov function is defined over',...
        ' the same polyhedral partion as the controller partition',...
        ' (with the same ordering of the polytopes)!'])
end
if nargin<=3  %use value funciton as LyapFunction
%     LyapFcn.Pn = ctrl.Pn;
%     LyapFcn.Bi = ctrl.Bi;
%     LyapFcn.Ci = ctrl.Ci;
    LyapFcn = ctrl.cost;
end

if ~isfield(Options, 'gamma')
    Options.gamma = 1e-5;
end
if ~isfield(Options, 'norm_p')
    Options.norm_p = inf;
end
if ~isfield(Options, 'dV_eps')
    Options.dV_eps = 1e-6;
end
if ~isfield(Options, 'merge')
    Options.merge = 0;
end
if ~isfield(Options, 'convex')
    Options.convex = 1;
end
if ~isfield(Options, 'verbose'),
    Options.verbose = mptOptions.verbose;
end
if ~isfield(Options, 'psolvers'),
    Options.psolvers = [3 1 5 2 4 0];
end

% % sysStruct  = ctrl.details.origSysStruct;
% % probStruct = ctrl.details.origProbStruct;
% sysStruct = ctrl.sysStruct;
% probStruct = ctrl.probStruct;
% [nx, nu, ny, ndyn, nbool] = mpt_sysStructInfo(sysStruct);

nx = ctrl.nx;
nu = ctrl.nu;
% ny = ctrl.ny;
% TODO: 
% Forcing ndyn = 1 (i.e. only LTI systems are assumed / no hybrid dynamics)
% forcing nbool = 0 (Try to determine from the mpt3 contorller if it contains any bool variables)
ndyn = 1;
nbool = 0;


if nbool > 0
    error('Systems with boolean inputs not supported.');
end

if ctrl.model.x.penalty.type == 2
    error('2-norm solutions not supported.');
end

% constraints on U                  [!!! TODO: needs to be modified for the X-U constraints case]
Hu = [eye(nu); -eye(nu)];
Ku = [ctrl.model.u.max; -ctrl.model.u.min];


% % read out Lyapunov function
% Pn = LyapFcn.Pn;
% Bi = LyapFcn.Bi;
% Ci = LyapFcn.Ci;

% % System data read out
% if ndyn < 2  % LTI system
%     A = ctrl.model.A;
%     B = ctrl.model.B;
%     if iscell(A)
%         A = A{1};
%         B = B{1};
%     end
%     if iscell(ctrl.model.f)
%         f = ctrl.model{1};
%     else
%         f = ctrl.model.f;
%     end
% else         % PWA system
% %     A = sysStruct.A;
% %     B = sysStruct.B;
% %     f = sysStruct.f;
% %     % error('PWA systems not yet supported.');  % [!!! TODO: NOT IMPLEMETED YET]
% end
if ndyn < 2
    A = ctrl.model.A;
    B = ctrl.model.B;
    f = ctrl.model.f;
else
    error('Sorry, PWA systems are not supported')
end



% detect flat regions, because
%  if Pn(ii) is lowdim => XUset is lowdim  (not considered in the soution)
%  if Pn(jj) is lowdim => XUset is lowdim  (not considered in the soution)
% !!! TODO: include if U-set is also flat => same reasoning!!!
ind_flat = [];
for ii=1:ctrl.nr
    xout = ctrl.partition.Set(ii).chebyCenter;
    Rn = xout.r;
    if Rn<mptOptions.rel_tol
        [R,l,u]=bounding_box(ctrl.partition.Set(ii),struct('noPolyOutput',1));
        d = abs(u-l);

        if any(d>mptOptions.rel_tol)
            ind_flat = [ind_flat ii];
        end
    end
end

if ~isempty(ind_flat),
    fprintf('%d flat regions will be skipped.\n', length(ind_flat));
end

% emptypoly = mptOptions.emptypoly;
XUset     = Pe;

[isin, zero_inside, closest] = contains(ctrl.partition,zeros(nx,1));
zero_inside = sort(zero_inside);


% extreme values of J(x) in Pi
Jmaxmin = [];
for ii=1:ctrl.nr
%     [H,K] = double(ctrl.partition.Set(ii));
    H = LyapFcn.Set(ii).A;
    K = LyapFcn.Set(ii).b;
    Bi = LyapFcn.Set(ii).getFunction('obj').F;
    Ci = LyapFcn.Set(ii).getFunction('obj').g;

    [xmin,Jmin,lambda,exitflag,how]=mpt_solveLPs(Bi,H,K,[],[],[],...
        mptOptions.lpsolver);
    [xmax,Jmax,lambda,exitflag,how]=mpt_solveLPs(-Bi,H,K,[],[],[],...
        mptOptions.lpsolver);

    Jmaxmin = [Jmaxmin; [-Jmax Jmin]+Ci];
end

isConvexOpt.abs_tol = mptOptions.rel_tol;
if isfield(Options, 'usehull'),
    isConvexOpt.usehull = Options.usehull;
end

% main loops
if ndyn<2 % LTI system

    for ii = 1:ctrl.nr  % source region

        if ismember(ii,ind_flat) % skip "flat" regions
            continue
        end

%         [H1, K1] = double(Pn(ii));
%         b1 = Bi{ii};
%         c1 = Ci{ii};
        H1 = LyapFcn.Set(ii).A;
        K1 = LyapFcn.Set(ii).b;
        b1 = LyapFcn.Set(ii).getFunction('obj').F;
        c1 = LyapFcn.Set(ii).getFunction('obj').g;

%         XUsetIter = emptypoly;
        XUsetIter = Pe;

        jj_ind = find(Jmaxmin(ii,1)>Jmaxmin(:,2));

        if 1  % NEW VERSION: with detecting convex XU sets

            if ismember(ii,zero_inside)
                % Case 1: (0 \in Pi) & (Jj_min < Ji_max)
                %         => XUset might be non-convex
                
%                 XUsetIter_case1 = emptypoly;
                XUsetIter_case1 = Pe;

                if Options.verbose > 0
                    if Options.gamma > 0
                        disp(sprintf('region: i=%d /%d  [region at 0: additional projection computations]',...
                            ii,ctrl.nr))
                    else
                        disp(sprintf('region: i=%d /%d  [region at 0]',ii,ctrl.nr))
                    end
                end
                
                for jj = jj_ind'

                    if ismember(jj,ind_flat) % skip "flat" regions
                        continue
                    end

                    H2 = LyapFcn.Set(jj).A;
                    K2 = LyapFcn.Set(jj).b;
                    b2 = LyapFcn.Set(jj).getFunction('obj').F;
                    c2 = LyapFcn.Set(jj).getFunction('obj').g;

                    if Options.gamma > 0
                        P = XUpoly_zero(Hu, Ku, H1, K1, H2, K2, b1, c1, b2, c2, A, B, f,...
                            nx, nu, Options.gamma, Options.norm_p, Options.psolvers);
                    else % don't use projection
                        P = XUpoly(Hu, Ku, H1, K1, H2, K2, b1, c1, b2, c2, A, B, f,...
                            nx, nu, 0, inf, 0);
                    end

                    XUsetIter_case1 = [XUsetIter_case1 P];
                end%jj
                
                % if set is convex, use it (does not need to be convex!)
                if Options.convex
                    U = PolyUnion(XUsetIter_case1);
                    if isConvex_mod(U,isConvexOpt.abs_tol)
                        XUsetIter_case1 = U.convexHull; 
                    end
%                     U = PolyUnion(XUsetIter_case1);
%                     if U.isConvex(), XUsetIter_case1 = U.convexHull; end

%                     [stat,env] = isconvex(XUsetIter_case1,isConvexOpt);
%                     if stat
%                         XUsetIter_case1 = env;
%                     end
                end
                
                % TO DELETE
                if length(XUsetIter_case1) > 1
                    plot(XUsetIter_case1)
%                     isConvex_mod(obj,convxtol)
                end

                XUsetIter = [XUsetIter XUsetIter_case1];
                
                
            else % CASE 2 & 3: ~(0 \in Pi)
                if Options.verbose > 0,
                    disp(sprintf('region: i=%d /%d',ii,ctrl.nr))
                end

                % CASE 2:  ~(0 \in Pi) & (0<= Jj(x) <= Ji_min)
                %       => XUset is convex!!
                ind = find(Jmaxmin(jj_ind,2)<Jmaxmin(ii,2));
                jj_case2 = jj_ind(ind);

                XUsetIter_case2 = Pe;

                
                for jj = jj_case2',

                    if ismember(jj,ind_flat) % skip "flat" regions
                        continue
                    end

                    H2 = LyapFcn.Set(jj).A;
                    K2 = LyapFcn.Set(jj).b;
                    b2 = LyapFcn.Set(jj).getFunction('obj').F;
                    c2 = LyapFcn.Set(jj).getFunction('obj').g;

                    P = XUpoly(Hu, Ku, H1, K1, H2, K2, b1, c1, b2, c2, A, B, f,...
                        nx, nu, 0, Jmaxmin(ii,2), Options.dV_eps);

                    XUsetIter_case2 = [XUsetIter_case2 P];
                end%jj

                % set is guaranteed convex!
                if Options.convex
                    U = PolyUnion(XUsetIter_case2);
                    if U.isConvex() 
                        XUsetIter_case2 = U.convexHull; 
                    else
                        error('Set should be convex (numerical problems)')
                    end
%                     [stat,env] = isconvex(XUsetIter_case2, isConvexOpt);
%                     if stat
%                         XUsetIter_case2 = env;
%                     else
%                         isConvexOpt2 = isConvexOpt;
%                         isConvexOpt2.abs_tol = mptOptions.abs_tol;
%                         [stat,env] = isconvex(XUsetIter_case2, isConvexOpt2);
%                         if stat
%                             XUsetIter_case2 = env;
%                         else
%                             disp('error: set should be convex (numerical problems)')
%                         end
%                     end
                end


                
                % CASE 3:  ~(0 \in Pi) & (Ji_min < Jj(x) <= Ji_max)
                %       => XUset is possibly non-convex!!
                
                ind = [];
                ind = find(Jmaxmin(jj_ind,1)<=Jmaxmin(ii,2));
                jj_case3 = setdiff(jj_ind,jj_ind(ind));

                XUsetIter_case3 = Pe;

                for jj = jj_case3',

                    if ismember(jj,ind_flat) % skip "flat" regions
                        continue
                    end
                    
                    H2 = LyapFcn.Set(jj).A;
                    K2 = LyapFcn.Set(jj).b;
                    b2 = LyapFcn.Set(jj).getFunction('obj').F;
                    c2 = LyapFcn.Set(jj).getFunction('obj').g;

                    P = XUpoly(Hu, Ku, H1, K1, H2, K2, b1, c1, b2, c2, A, B, f,...
                        nx, nu, Jmaxmin(ii,2), inf, Options.dV_eps);

                    XUsetIter_case3 = [XUsetIter_case3 P];
                end%jj


                % if set is convex, use it (does not need to be convex!)
                 U = PolyUnion(XUsetIter_case3);
                    if U.isConvex(),XUsetIter_case3 = U.convexHull; end

%                 if Options.convex
%                     [stat,env] = isconvex(XUsetIter_case3, isConvexOpt);
%                     if stat
%                         XUsetIter_case3 = env;
%                     end
%                 end
                

                %                 % set is possibly non-convex
                %                 if Options.merge>0 & length(XUsetIter_case3)>1
                %                     if Options.merge ==1      % greedy merging
                %                         XUsetIter_case3 = merge(XUsetIter_case3);
                %                     elseif Options.merge ==2  % optimal mergin
                %                         XUsetIter_case3 = merge(XUsetIter_case3,struct('greedy',0));
                %                     end
                %                 end

                



                XUsetIter = [XUsetIter_case2 XUsetIter_case3];
                
                % if set is convex, use it
                if Options.convex
                    U = PolyUnion(XUsetIter);
                    if isConvex_mod(U,isConvexOpt.abs_tol)
                        XUsetIter = U.convexHull; 
                    end
%                     if U.isConvex(),XUsetIter = U.convexHull; end
%                     [stat,env] = isconvex(XUsetIter, isConvexOpt);
%                     if stat
%                         XUsetIter = env;
%                     end
                end

                % TO DELETE
                if length(XUsetIter) > 1
                    plot(XUsetIter)
                    status = isConvex_mod(PolyUnion(XUsetIter),isConvexOpt.abs_tol);
                end


            end % different cases for ii

        
        else
            % CONVENTIONAL (OLD) VERSION: not detecting convex XU parts, 
            % just 'reachability'
            if Options.verbose > 0,
                if ~ismember(ii,zero_inside)
                    disp(sprintf('region: i=%d /%d',ii,ctrl.nr))
                else
                    if Options.gamma > 0
                        disp(sprintf('region: i=%d /%d  [region at 0: additional projection computations]',...
                            ii,ctrl.nr))
                    else
                        disp(sprintf('region: i=%d /%d  [region at 0]',ii,ctrl.nr))
                    end
                end
            end
            
            for jj = jj_ind',

                if ismember(jj,ind_flat) % skip "flat" regions
                    continue
                end


                H2 = LyapFcn.Set(jj).A;
                K2 = LyapFcn.Set(jj).b;
                b2 = LyapFcn.Set(jj).getFunction('obj').F;
                c2 = LyapFcn.Set(jj).getFunction('obj').g;

                if ~ismember(ii,zero_inside)
                    P = XUpoly(Hu, Ku, H1, K1, H2, K2, b1, c1, b2, c2, A, B, f,...
                        nx, nu, 0, Jmaxmin(jj,1), Options.dV_eps);
                else
                    if Options.gamma > 0
                        P = XUpoly_zero(Hu, Ku, H1, K1, H2, K2, b1, c1, b2, c2, A, B, f,...
                            nx, nu, Options.gamma, Options.norm_p, Options.psolvers);
                    else % don't use projection
                        P = XUpoly(Hu, Ku, H1, K1, H2, K2, b1, c1, b2, c2, A, B, f,...
                            nx, nu, 0, Jmaxmin(jj,1), Options.dV_eps);
                    end
                end
                
                XUsetIter = [XUsetIter P];

                % TO DELETE
                if length(XUsetIter) > 1
                    plot(XUsetIter)
                end
            end%jj

        end %if



        % Intermediate Merging: post-merging for region ii
        if Options.merge>0 & length(XUsetIter)>1
            if Options.merge ==1      % greedy merging
                XUsetIter = merge(XUsetIter);
            elseif Options.merge ==2  % optimal mergin
                XUsetIter = merge(XUsetIter,struct('greedy',0));
            end
        end

        XUset = [XUset XUsetIter];
    end%ii


else  % PWA system
    error('Sorry, PWA systems are not supported ...')
%     
%     [nx,nu,ny,ndyn,nbool,ubool,intInfo] = mpt_sysStructInfo(sysStruct);
%     
%     for ii = 1:ctrl.nr,  % source region
% % for ii = 129
% 
%         if ismember(ii,ind_flat) % skip "flat" regions
%             continue
%         end
% 
%         % find out to which dynamics does the region Pn(ii) belong to
%         Pn_in_dynamics = [];
%         for idyn = 1:ndyn,
%             if dointersect(intInfo.Pdyn{idyn}, Pn(ii)),
%                 Pn_in_dynamics = [Pn_in_dynamics idyn];
%             end
%         end
% 
%         [H1, K1] = double(Pn(ii));
%         b1 = Bi{ii};
%         c1 = Ci{ii};
%         XUsetIter = emptypoly;
%         
%         jj_ind = find(Jmaxmin(ii,1)>=Jmaxmin(:,2));
%         
%         % CONVENTIONAL (OLD) VERSION: not detecting convex XU parts, 
%         % just 'reachability'
%         if Options.verbose > 0,
%             if ~ismember(ii,zero_inside)
%                 disp(sprintf('region: i=%d / %d',ii,ctrl.nr))
%             else
%                 if Options.gamma > 0
%                     disp(sprintf('region: i=%d / %d  [region at 0: additional projection computations]',...
%                         ii,ctrl.nr))
%                 else
%                     disp(sprintf('region: i=%d / %d  [region at 0]',ii,ctrl.nr))
%                 end
%             end
%         end
%         
%         for idyn = Pn_in_dynamics,
%         
%             Gx = sysStruct.guardX{idyn};
%             Gu = sysStruct.guardU{idyn};
%             Gc = sysStruct.guardC{idyn};
%                 
%             for jj = jj_ind',
%   
%                 if ismember(jj,ind_flat) % skip "flat" regions
%                     continue
%                 end
%                 
%                 [H2, K2] = double(Pn(jj));
%                 b2 = Bi{jj};
%                 c2 = Ci{jj};
%                 
%                 if ~ismember(ii,zero_inside)
%                     P = XUpoly(Hu, Ku, H1, K1, H2, K2, b1, c1, b2, c2, A{idyn}, B{idyn}, f{idyn},...
%                         nx, nu, 0, Inf, Options.dV_eps, Gx, Gu, Gc);
%                 else
%                     if Options.gamma > 0
%                         P = XUpoly_zero(Hu, Ku, H1, K1, H2, K2, b1, c1, b2, c2, A{idyn}, B{idyn}, f{idyn},...
%                             nx, nu, Options.gamma, Options.norm_p, Options.psolvers, Gx, Gu, Gc);
%                     else % don't use projection
%                         P = XUpoly(Hu, Ku, H1, K1, H2, K2, b1, c1, b2, c2, A{idyn}, B{idyn}, f{idyn},...
%                             nx, nu, 0, Inf, Options.dV_eps, Gx, Gu, Gc);
%                     end
%                 end
%                 
%                 XUsetIter = [XUsetIter P];
%                 
%             end%jj
%             
%         end%ndyn
% 
%         if Options.convex,
%             % detect if union is convex
%             [iscvx, cvxset] = isconvex(XUsetIter, isConvexOpt);
%             if iscvx,
%                 if Options.verbose > 1,
%                     disp('Union is convex.');
%                 end
%                 % if the union is convex, use the convex representation
%                 XUsetIter = cvxset;
%             end
%         else
%             iscvx = 0;
%         end
% 
%         if iscvx == 0 & Options.merge > 0,
%             % union is not convex
%             if Options.convex==1 & Options.verbose > 1,
%                 fprintf('union is not convex\n');
%             end
% 
%             if Options.merge>0 & length(XUsetIter)>1
% 
%                 % use reduceunion() to get ride of some regions
%                 XUsetIter = reduceunion(XUsetIter);
% 
%                 % Intermediate Merging: post-merging for region ii
%                 if Options.verbose > 1,
%                     disp('Trying to simplify non-convex union.');
%                 end
%                 
%                 % now use greedy or optimal merging to simplify the
%                 % representation and to obtain less regions
%                 if Options.merge ==1      % greedy merging
%                     XUsetIter = merge(XUsetIter);
%                 elseif Options.merge ==2  % optimal mergin
%                     XUsetIter = merge(XUsetIter,struct('greedy',0));
%                 end
%             end
%         end
%         
%         XUset = [XUset XUsetIter];
%     end%ii
%         
end


return






% SUB-FUNCTIONS __________________________________
function P = XUpoly(Hu, Ku, H1, K1, H2, K2, b1, c1, b2, c2, A, B, f,...
        nx, nu, Jmin, Jmax, dV_eps, Gx, Gu, Gc)

H = [];
K = [];

nc1 = length(K1);
nc2 = length(K2);

% x \in P1
H = [H1 zeros(nc1, nu)];
K = K1;

% A*x + B*u + f \in P2
H = [H; H2*A H2*B];
K = [K; K2 - H2*f];

% u \in U
H = [H; zeros(2*nu, nx) Hu];
K = [K; Ku];

if nargin > 18,
    % PWA guards:
    % Gx*x + Gu*u <= Gc
    H = [H; Gx Gu];
    K = [K; Gc];
end

if Jmin > 0   % for Case 3: 0 < Ji_min <= J(x^+)
    H = [H; -b2*[A B]];
    K = [K; c2 - Jmin - b2*f];
end

if isinf(Jmax)
    % b2*(A*x+B*u+f) + c2 - (b1*x + c1) <= - dV_eps
    H = [H; (b2*A - b1) b2*B];
    K = [K; c1 - c2 - dV_eps - b2*f];

else
    % b2*(A*x+B*u+f) + c2 - Jmax <= - dV_eps
    H = [H; b2*A b2*B];
    K = [K; -c2 + Jmax - dV_eps - b2*f];
end

P = Polyhedron(H, K);
return



% (0 \in Pi): regions around the origin
function P = XUpoly_zero(Hu, Ku, H1, K1, H2, K2, b1, c1, b2, c2, A, B, f,...
        nx, nu, gamma, p, psolvers, Gx, Gu, Gc)

H = [];
K = [];

nc1 = length(K1);
nc2 = length(K2);

% x \in P1
H = [H1 zeros(nc1, nu)];
K = K1;

% A*x + B*u + f \in P2
H = [H; H2*A H2*B];
K = [K; K2 - H2*f];

if nargin > 18,
    % PWA guards:
    % Gx*x + Gu*u <= Gc
    H = [H; Gx Gu];
    K = [K; Gc];
end

% u \in U
H = [H; zeros(2*nu, nx) Hu];
K = [K; Ku];

% b2*(A*x+B*u+f) + c2 - (b1*x + c1) <= - gamma * ||x||_p
if isinf(p)  % inf-norm case
    H = [H zeros(size(H,1),1)];

    H = [H; (b2*A - b1) b2*B gamma];
    K = [K; c1 - c2 - b2*f];

    H = [H; eye(nx) zeros(nx,nu) -ones(nx,1); -eye(nx) zeros(nx,nu) -ones(nx,1)];
    K = [K; zeros(2*nx,1)];

else % 1-norm case
    H = [H zeros(size(H,1),nx)];

    H = [H; (b2*A - b1) b2*B gamma*ones(1,nx)];
    K = [K; c1 - c2 - b2*f];

    H = [H; eye(nx) zeros(nx,nu) -eye(nx); -eye(nx) zeros(nx,nu) -eye(nx)];
    K = [K; zeros(2*nx,1)];
end

P = Polyhedron(H, K);
if isFullDim(P)
    for ii = 1:length(psolvers)
        Pproj = projection(P,[1:(nx+nu)], [], struct('psolvers', psolvers(ii:end)));
        if ~isFullDim(Pproj) | ~isBounded(Pproj)
            if length(psolvers) > ii,
                fprintf('Warning: Numerical problems with projection, recomputing...\n');
            else
                error('Numerical problems with projection.');
            end
        else
            % hopefully correct (or at least fully-dimensional) projection found
            break
        end
    end
    P = Pproj;
end
return


function ts = isConvex_mod(obj,convxtol)
%
% check if the polyhedron array forms a convex union
%

if nargin < 2, convxtol = 1e-6;end

% deal with arrays
if numel(obj)>1
    ts = -ones(size(obj));
    for i=1:numel(obj)
        ts(i) = obj(i).isConvex;
    end
    return;
end

% empty obj
if obj.Num==0
    ts = false;
    return;
end

% if isempty(obj.Internal.Convex)
    % compute the convex hull
	H = obj.convexHull();
	% the union is convex if H\U = emptyset
	ts = all(isEmptySet(mldivide(H, obj.Set, true)));
    
    % This part is new: Need to better impose tollerance
    if ~ts
        temp = mldivide(H, obj.Set, true);
        temp2 = temp.chebyCenter;
        if temp2.r <= convxtol, ts = 1;end
    end

    % store internally
%     obj.Internal.Convex = ts;

% else
%     ts = obj.Internal.Convex;
% end
return


% function [status, Pconv] = isconvex(P,Options)
% %ISCONVEX Checks if a polytope array forms a convex union
% %
% % [status, Pconv] = isconvex(P)
% % [status, Pconv] = isconvex(P,Options)
% %
% % ---------------------------------------------------------------------------
% % DESCRIPTION
% % ---------------------------------------------------------------------------
% % STATUS = ISCONVEX(P) returns TRUE (1) if a given polytope array P is convex
% %
% % ---------------------------------------------------------------------------
% % INPUT
% % ---------------------------------------------------------------------------
% % P                - polytope array
% % Options.usehull  - if set to 1 (default is 0), uses convex hulls instead of
% %                    convex envelopes to detect convexity
% %
% % Note: If Options is missing or some of the fields are not defined, the default
% %       values from mptOptions will be used
% %
% % ---------------------------------------------------------------------------
% % OUTPUT                                                                                                    
% % ---------------------------------------------------------------------------
% % status           - Logical statement
% % Pconv            - if P is convex it returns the convex set, otherwise 
% %                    an empty polytope
% %
% 
% % Copyright is with the following author(s):
% %
% % (C) 2005 Frank J. Christophersen, Automatic Control Laboratory, ETH Zurich,
% %          fjc@control.ee.ethz.ch
% % (C) 2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
% %          kvasnica@control.ee.ethz.ch
% 
% % ---------------------------------------------------------------------------
% % Legal note:
% %          This program is free software; you can redistribute it and/or
% %          modify it under the terms of the GNU General Public
% %          License as published by the Free Software Foundation; either
% %          version 2.1 of the License, or (at your option) any later version.
% %
% %          This program is distributed in the hope that it will be useful,
% %          but WITHOUT ANY WARRANTY; without even the implied warranty of
% %          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
% %          General Public License for more details.
% % 
% %          You should have received a copy of the GNU General Public
% %          License along with this library; if not, write to the 
% %          Free Software Foundation, Inc., 
% %          59 Temple Place, Suite 330, 
% %          Boston, MA  02111-1307  USA
% %
% % ---------------------------------------------------------------------------
% 
% global mptOptions;
% 
% if nargin<2,
%     Options = [];
% end
% 
% if ~isfield(Options, 'usehull'),
%     Options.usehull = 0;
% end
% if ~isfield(Options, 'abs_tol'),
%     Options.abs_tol = mptOptions.abs_tol;
% end
% if ~isfield(Options, 'bbox_tol'),
%     % higher tolerance to check bounding boxes
%     % it is so high to avoid numerical problems
%     Options.bbox_tol = 1e4*mptOptions.abs_tol;
% end
% 
% % if ~isa(P, 'polytope'),
% %     error('ISCONVEX: First input must be a polytope object.');
% % end
% 
% if ~all(P.isFullDim)
%     % non-fully dimensional polytopes are assumed to be convex
%     status = 1;
%     Pconv  = P;
%     return
% end
% if length(P) == 1
%     % single polytope, it is for sure convex
%     status = 1;
%     Pconv  = P;
%     return
% end
% 
% if Options.usehull==1,
%      PU = PolyUnion(P);
%      outer = PU.convexHull;
% %     outer = hull(P, Options);
% else
%     PU = PolyUnion(P);
%     outer = PU.envelope;
% %     outer = envelope(P, Options);
% end
% 
% bboxOpt.noPolyOutput = 1;  % tell bounding_box that we just need vertices
% [R, Pl, Pu] = bounding_box(P, bboxOpt);
% [R, Ol, Ou] = bounding_box(outer, bboxOpt);
% bboxP = [Pl Pu];
% bboxO = [Ol Ou];
% 
% if any(isnan(Ol)) | any(isnan(Ou)) | isempty(Ol) | isempty(Ou)
%     % if envelope returns R^n, bounding_box returns NaNs
%     status = 0;
%     Pconv = mptOptions.emptypoly;
%     return
% end    
% 
% bbox_tol = Options.bbox_tol;
% if any(abs(bboxP(:,1) - bboxO(:,1)) > bbox_tol) | any(abs(bboxP(:,2) - bboxO(:,2)) > bbox_tol),
%     % bounding boxes differ by more than bbox_tol => polytopes cannot be equal
%     % therefore P is not convex
%     status = 0;
%     Pconv = mptOptions.emptypoly;
%     return
% end
% % we cannot reach any conclusion based solely on the fact that bounding
% % boxes are identical, therefore we continue...
% 
% mldivideOpt = Options;
% mldivideOpt.simplecheck = 1;   % tell set difference not to construct polytopes
% if isfulldim(mldivide(outer, P, mldivideOpt)),
%     % if set difference between the envelope (hull) and P is fully dimensional,
%     % it means that P is not convex
%     status = 0;
%     Pconv  = mptOptions.emptypoly;
% else
%     % set difference is empty => P is convex
%     status = 1;
%     Pconv  = outer;
% end
% return