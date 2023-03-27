function [ctrlXU,out] = mpt_XUctrl_mpt3(ctrl,Options,LyapFcn)
%MPT_XUCTRL computes the stabilizing control set on the basis of a given PWA Lyapunov function
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
% ctrlXU                  - updated MPTCTRL object:
%   .details.XU           - XU sets
%   .details.XUproj       - XU sets projected down to X-space
%   .details.XUtime       - time spent when computing XU sets
%   .details.proj_time    - time spent when computing projection of XU sets
%

% Copyright is with the following author(s):
%
% (C) 2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch

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

if nargin < 2,
    Options = [];
end

if ~isfield(Options, 'psolvers'),
    Options.psolvers = [3 1 5 2 4 0];
end

startt = clock;
if nargin==3,
    XU = mpt_XUsetPWALyap_mpt3(ctrl, Options, LyapFcn);
else
    XU = mpt_XUsetPWALyap_mpt3(ctrl, Options);
end
xutime = etime(clock, startt);

if ~isBounded(XU),
    error([mfilename ': XU set is not bounded, probably due to numerical problems.']);
end

% nx = mpt_sysStructInfo(ctrl.sysStruct);
nx = ctrl.model.nx;

startt = clock;
XUproj = projection(XU, 1:nx, Options);
if ~isBounded(XUproj),
    error([mfilename ': projected XU set is not bounded, probably due to numerical problems.']);
end
projectiontime = etime(clock, startt);

% ctrlXU = struct(ctrl);
% ctrlXU.details.XU = XU;
% ctrlXU.details.XUproj = XUproj;
% ctrlXU.details.XUtime = xutime;
% ctrlXU.details.proj_time = projectiontime;
% ctrlXU = mptctrl(ctrlXU);

% ctrlXU = copy(ctrl);
ctrlXU = [];
out.XU = XU;
out.XUproj = XUproj;
out.XUtime = xutime;
out.proj_time = projectiontime;
end
