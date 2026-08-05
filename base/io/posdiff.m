function [dxyz, dg] = posdiff(pos0, pos1)
% Calculate two position difference in meter.
%
% Prototype: dxyz = posdiff(pos0, pos1)
% Inputs: pos0,pos1 - two positions.
% Outputs: dxyz - E/N/U coordinate difference in meter =  pos1-pos0
%          dg - gravity difference
%          
% See also  pos2dxyz, disppos.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/05/2026
    if size(pos0,2)>3, pos0 = pos0(1,:)'; end  % if pos array, then first pos 
    if length(pos0)>3, pos0=pos0(end-3:end-1); end  % the last column is t
    if size(pos1,2)>3, pos1 = pos1(1,:)'; end
    if length(pos1)>3, pos1=pos1(end-3:end-1); end
    dxyz = pos2dxyz(pos1,pos0);
    if nargout>1
        eth0 = earth(pos0);  eth1 = earth(pos1);
        dg = -(eth1.gn(3)-eth0.gn(3));  % ~= -dxyz(3)*glv.beta2
    end
