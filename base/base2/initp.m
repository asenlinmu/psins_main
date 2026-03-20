function [ts, pos, g, wnie] = initp(tsfs, lat, lon, hgt, g0)
% Sampling interval & geographic position initial setting.
%
% Prototype: [ts, pos, g, wnie] = initp(tsfs, lat, lon, hgt, g0)
% Inputs: tsfs - sampling interval, >1 for sampling frequency
%         lat,lon,hgt - lat and lon are always in arcdeg,
%             & height is in m.
%         g0 - gravity @ [lat;lon;hgt]
% Outputs: ts - sampling interval
%          pos - =[lat*arcdeg; lon*arcdeg; hgt]
%          g,wnie - gravity magnitude & Earth rate vector (ENU part)
% 
% See also  posset, avpset, llh, nnts, wnieg.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/09/2025
global glv
    if nargin<5, g0=[]; end 
    if nargin<4, hgt=0; end 
    if nargin<3, lon=0; end 
    if nargin<2, lat=glv.pos0(1); lon=glv.pos0(2); hgt=glv.pos0(3);  end 
    if tsfs>1,  tsfs=1/tsfs;  end
    ts = tsfs;
    pos = [lat*glv.deg; lon*glv.deg; hgt];
    [wnie, g] = wnieg(pos);
    if ~isempty(g0), g=g0; end  % 2025-12-04
