function [wnie, g, gn, wN, wU] = wnieg(pos)
% Calculate the Earth related parameters, wnie & g_Lh.
%
% Prototype: [wnie, g, gn, wN, wU] = wnieg(pos)
% Input: pos - geographic position [lat;lon;hgt]
% Outputs: wnie - [0; wN; wU]
%          g,gn - gravity
%
% See also  earth, wbief.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/12/2023
    if length(pos)<2; pos=[pos;0;0]; end  % latitude only
    eth = earth(pos);
    wnie = eth.wnie;   g = eth.g;
    gn = [0;0;-g];
    wN = wnie(2);  wU = wnie(3);