function [wbie, fb] = wbief(pos, att)
% Calculate the gyro/acc output under static base.
%
% Prototype: [wbie, fb] = wbief(pos, att)
% Inputs: pos - geographic position [lat;lon;hgt]
%         att - attitude [pitch;roll;yaw]
% Outputs: wbie - static gyro algular rate in b-frame
%          fb - static acc specific force in b-frame
%
% See also  wnieg.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 25/06/2026
    [wnie, g, gn] = wnieg(pos);
    Cbn = a2mat(att)';
    wbie = Cbn*wnie;
    fb = -Cbn*gn;