function [serr, Ang, Dang] = rotserr(ang, dang)
% Rotation scale error calculation.
%
% Prototype: serr = rotserr(ang, dang)
% Inputs: ang - rotation angle array
%         dang - rotation angle value
% Outputs: serr - scale error
%          Ang - total rotation angle
%          Dang - angle error
%
% Example2:
%   ang = angle2pi((0:0.1:10)*2*pi*(1+10*glv.ppm));    s = rotserr(ang),
%   ang = angle2pi((0:0.1:100.5)*2*pi*(1+10*glv.ppm)); s = rotserr(ang,pi),
%   
% See also  angle2c, angle2pi, att2c.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 14/11/2025
    if nargin<2, dang=0; end
    if size(ang,2)>1, ang=ang(:,3); end % yaw in avp array
    Dang = ang(end)-(ang(1)+dang);
    if Dang>3*pi/2, Dang=Dang-2*pi;
    elseif Dang<-3*pi/2, Dang=Dang+2*pi; end
    ang = angle2c(ang);  % myfig, plot(ang);
    Ang = ang(end)-ang(1);
    serr = Dang/Ang;