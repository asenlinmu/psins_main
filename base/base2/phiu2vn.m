function phiu2vn(phiu, T, lti)
% Calculating yaw misalign angles iduced SINS north-velocity/position error.
%
% Prototype: phiu2vn(phiu, T, lti)
% Inputs: phiu - yaw misalignment
%         T - alignment time length
%         lti - latitude;
% Output: NA
%
% Example:
%   phiu2vn(3*glv.min, 300);
%   phiu2vn(1*glv.min, 180, 34*glv.deg);
%
% See also  vn2phiu.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/03/2025
global glv
    if nargin<3, lti=0; end
    if nargin<2, T=300; end
    t = 1:T;
    phie = -phiu*glv.wie*cos(lti(1))*t;
    vn = -phiu*glv.g0*glv.wie*cos(lti(1))/2*t.^2;
    myfig,
    subplot(311), plot(t, phie/glv.sec);  xygo('phiE');  title(sprintf('\\phi_U=%.3f^\\prime ,  L=%.3f^\\circ',phiu/glv.min,lti(1)/glv.deg));
    subplot(312), plot(t, vn);  xygo('VN');
    subplot(313), plot(t, cumsum(vn));  xygo('North / m');
