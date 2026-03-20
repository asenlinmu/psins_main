function g = imugrav(imu, pos, isfig)
% Gravity measure by IMU in static base, maybe swaying.
%
% Prototype: g = imugrav(imu, pos, isfig)
% Inputs: imu - IMU before calibration
%         pos - =[lat;lon;hgt]
%         isfig - figure flag
% Output: g - local gravity magnitude 
%
% See also  gravj4, imuclbt.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 01/11/2025
global glv
    if nargin<3, isfig=1; end
    if nargin<2, pos=zeros(3,1); end
    att = aligni0(imu, pos, 0);
    avp = inspure(imu, [att;pos], 'O');
    [wnie, g] = wnieg(pos);
    pp = polyfit(avp(:,end), avp(:,6), 1);  aU=pp(1);
    g = g+aU;
    subplot(3,2,3), plot(avp(:,end),polyval(pp,avp(:,end)),'linewidth',2);
    title(sprintf('aU=%.3f(ug), imugrav=%.8f',aU/glv.ug, g));
    if isfig==0, close(gcf); end
