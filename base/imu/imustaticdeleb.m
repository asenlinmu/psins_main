function [imu, eb, att] = imustaticdeleb(imu, pos, isfig)
% Using static-base condition to correct the gyro output by deleting their bias.
%
% Prototype: [imu, eb] = imustaticdeleb(imu, pos)
% Inputs: imu - raw SIMU data
%         pos - position [lat;lon;hgt]
% Outputs: imu - new SIMU data with gyro bias deleted
%          eb - gyro bias
%
% See also  delbias, imuadderr, imudeldrift, imudeleb, imucpn, imustaticscale.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 06/12/2025
global glv
    if nargin<3, isfig=0; end  % imuplot(imu);  avarimu(imu);
    if nargin<2
        [att, attk, eb] = alignsb(imu);
    else
        [att, attk, eb] = alignsb(imu, pos);
    end
    if isfig==0, close(gcf); end
    eb / glv.dph,
    ts = diff(imu(1:2,end));
    for k=1:3
        imu(:,k) = imu(:,k)-eb(k)*ts;
    end
    