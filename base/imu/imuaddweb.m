function imu = imuaddweb(imu, web)
% Add gyro angular random walk for IMU.
%
% Prototype: imu = imuaddweb(imu, eb)
% Inputs: imu - raw SIMU data
%         web - angular random walk, in deg/sqrt(h)
% Output: imu - new SIMU data angular random walk added
%
% See also  imuaddweb, imuadddb, imuaddwdb, imuaddedb, imustaticdeleb, imuaddkg, imuadderr, imudeldrift.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 16/12/2025
global glv
    if length(web)==1, web=[web;web;web]; end
    ts = diff(imu(1:2,end));
    for k=1:3
        imu(:,k) = imu(:,k)+web(k)*glv.dpsh*sqrt(ts);
    end
    