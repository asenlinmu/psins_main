function imu = imuaddeb(imu, eb)
% Add gyro bias for IMU.
%
% Prototype: imu = imuaddeb(imu, eb)
% Inputs: imu - raw SIMU data
%         eb - gyro bias, in deg/h
% Output: imu - new SIMU data with gyro bias added
%
% See also  imuadddb, imuaddedb, imustaticdeleb, imuaddkg, imuadderr, imudeldrift.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 07/12/2025
global glv
    if length(eb)==1, eb=[eb;eb;eb]; end
    ts = diff(imu(1:2,end));
    for k=1:3
        imu(:,k) = imu(:,k)+eb(k)*glv.dph*ts;
    end
    