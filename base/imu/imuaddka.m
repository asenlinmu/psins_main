function imu = imuaddka(imu, ka)
% Add gyro scale for IMU.
%
% Prototype: imu = imuaddka(imu, ka)
% Inputs: imu - raw SIMU data
%         ka - acc scale error, in ppm
% Output: imu - new SIMU data with acc scale added
%
% See also  imuaddka2, imuadddb, imuaddkg, imuaddeb, imuaddedb, imuadderr, imudeldrift.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 07/12/2025
global glv
    if length(ka)==1, ka=[ka;ka;ka]; end
    for k=1:3
        imu(:,3+k) = imu(:,3+k)*(1+ka(k)*glv.ppm);
    end
    