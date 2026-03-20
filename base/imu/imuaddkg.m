function imu = imuaddkg(imu, kg)
% Add gyro scale for IMU.
%
% Prototype: imu = imuaddkg(imu, eb)
% Inputs: imu - raw SIMU data
%         kg - gyro scale error, in ppm
% Output: imu - new SIMU data with gyro scale added
%
% See also  imuaddeb, imuaddka, imuadddb, imuaddedb, imuadderr, imudeldrift.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 07/12/2025
global glv
    if length(kg)==1, kg=[kg;kg;kg]; end
    for k=1:3
        imu(:,k) = imu(:,k)*(1+kg(k)*glv.ppm);
    end
    