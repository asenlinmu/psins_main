function imu = imuaddedb(imu, eb, db)
% Add gyro&acc bias for IMU.
%
% Prototype: imu = imuaddedb(imu, eb, db)
% Inputs: imu - raw SIMU data
%         eb - gyro bias, in deg/h
%         db - acc bias, in ug
% Output: imu - new SIMU data with gyro&acc bias added
%
% See also  imuaddeb, imuadddb, delbias, imuadderr, imudeldrift.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 07/12/2025
global glv
    if length(eb)==1, eb=[eb;eb;eb]; end
    if length(db)==1, db=[db;db;db]; end
    ts = diff(imu(1:2,end));
    for k=1:3
        imu(:,k) = imu(:,k)+eb(k)*glv.dph*ts;
    end
    for k=4:6
        imu(:,k) = imu(:,k)+db(k-3)*glv.ug*ts;
    end
    