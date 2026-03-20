function imu = imuadddb(imu, db)
% Add acc bias for IMU.
%
% Prototype: imu = imuadddb(imu, db)
% Inputs: imu - raw SIMU data
%         db - acc bias, in ug
% Output: imu - new SIMU data with acc bias added
%
% See also  imuaddeb, imuaddedb, delbias, imuadderr, addslope, imudeldrift.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 07/12/2025
global glv
    if length(db)==1, db=[db;db;db]; end
    ts = diff(imu(1:2,end));
    for k=4:6
        imu(:,k) = imu(:,k)+db(k-3)*glv.ug*ts;
    end
    