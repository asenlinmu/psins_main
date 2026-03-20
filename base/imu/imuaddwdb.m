function imu = imuaddwdb(imu, wdb)
% Add acc velocity random walk for IMU.
%
% Prototype: imu = imuaddwdb(imu, wdb)
% Inputs: imu - raw SIMU data
%         wdb - acc random walk, in ug/sqrt(Hz)
% Output: imu - new SIMU data with acc random walk added
%
% See also  imuadddb, imuaddeb, imuaddweb, imuaddedb, delbias, imuadderr, addslope, imudeldrift.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 16/12/2025
global glv
    if length(wdb)==1, wdb=[wdb;wdb;wdb]; end
    ts = diff(imu(1:2,end));
    for k=4:6
        imu(:,k) = imu(:,k)+wdb(k-3)*glv.ugpsHz*sqrt(ts);
    end
    