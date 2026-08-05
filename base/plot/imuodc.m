function io = imuodc(imu, od)
% Combine IMU/OD data.
%
% Prototype: io = imuodc(imu, od)
% Inputs: imu - IMU data
%         od - OD data
% Output: io - IMU/OD data
%
% See also  imuodplot, imuplot, odplot.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/04/2026
    if length(imu)~=length(od)
        co = interp1(od(:,2), cumsum(od(:,1)), imu(:,end), 'nearest', 'extrap');
        io = [imu(:,1:end-1), diff([co(1);co]), imu(:,end)];
    else
        io = [imu(:,1:end-1), od];
    end