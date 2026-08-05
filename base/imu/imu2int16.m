function [imu, inc1] = imu2int16(imu, inc0)
% Trans IMU array to int16 array for data compress.
%
% Prototype: [imu, inc1] = imu2int16(imu, inc0)
% Inputs: imu - 6-column IMU double array without time tag
%         inc0 - initial gyro ang/acc vel increment
% Outputs: imu - 6-column IMU int16 array
%          inc1 - end gyro ang/acc vel increment
%
% See also  imuzip, imuplot, binfile16.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 15/06/2026
global glv
    gyro_q = 0.1*glv.sec;  acc_q = 1.0e-4;  % 1e-4 m/s ~= 10ug*s
    if nargin<2; inc0=zeros(1,6); end
    imu = [inc0; imu(:,1:6)];
    imu = cumsum(imu);
    imu(:,1:3) = imu(:,1:3)/gyro_q;
    imu(:,4:6) = imu(:,4:6)/acc_q;
    inc1 = imu(end,1:6);
    imu = fix(imu);  inc1 = inc1-imu(end,:);
    imu = int16(diff(imu));