function [att1, att0] = imutransatt(imu, att)
% Coarse attitude transfer by IMU-gyro, neglecting gyro bias & Earth rotation.
%
% Prototype: [att1, att0] = imutransatt(imu, att)
% Inputs: imu - IMU gyro array
%         att - initial or end attitude Euler angle/quaternion
% Outputs: att1,att0 - end or initial Euler angle/quaternion
%
% See also  rv2qBatch, qq2rv, qq2phi, qq2afa.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/06/2026
    q = rv2qBatch(imu);  q01 = q(end,:)';
    if length(att)==3
        att1 = qmul(a2qua(att),q01);  att1 = q2att(att1);  % suppose att to be start attitude, forward to get att1
        att0 = qmul(a2qua(att),qconj(q01));  att0 = q2att(att0);  % suppose att to be end attitude, backward to get att0
    else
        att1 = qmul(att,q01);
        att0 = qmul(att,qconj(q01));
    end