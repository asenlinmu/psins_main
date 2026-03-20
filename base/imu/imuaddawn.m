function imu = imuaddawn(imu, awn)
% Add gyro angular Gauss white noise for IMU.
%
% Prototype: imu = imuaddawn(imu, awn)
% Inputs: imu - raw SIMU data
%         awn - angular angular white noise, in sec/sqrt(Hz)
% Output: imu - new SIMU data angular angular white noise added
%
% Example
%   imu0 = imustatic(34*glv.deg, 0.1, 100);  % imuplot(imu2);
%   imu1 = imuaddawn(imu0, 1);  imu2 = imuresample(imu1,1); imucsplot(imu1,2);
%   myfig, plot(sumn(imu1(:,1),10)/glv.sec);

% See also  imuaddweb, imuadddb, imuaddwdb, imuaddedb, imustaticdeleb, imuaddkg, imuadderr, imudeldrift.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 16/12/2025
global glv
    if length(awn)==1, awn=[awn;awn;awn]; end
    ts = diff(imu(1:2,end));  len = length(imu)+1;
    for k=1:3
        ak = awn(k)*glv.secpsHz/sqrt(ts)*randn(len,1);
        imu(:,k) = imu(:,k)+diff(ak);
    end
    