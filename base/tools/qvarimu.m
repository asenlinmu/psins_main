function [sigma, tau] = qvarimu(imu, maxT)
% Calculate quantization variance for SIMU gyro & acc.
%
% Prototype: [sigma, tau, m] = avarimu(imu, meanT)
% Inputs: imu - SIMU data
%         meanT - mean time
% Outputs: sigma - Allan std variance
%          tau - cluster time
%          m - sensor mean value
%
% Example
%     imu = imustatic(zeros(9,1), 0.01, 3600, imuerrset(0.01,100,0.01,1)); avarimu(imu);
%
% See also  avarimu, avars, avarfit.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 14/07/2023
global glv
    if nargin<2, maxT=10; end
    ts = diff(imu(1:2,end));  imu=imu(:,1:6);
    T = ts;  sigma=[]; tau=[];
    while 1
        if T>maxT, break; end
        sigma = [sigma; std(imu(:,1:6))];
        tau = [tau; T];
        imu = imu(1:2:end-1,:)+imu(2:2:end,:);  T=2*T;
    end
    myfig,
   	subplot(211), semilogx(tau, sigma(:,1:3)/glv.sec);  xygo('\it\tau \rm/ s', '\itQ_G\rm( \tau ) /\rm \prime\prime');
%   	subplot(224), loglog(tau(:,1), sigma(:,4:6));  xygo('\it\tau \rm/ s', '\it\sigma_A\rm( \tau ) /\rm mg'); legend('X','Y','Z');
