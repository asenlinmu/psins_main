function m = uaanalysis(gyro, rate, lat)
% Gyro uniform angular rate analysis, usually tested by single-axis turntable.
%
% Prototype: m = uaanalysis(gyro, rate, lat)
% Inputs: gyro - gyro angular rate (in rad/s)
%         rate - reference angular rate, (in rad/s)
%         lat - test latitude, or position [lat,lon,hgt]
% Output: m - mean of gyro
%          
% See also  heanalysis.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/02/2026
global glv
    if nargin<3, lat=0; end
    wU = glv.wie*sin(lat(1));
    if size(gyro,2)==1, ts=1; t=(1:length(gyro))';
    else, ts=diff(gyro(1:2,end)); end
    if size(gyro,2)>4, t=gyro(:,end); gyro=gyro(:,3)/ts; end  % gyro=imu
    m = mean(gyro(:,1));  err = gyro(:,1)-m;
    if nargin<2, rate=m;
    else, rate=rate+wU; end
    myfig
    subplot(211), plot(t,gyro(:,1)/glv.dps), hline(t,[m;rate]/glv.dps);  xygo('w');  legend('raw','mean','ref')
        title(sprintf('(mean-ref)/ref=%.3f ppm',(m-rate)/rate*1e6));
    subplot(212), plot(t,cumsum(err)*ts/glv.sec);  xygo('\Sigma\theta / \prime\prime');
