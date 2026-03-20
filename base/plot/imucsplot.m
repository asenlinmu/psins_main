function imu = imucsplot(imu, isDelMean)
% Plot IMU cumsum angular & velocity increment.
%
% Prototype: imucsplot(imu, isDelMean)
% Inputs: imu - IMU data to plot, 
%         isDelMean - delete data mean flag,
%                 =1 del first 1/3 data mean; =2 del mean; =3 del first element
%
% See also  imuplot, imumeanplot.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/10/2025
global glv;
    if nargin<2, isDelMean=1; end
    m = zeros(1,6);  ts=diff(imu(1:2,end));
    if isDelMean>=10, imuplot(imu,glv.dph); return; % = imuplot
    elseif isDelMean==1, for k=1:6, m(k)=mean(imu(1:fix(length(imu)/3),k)); imu(:,k)=cumsum(imu(:,k)-m(k)); end
    elseif isDelMean==2, for k=1:6, m(k)=mean(imu(:,k)); imu(:,k)=cumsum(imu(:,k)-m(k)); end
    elseif isDelMean==3, for k=1:6, m(k)=imu(1,k); imu(:,k)=cumsum(imu(:,k)-m(k)); end, end
    myfig,
    subplot(321), plot(imu(:,end), imu(:,1)/glv.sec), xygo('\Delta\theta_x / \prime\prime'); ptitle('ebx',m(1)/ts/glv.dph);
    subplot(323), plot(imu(:,end), imu(:,2)/glv.sec), xygo('\Delta\theta_y / \prime\prime'); ptitle('eby',m(2)/ts/glv.dph);
    subplot(325), plot(imu(:,end), imu(:,3)/glv.sec), xygo('\Delta\theta_z / \prime\prime'); ptitle('ebz',m(3)/ts/glv.dph);
    subplot(322), plot(imu(:,end), imu(:,4)*1000), xygo('\DeltaV_x / mm/s'); ptitle('fx',m(4)/ts/glv.g0);
    subplot(324), plot(imu(:,end), imu(:,5)*1000), xygo('\DeltaV_y / mm/s'); ptitle('fy',m(5)/ts/glv.g0);
    subplot(326), plot(imu(:,end), imu(:,6)*1000), xygo('\DeltaV_z / mm/s'); ptitle('fz',m(6)/ts/glv.g0);
