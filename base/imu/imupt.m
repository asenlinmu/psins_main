function imu = imupt(imu, att0, pt)
% IMU polarity test.
%
% Prototype: imu = imupt(imu, att0, pt)
% Inputs: imu - IMU arrya
%         att0 - initial attitude, or latitude
%         pt - gyro/acc polarity
%
% See also  imutclbt, imumeanplot, Mahony.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 15/06/2026
global glv
    imu = imuresample(imu,0.1);
    if nargin<3, pt=ones(6,1); end
    if nargin<2, att0=alignsb(imu(1:10,:),glv.pos0,0.0); end
    for k=1:length(pt), imu(:,k)=imu(:,k)*pt(k); end
    if length(att0)==1  % by checking the Up-axis gyro/acc outputs, att0=pos(1)
        wU = glv.wie*sin(att0(1));  
        ts=0.1; T=10;  im = imumeanplot(imu, T/ts);  close(gcf);
        msplot(211, im(:,end),im(:,1:3)/T/glv.dph), xygo('wdph');  ylim([-1,1]*20);
                    plot(im([1,end],end), [-wU,wU;-wU,wU]/glv.dph, '--');
        msplot(212, im(:,end),im(:,4:6)/T/glv.g0), xygo('f');
        return;
    end
    q0 = a2qua(att0);
    attfb = imu(:,1:6);
    for k=1:length(imu)
        attfb(k,:) = [q2att(q0); qmulv([q0(1);-q0(2:4)],[0;0;1])]';
        q0 = qupdt(q0, imu(k,1:3)');
    end
    % imuplot(imu,1);
    myfig,  ts=0.1;
    subplot(411), plot(imu(:,end), imu(:,1:3)/glv.dps/ts, 'LineWidth',2); xygo('w');  title('X_R Y_F Z_U');
    subplot(413), plot(imu(:,end), imu(:,4:6)/glv.g0/ts, 'LineWidth',2);  xygo('f');  mylegend('fx','fy','fz');
    subplot(412), plot(imu(:,end), attfb(:,1:3)/glv.deg, 'LineWidth',2);  xygo('att');
    subplot(414), plot(imu(:,end), attfb(:,4:6)/glv.g0, 'LineWidth',2);  xygo('f_{est} / g');
