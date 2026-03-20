function alignscat(imu, pos, T, dT, mth)
% SINS initial align scatter in static base, with both real gyro and acc, gyro or acc only.
%
% Prototype: alignscat(imu, pos, T, dT)
% Inputs: imu - IMU data
%         pos - position
%         T - align time length
%         dT - time step
%         mth - method, default=1 by alginvn, =2 by aligni0fitp, =3 by alignsb
%
% Examples
%    alignscat([0.0001;1], 34, 300);
%    alignscat([0.0001;1], -34, 300, 200, 2);
%    alignscat([0.0001;1], -34, 600, 200, 3);
%
% See also  aligni0fitp, alignsb, vn2phistd.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/10/2025
global glv
    if nargin<5, mth=1; end
    if nargin<3, T=300; end
    if nargin<4, dT=T; end
    if length(pos)==1, pos=[pos*glv.deg;0;0]; end
    if numel(imu)==2, wedb=imu; imu=imustatic([zeros(6,1);pos],0.1,30*T,imuerrset(0,0,wedb(1),wedb(2)));
    else, imu(:,end)=imu(:,end)-imu(1,end); end
    qnb = alignsb(datacut(imu,0,10),pos,[],0);  phi0 = [3;3;30]*glv.min;
    imuerr = imuerrset(0.01*100,100,0.001,1);  wvn=[1;1;1]*0.0001;
    N = fix((imu(end,end)-imu(1,end)-T)/dT);
    myfig; subplot(131); t0=imu(1,end);
    for k=1:N   % real gyro & acc
        if mth==1
            qnb = alignsb(datacut(imu,t0+(k-1)*dT,t0+10+(k-1)*dT),pos,[],0);
            [att, attk] = alignvn(datacut(imu,t0+(k-1)*dT,t0+T+(k-1)*dT), qnb, pos, phi0, imuerr, wvn, 0);
            plot(attk(1:100:end,end)-t0-(k-1)*dT, attk(1:100:end,3)/glv.deg); hold on;
        elseif mth==2
            [att,res] = aligni0fitp(datacut(imu,t0+(k-1)*dT,t0+T+(k-1)*dT),pos,0);
            plot(res.attkp(1:100:end,end)-res.attkp(1,end), res.attkp(1:100:end,3)/glv.deg); hold on;
        elseif mth==3
            [att,attk] = alignsb(datacut(imu,t0+(k-1)*dT,t0+T+(k-1)*dT),pos,[],0);
            plot(attk(1:10:end,end)-t0-(k-1)*dT, attk(1:10:end,3)/glv.deg); hold on;
        end
    end
    xygo('y');  [s1,m1] = plotstd(3,2);  legend('gyro&acc')
    subplot(132);  imu1=imu;  for k=4:5, imu1(:,k)=mean(imu1(:,k));  end  % costant acc
    for k=1:N
        if mth==1
            qnb = alignsb(datacut(imu1,t0+(k-1)*dT,t0+10+(k-1)*dT),pos,[],0);
            [att, attk] = alignvn(datacut(imu1,t0+(k-1)*dT,t0+T+(k-1)*dT), qnb, pos, phi0, imuerr, wvn, 0);
            plot(attk(1:100:end,end)-t0-(k-1)*dT, attk(1:100:end,3)/glv.deg); hold on;
        elseif mth==2
            [att,res] = aligni0fitp(datacut(imu1,t0+(k-1)*dT,t0+T+(k-1)*dT),pos,0);
            plot(res.attkp(1:100:end,end)-res.attkp(1,end), res.attkp(1:100:end,3)/glv.deg); hold on;
        elseif mth==3
            [att,attk] = alignsb(datacut(imu1,t0+(k-1)*dT,t0+T+(k-1)*dT),pos,[],0);
            plot(attk(1:10:end,end)-t0-(k-1)*dT, attk(1:10:end,3)/glv.deg); hold on;
        end
    end
    xygo('y');  [s2,m2,t2] = plotstd(3,2);  legend('costant acc')
    subplot(133);  for k=1:3, imu(:,k)=mean(imu(:,k));  end  % costant gyro
    for k=1:N
        if mth==1
            qnb = alignsb(datacut(imu1,t0+(k-1)*dT,t0+10+(k-1)*dT),pos,[],0);
            [att, attk] = alignvn(datacut(imu,t0+(k-1)*dT,t0+T+(k-1)*dT), qnb, pos, phi0, imuerr, wvn, 0);
            plot(attk(1:100:end,end)-t0-(k-1)*dT, attk(1:100:end,3)/glv.deg); hold on;
        elseif mth==2
            [att,res] = aligni0fitp(datacut(imu,t0+(k-1)*dT,t0+T+(k-1)*dT),pos,0);
            plot(res.attkp(1:100:end,end)-res.attkp(1,end), res.attkp(1:100:end,3)/glv.deg); hold on;
        elseif mth==3
            [att,attk] = alignsb(datacut(imu,t0+(k-1)*dT,t0+T+(k-1)*dT),pos,[],0);
            plot(attk(1:10:end,end)-t0-(k-1)*dT, attk(1:10:end,3)/glv.deg); hold on;
        end
    end
    xygo('y');  [s3,m3,t3] = plotstd(3,2);  legend('costant gyro')
    m = mean([m1(end);m2(end);m3(end)]);
    s = max([s1(end);s2(end);s3(end)]);
    ylimall(m-10*s,m+10*s);
    % subplot(131), ylim(m1(end)+10*[-s,s]);
    % subplot(132), ylim(m2(end)+10*[-s,s]);
    % subplot(133), ylim(m3(end)+10*[-s,s]); 

    if exist('wedb','var')
        wN = glv.wie*cos(pos(1));
        subplot(132); plot(t2, wedb(1)*glv.dpsh./sqrt(t2)/wN/glv.deg*3, 'LineWidth',2);
        subplot(133); plot(t3, wedb(2)*4*glv.ugpsHz/glv.g0./t3.^1.5/wN/glv.deg*3, 'LineWidth',2);
    end
