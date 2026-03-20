function [att, att0, res, phi, vnerr] = alignsar(av, pos, isppm, isfig)
% Single-axis rotation SIMU alignment from AV from open-loop INS update.
%
% Prototype: [att, attk, eb, db] = alignsb(imu, pos, yaw0, isfig)
% Inputs: imu - SIMU data
%         pos - initial position
%         ywo0 - initial yaw
%         isfig - figure flag
% Outputs: att, att0 - attitude align results Euler angles
%          res, phi - 
%
% Example
%   [att, att0] = aligni0(datacut(imu,t0,t1),pos);
%   avp = inspure(datacut(imu,t1,t2),[att;pos], 'O');
%   [att1, att01] = alignsar(datacut(avp,t1,t2), pos, 0, 1);
%
% See also  alignsars, alignvn, aligni0, alignsbtp, insupdate.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/08/2025
global glv
    if nargin<4, isfig=1; end
    if nargin<3, isppm=1; end
    if nargin<2, pos=av(1,7:9)'; end
    av=av(:,[1:6,end]);
    ts = mean(diff(av(:,end)));  fs = fix(1/ts);
    dyaw = diff(av(:,3));
    idx = dyaw>pi;  dyaw(idx)=dyaw(idx)-2*pi;    idx = dyaw<-pi;  dyaw(idx)=dyaw(idx)+2*pi;
    yaw = cumsum([av(1,3);dyaw]);
    wz = mean(dyaw)/ts;  abswz = abs(wz);
    [wnie,g,gn,wN,wU] = wnieg(pos);  gw = g/wz;  gw2 = gw/wz;
    if yaw(1)*yaw(end)>0  % if no cross zero-yaw
        if yaw(1)>0, yaw=yaw-2*pi; else, yaw=yaw+2*pi; end
    end
    i0 = find(yaw(1:end-1).*yaw(2:end)<0,1);
    pch0=av(i0,1);  rll0=av(i0,2);   t0=av(i0,end);
    t = av(:,end)-t0;  swt = sin(av(:,3));  cwt = cos(av(:,3));  t2=t.^2; t3=t.^3;

    res = [];  i1 = i0;
    for k=1:10000
        i10=i1;  i1 = i0+fix(2*pi/abswz*fs*k);
        if i1>length(t), i1=i10; break; end
        I1 = ones(i1-i0+1, 1);
        He = [I1, -g*t(i0:i1), swt(i0:i1),  cwt(i0:i1), -1/2*g*t2(i0:i1)];
        xe = lscov(He, av(i0:i1,4));
        Hn = [I1,  g*t(i0:i1), swt(i0:i1), -cwt(i0:i1), -1/2*g*t2(i0:i1), 1/6*g*wN*t3(i0:i1)];
        if isppm==1
            xn = lscov(Hn, av(i0:i1,5));
        else
            xn = lscov(Hn(:,1:5), av(i0:i1,5));  xn(6)=0;
        end
        Hu = [I1, t(i0:i1)];
        xu = lscov(Hu, av(i0:i1,6));
        phiEe = xn(2)+xu(2)*pch0/g;  phiNe = xe(2)+xu(2)*rll0/g;  phiUe = (xn(5)-xn(6)*rll0+wU*phiNe)/wN; % 20260215
        res(k,:) = [[phiEe,phiNe,phiUe], xn(6)/wz, xe(3:4)'*wz,xu(2), (i1-i0)*ts];
        vnerr = [av(i0:i1,4:6)-[He*xe,Hn*xn,Hu*xu], av(i0:i1,end)];
    end
    phiE0 = res(end,1);  phiN0 = res(end,2); phiU0 = res(end,3);  dkgzzwz=xn(6);
    phi = [phiE0-(wN*phiU0+dkgzzwz*rll0)*t+1/2*wN*dkgzzwz*t2, phiN0+dkgzzwz*pch0*t, phiU0-dkgzzwz*t, t+t0];
    att = [adelphi(av(end,1:3)',phi(end,1:3)'); av(end,end)];  att0 = [adelphi(av(1,1:3)',phi(1,1:3)'); av(1,end)];
    if isfig==1
        insplot(av,'av');
        subplot(221), plot(t(i0)+t0, av(i0,1:2)/glv.deg, 'mO');  ptitle('t0',t0, '\theta_0/\gamma_0(\circ)',[pch0,rll0]/glv.deg);
        subplot(223), plot(t(i0)+t0, av(i0,3)/glv.deg, 'mO');
           ptitle('wz',wz/glv.dps, 'dKgzz',res(end,4)/glv.ppm);
           % title(['\omega_z=', sprintf('%.2f',wz/glv.dps),'(\circ/s), \deltaKgzz=',sprintf('%.2f',res(end,4)/glv.ppm),'(ppm)']);
        subplot(222), plot(t(i0:i1)+t0, [He*xe, Hn*xn],'linewidth',1);  plot(t(i0)+t0, [He(1,:)*xe, Hn(1,:)*xn], 'mO');
            title(['\phi_0=',sprintf('%.3f  ',res(end,1:3)/glv.min),'(\prime)']);
        subplot(224), plot(t(i0:i1)+t0, Hu*xu,'linewidth',1);  plot(t(i0)+t0, Hu(1,:)*xu, 'mO');
            title(['\nabla^b=', sprintf('%.2f  ',res(end,5:7)/glv.ug),'(ug) [rx,ry=',sprintf('%.2f ',res(end,5:6)/wz^2*100),'(cm)]']);
     %   myfig, ve = interp1(av(:,end),av(:,4),t(i0:i1)+t0); plot(t(i0:i1)+t0, [ve,He*xe,200*(smooth(ve,2)-He*xe)]); xygo('dve');
     %   myfig, ven = interp1(av(:,end),av(:,4:5),t(i0:i1)+t0); plot(t(i0:i1)+t0, smoothn(ven,20)-[He*xe, Hn*xn]); xygo('dv');
        fst=3;
        if size(res,1)>fst
            myfig
            subplot(321), plot(res(fst:end,end),res(fst:end,1)/glv.sec), xygo('phiE'); ptitle('\phi_0/(\prime)', res(end,1:3)/glv.min);
            subplot(323), plot(res(fst:end,end),res(fst:end,2)/glv.sec), xygo('phiN');
            subplot(322), plot(res(fst:end,end),res(fst:end,3)/glv.min), xygo('phiU');
            subplot(324), plot(res(fst:end,end),res(fst:end,4)/glv.ppm), xygo('dKgzz');
            subplot(325), plot(phi(:,end),phi(:,1:2)/glv.sec), xygo('phiEN'); ptitle('\phi_{end}/(\prime)', phi(end,1:3)/glv.min);
            subplot(326), plot(phi(:,end),phi(:,3)/glv.min), xygo('phiU');
        end
    end

