function [avpr, xkpkr, zkrkr, skr, insr, kfr] = sinsgpsrvs(imu, gps, ins1, davp, imuerr, lever, dT, rk, Pmin, Rmin, fbstr, isfig)
% 19-state SINS/GNSS reverse integrated navigation Kalman filter.
% 
% Example
%   [avp1, xkpk1, zkrk1, sk1, ins1, kf1] = sinsgps(cut(imu,152,300), gps, [att00;pos0], ...
%        avperr, imuerr, [0.420;0.584;0.342;1;1], [0.00;0], vperrset(0.1,0.05), Pmin, Rmin*0, 'avpedL');
%   [avpr, xkpkr, zkrkr, skr, insr, kfr] = sinsgpsrvs(cut(imu,152,300), gps, ins1, ...
%        avperr, imuerr, [0.420;0.584;0.342;1;1], [0.00;0], vperrset(0.1,0.05), Pmin, Rmin*0, 'avpedL');
%   t = union(avp1(:,end),avpr(:,end));
%   rf = interp1n(avp1(:,[1:9,end]),t);  rr = interp1n(avpr(:,[1:9,end]),t);
%   xkpk11 = interp1n(xkpk1(:,[1:9,20:28,end]),t);  xkpkr1 = interp1n(xkpkr(:,[1:9,20:28,end]),t);
%   psf = POSFusion(rf, xkpk11, rr, xkpkr1);
%   avpcmpplot(gps, psf.rf(:,[4:9,end]), 'vp');
%
% See also  sinsgps, POSFusion, POSProcessing, inspurervs.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/06/2026
global glv
    if ~exist('isfig','var'), isfig=1; end
    glv.wie = -glv.wie;
    ins = insinit([ins1.att; -ins1.vn; ins1.pos], diff(imu(2:3,end)));  ins.eb=-ins1.eb; ins.db=ins1.db;
    if mod(length(imu),2)==0, imu(end,:)=[]; end
    t1 = fix(ceil(imu(end,end)));
    imu = flipud(imu); imu(:,end)=t1-imu(:,end);  imu(:,1:3)=-imu(:,1:3);  % imuplot(imu);
    gps = flipud(gps); gps(:,end)=t1-gps(:,end); if size(gps,2)>6, gps(:,1:3)=-gps(:,1:3); end  % gpsplot(gps);

    [avpr, xkpkr, zkrkr, skr, insr, kfr] = sinsgps(imu, gps, ins, davp, imuerr, lever, dT, rk, Pmin, Rmin, fbstr, 0);

    rvsidx = [4:6,10:12,19];
    avpr = flipud(avpr); avpr(:,end)=t1-avpr(:,end); avpr(:,rvsidx)=-avpr(:,rvsidx);
    xkpkr = flipud(xkpkr); xkpkr(:,end)=t1-xkpkr(:,end); xkpkr(:,rvsidx)=-xkpkr(:,rvsidx);
    zkrkr = flipud(zkrkr); zkrkr(:,end)=t1-zkrkr(:,end); if size(zkrkr,2)>12, zkrkr(:,1:3)=-zkrkr(:,1:3); end
    skr = flipud(skr); skr(:,end)=t1-skr(:,end);
    kfr.xk(rvsidx) = -kfr.xk(rvsidx);
    glv.wie = -glv.wie;

    if isfig==1
        insplot(avpr);
        kfplot(xkpkr);
        rvpplot(zkrkr);
        stateplot(skr,length(kfr.Rk)/3);
    end

