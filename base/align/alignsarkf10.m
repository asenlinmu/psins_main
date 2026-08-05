function [av, xkpk] = alignsarkf10(imu, avp0, imuerr, phi0, wvn)
% 10-state single axis rotation initial alignment by KF and using vn as
% measurement. Usually rotated by z/Up axis and with z-axis gyro scale
% factor estimated.
%
% Prototype: [av, xkpk] = alignsarkf10(imu, avp0, imuerr, phi0, wvn)
% Inputs: imu - SIMU data
%         avp0 - initial [att0,pos0]angles
%         imuerr - IMU error struct, NOTE: [dKgxx,dKgyy,dKgzz] for ... see the code
%         phi0 - initial misalignment 
%         wvn - velocity measure noise
% Outputs: av - att & vn out:
%          xkpk - KF xk & Pk
%
% Example
%   ierr = imuerrset(0.01,1000,0.001,10, 0,0,0,0, [0.1;1;100]);
%   [av, xkpk] = alignsarkf10(imu, [att0;pos0], ierr, [10;10;60]*glv.min, [1;1;1]*0.1);
%
% See also  alignsar, alignvn, alignvnkf43, sysclbt, imuerrset, alignscat.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/06/2026
global glv
    if nargin<6, lvb=[0;0;0]; end
    if nargin<5, wvn=0.01; end
    if length(wvn)==1; wvn=[wvn;wvn;wvn]; end
    if nargin<4, phi0=[0.1;0.1;1]*glv.deg; end
    if nargin<3, imuerr=imuerrset(0.01,100,0.001,10, 0,0,0,0, [0;1;100]); end
    qnb = a2qua(avp0(1:3)); vn=zeros(3,1); pos0 = avp0(end-2:end);
    [wnie,g,gn] = wnieg(pos0);
    [nn,ts,nts] = nnts(2, diff(imu(1:2,end)));
    len = length(imu);  dkgzz=0;
    kf = alnkfinit(nts,imuerr,phi0,wvn);
    t1s = 0;
    av = zeros(len,8); xkpk = zeros(len, kf.n*2+1);  kk = 1;
    timebar(nn, len, sprintf('10-state single axis rotation initial alignment, '));
    afa = imuerr.dKg(1,1)/glv.ppm;  afa1 = 1.0-afa;   % NOTE: dKgxx=feedback coefficience afa
    for k=1:nn:len-nn
        k1 = k+nn-1;
        wm = imu(k:k1,1:3); vm = imu(k:k1,4:6); t = imu(k1,end);
        [phim, dvbm] = cnscl([wm,vm]);  phim(3)=(1-dkgzz)*phim(3);
        wb = phim/nts; fb = dvbm/nts;
        fn = qmulv(qnb, fb);
        an = rotv(-wnie*nts/2, fn) + gn;
        vn = vn + an*nts;
        qnb = qupdt2(qnb, phim, wnie*nts);   % insupdate
        t1s = t1s + nts;
        t1s = t1s + nts;   Cnb = q2mat(qnb);
        kf.Phikk_1 = eye(kf.n)+getFt(fb, wb, Cnb, wnie)*nts;
        kf = kfupdate(kf);
        if t1s>(0.2-ts/2)  % kf measurement update every 1 second
            t1s = 0;
            kf = kfupdate(kf, vn+Cnb*cros(wb,lvb));
            qnb = qdelphi(qnb, afa*kf.xk(1:3));   kf.xk(1:3) = afa1*kf.xk(1:3);
            vn = vn-afa*kf.xk(4:6);  kf.xk(4:6) = afa1*kf.xk(4:6);
            dkgzz = dkgzz+afa*kf.xk(10);  kf.xk(10) = afa1*kf.xk(10);
        end
        av(kk,:) = [q2att(qnb); vn; dkgzz; t]';
        xkpk(kk,:) = [kf.xk; diag(kf.Pxk); t]'; kk = kk+1;
        timebar;
    end
    av(kk:end,:) = []; xkpk(kk:end,:) = [];
    alnkfplot(av, xkpk);
    
function kf = alnkfinit(ts, ierr, phi0, wvn)
global glv
    kf.Qt = diag([ ierr.web; ierr.wdb; [1;1;1]*glv.ug/sqrt(glv.hur); ierr.dKg(2,2)/glv.ppm*glv.ppmpsh ])^2;  % NOTE: dKgyy=random walk of dKgzz
    kf.Rk = diag(wvn)^2;
    kf.Pxk = diag([ phi0; [1;1;1]*1.1; ierr.db; ierr.dKg(3,3) ])^2;
    kf.Hk = [zeros(3),eye(3),zeros(3,4)];
    kf = kfinit0(kf, ts);

function Ft = getFt(fb, wb, Cnb, wnie)   % kffk
    o33 = zeros(3); o31 = zeros(3,1);
    wX = askew(wnie); fX = askew(Cnb*fb); wz = wb(3);
    %        1   4     7    10       
    %states: fi  dvn   db   dKgzz
    Ft = [  -wX  o33   o33  -[0;0;wz]
             fX  o33   Cnb   o31
             zeros(4,10) ];
 
function alnkfplot(av, xkpk)
global glv
    myfigure
    subplot(221), plot(av(:,end), av(:,1:2)/glv.deg); xygo('pr');
    subplot(223), plot(av(:,end), av(:,3)/glv.deg); xygo('y');  ptitle('y',av([1,end],3)/glv.deg);
    subplot(222), plot(av(:,end), av(:,4:6)); xygo('V');
    subplot(224), plot(av(:,end), av(:,7)/glv.ppm); xygo('dKgzz');
    myfigure
    xk = xkpk(:,1:10); t = xkpk(:,end);
    subplot(421), plot(t, xk(:,1:3)/glv.min); xygo('phi');
    subplot(423), plot(t, xk(:,4:6)); xygo('dv')
    subplot(425), plot(t, xk(:,7:9)/glv.ug); xygo('db');
    subplot(427), plot(t, xk(:,10)/glv.ppm); xygo('dKgzz');
    xk = sqrt(xkpk(:,11:end-1));
    subplot(422), plot(t, xk(:,1:3)/glv.min); xygo('phi');
    subplot(424), plot(t, xk(:,4:6)); xygo('dv')
    subplot(426), plot(t, xk(:,7:9)/glv.ug); xygo('db');
    subplot(428), plot(t, xk(:,10)/glv.ppm); xygo('dKgzz');
