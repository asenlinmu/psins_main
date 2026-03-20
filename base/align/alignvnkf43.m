function [av, xkpk] = alignvnkf43(imu, avp0, imuerr, phi0, wvn)
% Initial alignment by KF and using vn as measurement. To test which state
% is observable.
%
% Prototype: [av, xkpk] = alignvnkf43(imu, avp0, imuerr, phi0, wvn)
% Inputs: imu - SIMU data
%         avp0 - intital [att0,pos0]
%         imuerr - IMU error struct, 0-element for no corresponding state esitmated
%         phi0 - initial misalignment angles
%         wvn - velocity measure noise
% Outputs: av - att & vn out:
%          xkpk - KF xk & Pk
%
% See also  alignvn, sysclbt, imuerrset, alignscat.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/10/2025
global glv
    if nargin<5, wvn=[1;1;1]*0.001; end
    if nargin<4, phi0=[0.1;0.1;1]*glv.deg; end
    if nargin<3, imuerr=imuerrset(0.01,100,0.001,10, 0.001,1000,10,1000, 10,10,10,10, 10, 10, 10); end
    qnb = a2qua(avp0(1:3)); vn=zeros(3,1); pos0 = avp0(end-2:end);
    [wnie,g,gn] = wnieg(pos0);
    [nn,ts,nts] = nnts(2, diff(imu(1:2,end)));
    len = length(imu);
    dotwf = imudot(imu, 5.0);
    kf = alnkfinit(nts,imuerr,phi0,wvn);
    t1s = 0;
    av = zeros(fix(len*ts),7); xkpk = zeros(fix(len*ts), kf.n*2+1);  kk = 1;
    timebar(nn, len, sprintf('Initial alignment KF 43-states'));
    for k=1:nn:len-nn
        k1 = k+nn-1;
        wm = imu(k:k1,1:3); vm = imu(k:k1,4:6); t = imu(k1,end); dwb = mean(dotwf(k:k1,1:3),1)';
        [phim, dvbm] = cnscl([wm,vm]);
        wb = phim/nts; fb = dvbm/nts;
        SS = lvS(eye(3), wb, dwb);
        fn = qmulv(qnb, fb);
        an = rotv(-wnie*nts/2, fn) + gn;
        vn = vn + an*nts;
        qnb = qupdt2(qnb, phim, wnie*nts);   % insupdate
        t1s = t1s + nts;
        kf.Phikk_1 = eye(kf.n)+getFt(fb, wb, q2mat(qnb), wnie, SS)*nts;
        kf = kfupdate(kf);
        if t1s>(0.2-ts/2)  % kf measurement update every 1 second
            t1s = 0;
            kf = kfupdate(kf, vn);
            qnb = qdelphi(qnb, 0.5*kf.xk(1:3));   kf.xk(1:3) = 0.5*kf.xk(1:3);
        end
        av(kk,:) = [q2att(qnb); vn; t]';
        xkpk(kk,:) = [kf.xk; diag(kf.Pxk); t]'; kk = kk+1;
        timebar;
    end
    av(kk:end,:) = []; xkpk(kk:end,:) = [];
    alnkfplot(av, xkpk);
    
function kf = alnkfinit(ts, ierr, phi0, wvn)
global glv
    kf.Qt = diag([ ierr.web; ierr.wdb; zeros(37,1) ])^2;
    kf.Rk = diag(wvn)^2;
    kf.Pxk = diag([ phi0; [1;1;1]; ierr.eb; ierr.db; ...
        ierr.dKg(:,1); ierr.dKg(:,2); ierr.dKg(:,3); ierr.dKa(:,1); ierr.dKa(:,2); ierr.dKa(:,3); ierr.Ka2; ...
        ierr.rx; ierr.ry; ierr.rz; ierr.dtGA ])^2;
    kf.Hk = [zeros(3),eye(3),zeros(3,37)];
    kf = kfinit0(kf, ts);

function Ft = getFt(fb, wb, Cnb, wnie, SS)   % kffk
    o33 = zeros(3); o31 = zeros(3,1);  %wb=[1;2;3]; fb=[1;2;10];
    wX = askew(wnie); fX = askew(Cnb*fb);
    wx = wb(1); wy = wb(2); wz = wb(3); fx = fb(1); fy = fb(2); fz = fb(3);
    CDf2 = Cnb*diag(fb.^2); CwXf = Cnb*cross(wb,fb);
    %        1   4     7    10    13       16       19       22       25       28       31    34  37  40   43    
    %states: fi  dvn   eb   db    dKg(:,1) dKg(:,2) dKg(:,3) dKa(:,1) dKa(:,2) dKa(:,3) dKa2  rx  ry  rz   tGA
    Ft = [  -wX  o33  -Cnb  o33  -wx*Cnb  -wy*Cnb  -wz*Cnb   o33      o33      o33      o33   o33 o33 o33  o31
             fX  o33   o33  Cnb   o33      o33      o33      fx*Cnb   fy*Cnb   fz*Cnb   CDf2  Cnb*SS       CwXf
             zeros(37,43) ];

function SS = lvS(Cba, wb, dotwb) % Ref: Yan G, Inner lever arm compensation and its test verification for SINS, Journal of Astronautics, 2012.
    U = (Cba')^-1; V1 = Cba(:,1)'; V2 = Cba(:,2)'; V3 = Cba(:,3)';
    Q11 = U(1,1)*V1; Q12 = U(1,2)*V2; Q13 = U(1,3)*V3;
    Q21 = U(2,1)*V1; Q22 = U(2,2)*V2; Q23 = U(2,3)*V3;
    Q31 = U(3,1)*V1; Q32 = U(3,2)*V2; Q33 = U(3,3)*V3;
    W = askew(dotwb)+askew(wb)^2;
    SS = [Q11*W, Q12*W, Q13*W; Q21*W, Q22*W, Q23*W; Q31*W, Q32*W, Q33*W];
    
function alnkfplot(av, xkpk)
global glv
    myfigure
    subplot(211), plot(av(:,end), av(:,1:3)/glv.deg); xygo('att');
    subplot(212), plot(av(:,end), av(:,4:6)); xygo('V');
    plotxk(xkpk(:,[1:43,end]));
    plotxk([sqrt(xkpk(:,[44:end-1])),xkpk(:,end)]);
 
function plotxk(xk)        
global glv
    myfigure
    t = xk(:,end);
    subplot(331), plot(t, xk(:,1:3)/glv.min); xygo('phi');
    subplot(332), plot(t, xk(:,4:6)); xygo('dv')
    subplot(333), plot(t, xk(:,7:9)/glv.dph); xygo('eb');
    subplot(334), plot(t, xk(:,10:12)/glv.ug); xygo('db');
    subplot(335), plot(t, xk(:,13:4:21)/glv.ppm); xygo('dKii');
        hold on,  plot(t, xk(:,22:4:30)/glv.ppm, '--');
    subplot(336), plot(t, xk(:,[14:16,18:20])/glv.sec); xygo('dKij');
    	hold on,  plot(t, xk(:,[23:25,27:29])/glv.sec, '--');
    subplot(337), plot(t, xk(:,31:33)/glv.ugpg2); xygo('Ka2');
    subplot(338), plot(t, xk(:,34:36)); xygo('lever arm / m');
    	hold on,  plot(t, xk(:,37:39),'--'); plot(t, xk(:,40:42), '-.');
    subplot(339), plot(t, xk(:,43)); xygo('\tau_{GA} / s');
