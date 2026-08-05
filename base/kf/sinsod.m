function [avpo, xkpk, zkrk, sk, ins, kf] = sinsod(imuod, avp0, davp, imuerr, inst, intT, rk, Pmin, Rmin, fbstr, isfig)
% 18-state SINS/OD integrated navigation Kalman filter.
% The 18-state includes:
%       [phi(3); dvn(3); dpos(3); eb(3); db(3); dpch/dKod/dyaw(3)]
%
% Prototype: [avpo, xkpk, zkrk, sk, ins, kf] = sinsod(imuod, avp0, davp, imuerr, inst, intT, rk, Pmin, Rmin, fbstr, isfig)
% Inputs: imuod - IMU/OD array [wm, vm, dS, t]
%         avp0 - init [Att; Vn, Pos]
%         davp - AVP array for P0 setting
%         imuerr - set by function 'imuerrset', for P0 and Qk setting
%         inst - OD installation parameters & scale factor, [pch;Kod;yaw]
%         intT - velocity integral interval for measuremet
%         rk - velocity measurement noise in m/s
%         Pmin - Pmin setting, Pmin<=0 for no Pmin constrain
%         Rmin - Rmin setting, Rmin<=0 for no adaptive KF, Rmin=0~1 scale for adaptive KF and Rmin = Rk*Rmin
%         fbstr - KF feedback string from any combination of 'avpedLT'
%         isfig - figure flag
%
% Example:
% avp0 = [att; pos0];
% davp = avperrset([60;300], 1, 100);
% imuerr = imuerrset(0.03, 100, 0.001, 1);
% Pmin = [avperrset([0.1,1],0.001,0.01); gabias(0.001, [10,30]); [0.1*glv.min;0.0001;0.1*glv.min]].^2;
% Rmin = verrset(0.01).^2;
% [avpo, xkpk, zkrk, sk, ins, kf] = sinsod(imuod, avp0, davp, imuerr, [0;1;0], 1.0, 0.1, Pmin*0, Rmin*0, avp, 1);  
%
% See also  sinsgps, drupdate, drinit.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/04/2026
global glv
    [nn, ts, nts] = nnts(2, diff(imuod(2:3,end)));
    if ~exist('rk', 'var'), rk=0.1; end
    if length(rk)==1, rk=[rk;rk;rk]; end
    if ~exist('intT', 'var'), intT=0.5; end
    if ~exist('inst', 'var'), inst=[0;1;0]; end
    if ~exist('imuerr', 'var'), imuerr = imuerrset(0.05, 500, 0.001, [10;10;100]); end
    if ~exist('davp', 'var'), davp = avperrset([10;300], 1, [10;30]); end
    ins = insinit(avp0, ts);
    kf = [];
    kf.Qt = diag([imuerr.web; imuerr.wdb])^2;
    kf.Rk = diag(rk)^2;
    kf.Pxk = diag([davp; imuerr.eb; imuerr.db; [1*glv.deg;0.001;1*glv.deg]])^2;
    kf.Hk = [zeros(3,3), eye(3), zeros(3,12)];
    kf = kfinit0(kf, nts);
    if exist('Pmin', 'var'),
        if sum(Pmin)<=0, kf.pconstrain=0;
        else kf.Pmin = Pmin; kf.pconstrain = 1; end
    end
    kf.adaptive = 1;
    if exist('Rmin', 'var'), 
        if sum(Rmin)<=0, kf.adaptive=0; end
        if kf.adaptive==1,
            if length(Rmin)==1, kf.Rmin = kf.Rk*Rmin;
            else kf.Rmin = diag(Rmin); end
        end
    end
    if exist('fbstr', 'var'), kf.fbstr=fbstr; end
    kf.xtau = [ [1;1;1]; [1;1;1]; [1;1;1]; [1;1;1]; [1;1;1]; [1;1;1]]*1;
    len = length(imuod); [avpo, xkpk, zkrk, sk] = prealloc(fix(len/nn), 19, 2*kf.n+1, 2*kf.m+1, 2);
    ivn = [0;0;0];  dSn = [0;0;0];  Cnb = zeros(3);  dT = 0;
    Cbo = a2mat(-[inst(1);0;inst(3)]);  prj = Cbo*[0;inst(2);0];
    timebar(nn, len, '18-state SINS/OD processing.'); ki = 1; kiz = 1;
    % kfs = kfstat([], kf);
    for k=1:nn:len-nn+1
        k1 = k+nn-1; 
        wvm = imuod(k:k1,1:6); dSn=dSn+ins.Cnb*prj*sum(imuod(k:k1,7)); Cnb=Cnb+ins.Cnb*nts; dT=dT+nts; t = imuod(k1,end);
        ins = insupdate(ins, wvm);  ivn=ivn+ins.vn*nts;
        [~, kf.Phikk_1] = etm(ins, kf.n, nts);
        kf = kfupdate(kf);      if exist('kfs','var'), kfs = kfstat(kfs, kf, 'T'); end
        if norm(ins.wbar)>3*glv.dps || abs(ins.wbar(3))>1*glv.dps
            dT = 0;  ivn=[0;0;0];  dSn = [0;0;0];  Cnb = zeros(3);
        end
        if dT>intT
            ovn = dSn/dT;  Cnb = Cnb/dT;
            MvkD = norm(ovn)*[-Cnb(:,3),Cnb(:,[2,1])];
            kf.Hk(:,[1:3,16:18]) = [-askew(ovn),-MvkD];
            zk = ivn/dT-dSn/dT;
            kf = kfupdate(kf, zk, 'M');     if exist('kfs','var'), kfs = kfstat(kfs, kf, 'M'); end
            zkrk(kiz,:) = [zk; diag(kf.Rk); t];  kiz = kiz+1;
            dT = 0;  ivn=[0;0;0];  dSn = [0;0;0];  Cnb = zeros(3);
        end
        [kf, ins] = kffeedback(kf, ins, nts);
        if 1  % OD-Kappa feedback
            afa=0.1;  inst([1,3]) = inst([1,3]) - afa*kf.xk([16,18]);  kf.xk([16,18]) = (1-afa)*kf.xk([16,18]);
                      inst(2) = inst(2) * (1-afa*kf.xk(17));  kf.xk(17) = (1-afa)*kf.xk(17);
            Cbo = a2mat(-[inst(1);0;inst(3)]);  prj = Cbo*[0;inst(2);0];
        end
        avpo(ki,:) = [ins.att; ins.vn;  ins.pos;  ins.eb; ins.db; inst; t]';
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';
        sk(ki,:) = [kf.measlog, t];  kf.measlog=0;  ki = ki+1; 
        timebar;
    end
    avpo(ki:end,:) = []; xkpk(ki:end,:) = [];  zkrk(kiz:end,:) = []; sk(ki:end,:) = [];
    if ~exist('isfig', 'var'), isfig=1; end
    if isfig==1
        insplot(avpo,'avpedod');
        inserrplot(xkpk(:,[1:18,end]),'avpedod');
        inserrplot([sqrt(xkpk(:,19:end-1)),xkpk(:,end)],'avpedod');
        myfig
        subplot(211), plot(zkrk(:,end), zkrk(:,1:3)); xygo('dv');
        subplot(212), plot(zkrk(:,end), sqrt(zkrk(:,4:6))); xygo('dv');
        stateplot(sk,1);
        if exist('kfs','var'), kfs = kfstat(kfs); end
    end
