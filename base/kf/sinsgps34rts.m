function [avp, xkpk, zkrk, sk, ins, kf] = sinsgps34rts(imu, gps, ins, davp, imuerr, lever, dT, rk, Pmin, Rmin, fbstr, isfig)
% 34-state SINS/GNSS integrated navigation Kalman filter.
% The 34-state includes:
%       [phi(3); dvn(3); dpos(3); eb(3); db(3); lever(3); dT(1); dKg(9); dKa(6)]
% The 3- or 6- measurement is:
%       [dpos(3)] or [dvn(3); dpos(3)]
%
% See also sinsgps, RTSProcessing.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/07/2026
global glv
    [nn, ts, nts] = nnts(2, diff(imu(1:2,end)));
    clmgps = size(gps,2); SatNum = 20; DOP = 1.0;
    if clmgps<=5, gpspos_only=1; pos0=gps(1,1:3)'; else, gpspos_only=0; pos0=gps(1,4:6)'; end 
    if ~exist('rk', 'var'),
        if gpspos_only==1, rk=poserrset([10,30]);
        else, rk=vperrset([0.1;0.3],[10,30]); end
    end
    if ~exist('dT', 'var'), dT = 0.01; end;   if length(dT)==1, dT(2,1)=1; end
    if ~exist('lever', 'var'), lever = rep3(1); else, lever=rep3(lever); end;   if length(lever)==3, lever(4)=1; end;   if length(lever)<5, lever(5)=1; end
    if ~exist('imuerr', 'var'), imuerr = imuerrset(0.05, 500, 0.001, [10;10;100]); end
    if ~exist('davp', 'var'), davp = avperrset([10;300], 1, [10;30]); end
    if ~exist('ins', 'var'), ins=100; end
    if ~isstruct(ins) 
        if length(ins)==1   % sinsgps(imu, gps, T);  T=ins align time
            [~, att0] = aligni0(imu(1:fix(ins/ts),:), pos0);  imu(1:fix(ins/ts),:)=[];  vn0=zeros(3,1);
        elseif length(ins)==3, att0 = ins; vn0=zeros(3,1);  % sinsgps(imu, gps, att0);
        elseif length(ins)==6, att0 = ins(1:3); vn0=zeros(3,1); pos0=ins(4:6); % sinsgps(imu, gps, [att0;pos0]);
        elseif length(ins)==9, att0 = ins(1:3); vn0=zeros(3,1); pos0=ins(7:9); end  % sinsgps(imu, gps, [att0;vn0;pos0]);
        ins = insinit([att0; vn0; pos0], ts); ins.nts=nts;
    end
    ins.lever = lever(1:3)*(1-lever(4));  ins.tDelay = dT(1)*(1-dT(2));
    ins = inslever(ins, -ins.lever);  ins.vn = ins.vnL; ins.pos = ins.posL;
    if ~isempty(glv.dgn), ins.eth = attachdgn(ins.eth, glv.dgn); end
    psinstypedef(346-gpspos_only*3);
    kf = [];
    kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(3,1); imuerr.sqg; imuerr.sqa; zeros(3,1); 0; zeros(15,1)])^2;
    kf.Rk = diag(rk)^2;
    kf.Pxk = diag([davp; imuerr.eb; imuerr.db; lever(1:3)*lever(4); dT(1)*dT(2); imuerr.dKga]*1.0)^2;   % 2021/11/2
    kf.Hk = zeros(length(rk),34);
    kf = kfinit0(kf, nts);
    kf.xk(16:18) = lever(1:3)*lever(4);  kf.xk(19) = dT(1)*dT(2);
    if exist('Pmin', 'var'),
        if sum(Pmin)<=0, kf.pconstrain=0;
        else, kf.Pmin = Pmin; kf.pconstrain = 1;
            if lever(4)==0, kf.Pmin(16:18)=0; end
            if dT(2)==0, kf.Pmin(19)=0; end
        end
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
    kf.xtau = [ [1;1;1]; [1;1;1]; [1;1;1]; [1;1;1]; [1;1;1]; [1;1;1]; 1; zeros(15,1)]*1;
    imugpssyn(imu(:,end), gps(:,end));
    len = length(imu); [avp, xkpk, zkrk, sk] = prealloc(fix(len/nn), 20, 2*kf.n+1, 2*kf.m+1, 2);
    if len<101, return; end  % return kf struct
    timebar(nn, len, '34-state SINS/GNSS simulation.'); ki = 1; kiz = 1;
    rtsT = 0.05;  rtst = 0.0;  Phikk_1 = eye(kf.n);
    RTSProcessing(kf.n, ceil(length(avp)*nts/rtsT));
    for k=1:nn:len-nn+1
        k1 = k+nn-1; 
        wvm = imu(k:k1,1:6); t = imu(k1,end);
        if k>1, ins.ts=(imu(k1,end)-imu(k-1,end))/nn; end % 2026-07-04
        ins = insupdate(ins, wvm);  ins.eth.dgnt=t;
        kf.Phikk_1 = kffk(ins);
        kf = kfupdate(kf);
        [kgps, dt] = imugpssyn(k, k1, 'F');
        ins = inslever(ins); 
        if kgps>0 && (clmgps==5||clmgps==8)  % having SatNum.DOP
            SatNum = gps(kgps,end-1); DOP = (SatNum-fix(SatNum))*1000;
            if SatNum<10||DOP>1.5, kgps=0; end  % disable meas
        end
        if kgps>0
            dtpos=+vn2dpos(ins.eth,ins.vnL,ins.tDelay-dt);
            if gpspos_only==1
                zk = ins.posL+dtpos-gps(kgps,1:3)';
                kf.Hk(:,1:19) = [zeros(3,6), eye(3), zeros(3,6), -ins.MpvCnb,-ins.Mpvvn];
            else
                zk = [ins.vnL+(ins.tDelay-dt)*ins.anbar;ins.posL+dtpos]-gps(kgps,1:6)';
                kf.Hk(:,1:19) = [zeros(6,3), eye(6), zeros(6,6), [-ins.CW,-ins.anbar;-ins.MpvCnb,-ins.Mpvvn]];
            end
            kf = kfupdate(kf, zk, 'M');
            zkrk(kiz,:) = [zk; diag(kf.Rk); t];  kiz = kiz+1;
        end
        rtst = rtst+nts;  Phikk_1=kf.Phikk_1*Phikk_1;
        if rtst>rtsT || kgps>0
            kf.Phikk_1 = Phikk_1(1:9,:);  % NOTE: states 10~34 to be random-constant
            RTSProcessing(kf, t);
            rtst = 0;  Phikk_1 = eye(kf.n);
            [kf, ins] = kffeedback(kf, ins, rtsT);
        end
        if lever(5)==1
            insL = inslever(ins, ins.lever+kf.xk(16:18));
            avp(ki,:) = [ins.att; insL.vnL; insL.posL; ins.eb; ins.db; ins.lever; ins.tDelay; t]';
        else
            avp(ki,:) = [ins.att; ins.vn;  ins.pos;  ins.eb; ins.db; ins.lever; ins.tDelay; t]';
        end
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';  % xkpk(ki,19)=dt;
        sk(ki,:) = [kf.measlog, t];  kf.measlog=0;  ki = ki+1; 
        timebar;
    end
    avp(ki:end,:) = []; xkpk(ki:end,:) = [];  zkrk(kiz:end,:) = []; sk(ki:end,:) = [];
    RTSProcessing();
    if ~exist('isfig', 'var'), isfig=1; end
    if isfig==1
        insplot(avp);
        kfplot(xkpk);
        rvpplot(zkrk);
        stateplot(sk,length(kf.Rk)/3);
    end
