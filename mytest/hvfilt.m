function [avp, xkpk] = hvfilt(imu, pos0, yaw0, imuerr, rk, davp, Pmin)
global glv
    if nargin<7, Pmin=[avperrset(0.1,0.001,0.01); gabias(0.1, 10)].^2; end
    if nargin<6, davp=avperrset([600;600;0], 10, 10); end
    if nargin<5, rk=[1;1;1]*10; end
    if nargin<4, imuerr=imuerrset([100;100;0], [0;0;1000], [1;1;0], [0;0;100]); end
    if nargin<3, yaw0=0; end
    if nargin<2, pos0=glv.pos0; end
    if numel(pos0)==1, pos0=[pos0;0;0]; end
    if rk(1)>1000/glv.Re, rk=poserrset(rk); end  % if in meter
    [nn, ts, nts] = nnts(2, diff(imu(1:2,end)));
    ins = insinit([0;0;yaw0; 0;0;0; pos0], ts);
    psinstypedef(153);
    kf = kfinit(ins, davp, imuerr, rk);
    kf.Pmin = Pmin;  kf.pconstrain=1;  kf.xtau=ones(size(kf.xk))*5;
    len = length(imu);  [avp, xkpk] = prealloc(fix(len/nn), 16, 2*kf.n+1);
    timebar(nn, len, 'Heave calculation by 15-state SINS/GPS KF method.'); 
    ki = 1;  tmeas=0;
    for k=1:nn:len-nn+1
        k1 = k+nn-1;  
        wvm = imu(k:k1,1:6);  t = imu(k1,end);  tmeas=tmeas+nts;
        ins = insupdate(ins, wvm);
        kf.Phikk_1 = kffk(ins);
        kf = kfupdate(kf);
        if tmeas>=0.1
            kf = kfupdate(kf, ins.pos-pos0, 'M');  tmeas=0;
        end
        [kf, ins] = kffeedback(kf, ins, max(nts,1/(k*nts)), 'avped');
        avp(ki,:) = [ins.avp; ins.eb; ins.db; t]';
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';  ki = ki+1;
        timebar;
    end
    avp(ki:end,:) = [];  xkpk(ki:end,:) = []; 
    insplot(avp);
    kfplot(xkpk);