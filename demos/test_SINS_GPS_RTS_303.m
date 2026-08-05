% SINS/GPS intergrated navigation & RTS simulation.
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS_trj, test_SINS_GPS_153, test_RTS_simple_example.
% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/06/2026
glvs
psinstypedef(303);
trj = trjfile('trj10ms.mat');
% initial settings
[nn, ts, nts] = nnts(2, trj.ts);
imuerr = imuerrset(0.03, 100, 0.001, 5, 0,0,0,0, 100,100,100,100);
imu = imuadderr(trj.imu, imuerr);
davp0 = avperrset([0.5;-0.5;20], 0.1, [1;1;3]);
ins = insinit(avpadderr(trj.avp0,davp0), ts);
% KF filter
rk = poserrset([1;1;3]);
kf = kfinit(ins, davp0, imuerr, rk);
kf.Pmin = [avperrset(0.01,1e-4,0.001); gabias(1e-3, [1,5]); dkg9(3,2); dka6(3,2)].^2;  kf.pconstrain=1;
len = length(imu); [avp, xkpk] = prealloc(fix(len/nn), 10, 2*kf.n+1);
timebar(nn, len, '30-state SINS/GPS & RTS Simulation.'); 
ki = 1;
%
global grts;
RTSProcessing(kf.n, length(avp)+1);
for k=1:nn:len-nn+1
    k1 = k+nn-1;  
    wvm = imu(k:k1,1:6);  t = imu(k1,end);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);
    if mod(t,1)==0
        posGPS = trj.avp(k1,7:9)' + davp0(7:9).*randn(3,1);
        kf = kfupdate(kf, ins.pos-posGPS, 'M');
        avp(ki,:) = [ins.avp', t];
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';  ki = ki+1;
    end
    RTSProcessing(kf, t);  % NOTE: must call RTS after measurement & before feedback
    if mod(t,1)==0
        [kf, ins] = kffeedback(kf, ins, 1, 'avp');
    end
    timebar;
end
avp(ki:end,:) = [];  xkpk(ki:end,:) = []; 
% % show results
% avperr = avpcmpplot(trj.avp, avp);
% kfplot(xkpk, avperr, imuerr);
RTSProcessing();
avps = avpdelxkBatch(avp, grts.Xsk);
avpcmpplot(trj.avp, avps);
return;

myfig,
subplot(211), plot(xkpk(:,end), xkpk(:,10:12)/glv.dph, ...
    grts.Xsk(:,end), grts.Xsk(:,10:12)/glv.dph); xygo('eb');
subplot(212), plot(xkpk(:,end), xkpk(:,13:15)/glv.ug, ...
    grts.Xsk(:,end), grts.Xsk(:,13:15)/glv.ug); xygo('db');
