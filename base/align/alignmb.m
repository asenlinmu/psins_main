function [att, att0, attk] = alignmb(imu, vnGPS, isfig)
% SINS coarse align on high-moving base aided by GPS velocity. It must
% undertake big horizental acceleration.
%
% Prototype: [att, att0, attk] = alignmb(imu, vnGPS, isfig)
% Inputs: imu - SIMU data
%         vnGPS - GPS velocity array
%         isfig - figure flag
% Outputs: att, att0 - end/start attitude align results (Euler angles)
%          attk - attitude array
%
% See also  dv2atti, alignvn, aligncmps, aligni0, insupdate.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/07/2016, 09/06/2026
    if nargin<3, isfig=1; end
    t0 = max(imu(1,end),vnGPS(1,end)); t1 = min(imu(end,end),vnGPS(end,end));
    imu = cut(imu,t0,t1);  vnGPS = cut(vnGPS(:,[1:3,end]),t0,t1);
    qb0b = [1;0;0;0];  vb0 = [0;0;0];
    res = zeros(length(vnGPS), 7);  kk = 1;
    for k=1:length(imu)
        vb0 = vb0 + qmulv(qb0b,imu(k,4:6)');
        qb0b = qupdt(qb0b, imu(k,1:3)');
        if imu(k,end)>=vnGPS(kk,end)
            res(kk,:) = [vb0', vnGPS(kk,:)];  kk=kk+1;
            if kk>size(vnGPS,1), break; end
        end
    end
    res(kk:end,:) = [];
    an = res(:,4:6)-vnGPS(1,1:3);  fb0 = res(:,1:3);
    fn = an;  fn(:,3) = fn(:,3)+9.80*(res(:,end)-t0);
    qnb0 = mv2atti(diff(fn), diff(fb0));  att0 = q2att(qnb0);
    qnb = qmul(qnb0,qb0b);  att = q2att(qnb);
    if nargout>2
        attk=imu(:,4:7);
        for k=1:length(imu)
            qnb0 = qupdt(qnb0, imu(k,1:3)');
            attk(k,1:3) = q2att(qnb0);
        end
    end
    if isfig==1
        global glv
        msplot(211, vnGPS(:,end), vnGPS(:,1:3)), xygo('V');  ptitle('att0',att0/glv.deg,'att',att/glv.deg);
        msplot(212, res(:,end), res(:,4:6)-vnGPS(1,1:3));
        fn = res(:,1:3)*a2mat(att0)';  fn(:,3) = fn(:,3)-9.80*(res(:,end)-t0);
        hold on; plot(res(:,end), fn, '--'); xygo('dvn_{GPS} & Integral(Cnb0*fb0+gn)');
    end
    

