function  avp = ap2avp_ecef(ap, ts)
% Cubic spline interpolation of ap to generate avp with sampling time ts,
% where the velocity is the differentiation of position.

% Prototype: avp = ap2avp_ecef(ap, ts)
% Inputs: ap = [attECEF, pe, t]
%         ts - sampling time interval
% Output: avp = [attECEF, ve, pe, t]
%
% See also  avp2imu_ecef, ap2avp, ap2imu, apmove, apscale, trjsimu, insupdate.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/11/2024
    if size(ap,2)>9, ap=ap(:,[1:3,7:10]); end  % avp->ap
    ts0 = ap(2,7)-ap(1,7);
    if nargin<2,  ts = ts0;  end
    t = (ap(1,7):ts:ap(end,7))';
    qeb = a2quaBatch(ap(:,1:3));
    [rv, qeb0] = q2rvBatch(qeb);  ap(:,1:3) = [[0,0,0];rv];
    avp = zeros(length(t), 10);
    for k=1:3  % rv spline interpolation
        avp(:,k) = spline(ap(:,7), ap(:,k)*ts/ts0, t);
    end
    qeb = qeb0;
    for k=1:length(avp)  % att re-calculate
        qeb = qmul(qeb,rv2q(avp(k,1:3)'));
        avp(k,1:3) = q2att(qeb)';
    end
    for k=1:3
        pp = spline(ap(:,7), ap(:,3+k));
        avp(:,6+k) = ppval(pp, t); % position interpolation
        dpp = pp;
        for kk=1:pp.pieces
            dpp.coefs(kk,:) = [0, [3,2,1].*pp.coefs(kk,1:3)]; % coef differentiation
        end
        avp(:,3+k) = ppval(dpp, t); % velocity interpolation [vx,vy,vz]
    end
    avp(:,10) = t;

