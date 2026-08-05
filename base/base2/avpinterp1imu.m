function avp = avpinterp1imu(avp, imu)
% AVP interpolation by IMU. 
%
% Prototype: avp = avpinterp1imu(avp, imu)
% Inputs: avp - input avp
%         imu - input imu
% Output: avp - interpolated avp
%
% See also  attinterp, avpinterp1, insupdate, ethupdate, RTSProcessing.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/06/2026
    eth = earth(avp(1,7:9)');
    [~, iavp] = combinedata(imu, avp(:,[1:9,end]));
    nm = normv(iavp(:,7:15));
    idx0 = find(nm>eps,1,'first');  nm(1:idx0-1)=[];  iavp(1:idx0-1,:)=[];
    timebar(1,length(iavp),'AVP interpolation by IMU.');
    for k=1:length(iavp)
        if nm(k)>0
            qnb=a2qua(iavp(k,7:9)'); vn_1=iavp(k,10:12)'; pos=iavp(k,13:15)';
        else
            wm = iavp(k,1:3)';  vm = iavp(k,4:6)';  ts = iavp(k,end)-iavp(k-1,end);
            vn = vn_1+qmulv(qnb,vm+cros(wm, vm)); vn(3) = vn(3)-eth.g*ts;
            vn01 = (vn_1+vn)/2;  vn_1 = vn;
            pos = pos+[vn01(2)/eth.RMh; vn01(1)/eth.clRNh; vn01(3)]*ts;
            qnb = qupdt(qnb,wm);
            iavp(k,7:15) = [q2att(qnb); vn; pos]';
        end
        timebar;
    end
    timebar(-1);
    avp = iavp(:,[7:15,end]);

