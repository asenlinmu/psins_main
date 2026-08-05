function avp = avpdelxkBatch(avp0, xk)
% avp del KF estimated state xk with Batch processing.
%
% Prototype: avp = avpdelxkBatch(avp0, xk)
% Inputs: avp0 - avp0=[att0;vn0;pos0]
%         xk - KF estimated state xk
% Output: avp - modified avp
% 
% See also  avpadderr, qaddphi, avpcmp, avpinterp.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/06/2026
    t1 = max(avp0(1,end),xk(1,end));  t2 = min(avp0(end,end),xk(end,end));
    avp = cut(avp0,t1,t2);  xk = interp1n(xk, avp(:,end), 'spline');
    qnb = a2quaBatch(avp(:,1:3));
%    qnn = rv2qBatch(xk(:,1:3));
    nm = normv(xk(:,1:3));
    idx0 = nm>1.0e-20;   f=ones(size(nm));  f(idx0)=sin(nm(idx0)/2)./nm(idx0);
    qnn = [cos(nm/2), f.*xk(:,1), f.*xk(:,2), f.*xk(:,3)];
    qnb = qmulBatch(qnn, qnb);
    avp(:,1:3) = q2attBatch(qnb);
    n = size(avp,2)-1;
    avp(:,4:n) = avp(:,4:n)-xk(:,4:n);
