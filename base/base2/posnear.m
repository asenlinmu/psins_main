function t = posnear(pos, posi, isfig)
% Find the nearest position's time tag.
%
% Prototype: t = posnear(pos, posi)
% Inputs: pos - [lat,lon,hgt,t] pos array
%         posi - pos to find
%         isfig - figure flag
% Output: t - pos time tag
% 
% See also  pos2dxyz, getat.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/11/2025
    if nargin<3, isfig=0; end
    [m,n] = size(posi);
    if n==1
        dxyz = pos2dxyz(pos,posi);
        [d2, idx] = min(dxyz(:,1).^2+dxyz(:,2).^2);
        t = pos(idx,end);
        if isfig==1
            myfig,
            subplot(221), plot(dxyz(:,end), dxyz(:,1:2), '-', t,0,'o'); xygo('X|Y / m')
            subplot(223), plot(dxyz(:,end), dxyz(:,3), '-', t,0,'o'); xygo('Z / m')
            subplot(2,2,[2,4]), plot(dxyz(:,1),dxyz(:,2), '-', 0,0,'o');  xygo('E', 'N');
        end
        return;
    end
    dxyz = pos2dxyz(pos);
    dxyzi = pos2dxyz(posi,pos(1,end-3:end-1)');
    t = zeros(m,1);
    for k=1:m
        [d2, idx] = min((dxyz(:,1)-dxyzi(k,1)).^2+(dxyz(:,2)-dxyzi(k,2)).^2);
        t(k) = pos(idx,end);
    end
    if isfig==1
        myfig,
        subplot(221), plot(dxyz(:,end), dxyz(:,1:2), '-', t,dxyzi(:,1:2),'o'); xygo('X|Y / m')
        subplot(223), plot(dxyz(:,end), dxyz(:,3), '-', t,dxyzi(:,3),'o'); xygo('Z / m')
        subplot(2,2,[2,4]), plot(dxyz(:,1),dxyz(:,2), '-', dxyzi(:,1),dxyzi(:,2),'o');  xygo('E', 'N');
    end
