function roll = vn2roll(vn, isfig)
% trans velocity to roll angle under contition of coordinate turn.
%
% Prototype: roll = vn2roll(vn, isfig)
% Inputs: vn - vel array
%         isfig - figure flag
% Output: roll - roll angle
%
% See also  vn2att, pp2vn, aoa, aos.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/12/2024
global glv
    if nargin<2, isfig=0; end
    vn(:,3) = 0;  ts = mean(diff(vn(:,end)));
    an = vn;
    an(2:end-1,1:3) = (vn(3:end,1:3)-vn(1:end-2,1:3))/2/ts;
    s = vn(1:end-2,1).*vn(3:end,2)-vn(1:end-2,2).*vn(3:end,1);  s=s([1,1:end,end]);
    idx1=s<-1; idx_1=s>1; idx0=~(idx1|idx_1);
    s(idx1)=1; s(idx0)=0; s(idx_1)=-1;  % sign, turn left or right
    an(1,1:3) = an(2,1:3); an(end,1:3) = an(end-1,1:3); 
    u = vnormlz(vn(:,1:3));
    at = dot(u, an(:,1:3), 2);  % tangent
    ac = an(:,1:3)-[at.*an(:,1),at.*an(:,2),at.*an(:,3)];  % normal/center
    ac(isnan(ac))=0;
    roll = [atan(normv(ac)/glv.g0).*s, vn(:,end)];
    if isfig==1
        myfig,
        subplot(211), plot(vn(:,end), vn(:,1:3)), xygo('V');
        subplot(212), plot(roll(:,end), roll(:,1)/glv.deg), xygo('r');
    end