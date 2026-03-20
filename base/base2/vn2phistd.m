function phi = vn2phistd(vn, lti, alignT, intervalT, isfig)
% Calculating yaw misalign angle STD from pure SINS velocity error.
%
% Prototype: phi = vn2phistd(vn, lti, alignT, intervalT)
% Inputs: vn - pure SINS velocity error, in most case for static base
%         lti - latitude
%         alignT - align time lenght
%         intervalT - interval time
%         isfig - figure flag
% Output: phi - misalignment angles
%
% See also  vn2phiu, pn2phistd, av2phiu, vn2phi, vn2att, alignscat.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/01/2025
global glv
    if nargin<5, isfig=1; end
    if nargin<3, alignT=300; end
    if nargin<4, intervalT=alignT/5; end
    if intervalT<1, intervalT=1; elseif intervalT>alignT, intervalT=alignT; end
    if length(alignT)>1  % nest call vn2phistd for more than one alignT
        phis=zeros(length(alignT),4); k=1; myfig;
        for T=alignT
            phi = vn2phistd(vn, lti, T, intervalT, 0);
            subplot(321), plot(phi(:,end), phi(:,1)/glv.sec), xygo('phiE');
            subplot(323), plot(phi(:,end), phi(:,2)/glv.sec), xygo('phiN');
            subplot(325), plot(phi(:,end), phi(:,3)/glv.min), xygo('phiU');
            % phis(k,:) = [std(phi(:,1:3)), T]; k=k+1;
            [y,y0] = deltrend(phi,1,0);
            phis(k,:) = [std(y(:,1:3)), T]; k=k+1;
        end
        subplot(321), plot(phi(:,end), y0(:,1)/glv.sec,'linewidth',1.5), xygo('phiE');
        subplot(323), plot(phi(:,end), y0(:,2)/glv.sec,'linewidth',1.5), xygo('phiN');
        subplot(325), plot(phi(:,end), y0(:,3)/glv.min,'linewidth',1.5), xygo('phiU');
        subplot(322); plot(phis(:,end), phis(:,1)/glv.sec, '-o');  xygo('T / s', 'phiE');
        subplot(324); plot(phis(:,end), phis(:,2)/glv.sec, '-o');  xygo('T / s', 'phiN');
        subplot(326); plot(phis(:,end), phis(:,3)/glv.min, '-o');  xygo('T / s', 'phiU');  phi=phis;
        return;
    end
    if length(lti)>1, lti=lti(1); end  % lti = pos;
    if isempty(lti), lti=vn(1,7); end  % vn=avp
    if size(vn,2)>=6, vn=vn(:,[4:6,end]); end  % av or avp
    if size(vn,2)==2, vn=[vn(:,1)*0, vn(:,1), vn(:,1)*0, vn(:,2)]; end  % [vN,t]
    t0 = vn(1,end); vn(:,end)=vn(:,end)-t0;
    for k=1:3, vn(:,k)=deltrend(vn(:,k),1,0);  vn(:,k)=vn(:,k)-vn(1,k);  end
    n = fix((vn(end,end)-alignT)/intervalT)+2;
    phi = zeros(n,4);
    for k=1:n
        t1 = (k-1)*intervalT; t2 = t1+alignT;
        if t2>vn(end,end)+1, phi(k:end,:)=[]; break; end
        [~, ~, phik] = vn2phiu(datacut(vn,t1,t2), lti, 0);
        phi(k,:) = [phik; t1]';
    end
    vn(:,end) = vn(:,end)+t0;
    phi(:,end) = phi(:,end)+t0;
    if isfig==1
        myfig,
        subplot(221), plot(vn(:,end), vn(:,1:2)); xygo('VEN'); xm=xlim;
        title(sprintf('alignT=%.1fs, intervalT=%.1fs',alignT,intervalT));
        subplot(223), plot(phi(:,end), phi(:,3)/glv.min, '-o'); xygo('phiU'); xlim(xm);
        title(sprintf('std=%.2f^\\prime',std(phi(:,3))/glv.min));
        subplot(222), plot(phi(:,end), phi(:,1)/glv.sec, '-o'); xygo('phiE'); xlim(xm);
        title(sprintf('std=%.2f^\\prime^\\prime',std(phi(:,1))/glv.sec));
        subplot(224), plot(phi(:,end), phi(:,2)/glv.sec, '-o'); xygo('phiN'); xlim(xm);
        title(sprintf('std=%.2f^\\prime^\\prime',std(phi(:,2))/glv.sec));
    end
