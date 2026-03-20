function [phi_std, phi_mn] = alignsars(av, pos, T, dT, ylimv)
% Statistic plot for Single-axis rotation SIMU alignment.
%
% Prototype: [phi_std, phi_mn] = alignsars(av, pos, T, dT, ylimv)
% Inputs: av - SIMU att&vel array
%         pos - initial position
%         T - alignment time length
%         dT - alignment step time
%         ylimv - y-axis display limits
% Outputs: phi_std, phi_mn - misalignment angles std & mean
%
% See also  alignsar.
%
% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/11/2025
global glv
    if nargin<5, ylimv=[-10,10]; end
    if length(ylimv)==1, ylimv=[-ylimv,ylimv]; end
    if nargin<4, dT=30; end
    if nargin<3, T=300; end
    t00 = av(1,end);
    myfig;
    for k=1:100
        t0 = t00+(k-1)*dT; t1 = t0+T;
        if t1>av(end,end), break; end
        [att, att0, res, phi] = alignsar(datacut(av,t0,t1), pos, 0, 0);
        subplot(131);  plot(res(:,end),res(:,1)/glv.sec); xygo('phiE')
        subplot(132);  plot(res(:,end),res(:,2)/glv.sec); xygo('phiN')
        subplot(133);  plot(res(:,end),res(:,3)/glv.min); xygo('phiU')
    end
    subplot(131), [phi_std(:,1), phi_mn(:,1)] = plotstd(3,2);
    subplot(132), [phi_std(:,2), phi_mn(:,2)] = plotstd(3,2);
    subplot(133), [phi_std(:,3), phi_mn(:,3)] = plotstd(3,2); ylim(ylimv);
