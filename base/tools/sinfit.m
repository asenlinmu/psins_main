function [A, t0, y0, yerr] = sinfit(t, y, f, isfig)
% Slove model y = A*sin(2*pi*f*(t-t0))+y0 to get parameters A, t0 and y0,
% where f is predetermined frequency.
%
% Prototype: [A, t0, y0, yerr] = sinfit(t, y, f, isfig)
% Inputs: t,y - t,y input t-y data
%         f - predetermined frequency
%         isfig - figure flag
% Outputs: A, t0, y0 - parameter estimated
%          yerr - model error
%
% Example
%   f=2;  t=(-1.51:0.01:1.51)';  y=2*sin(2*pi*f*(t-0.72))+0.3;  [A, t0, y0, yerr] = sinfit(t, y, f);
%
% See also  wavefit, kaafit, polyfit.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/12/2025
    if nargin<4, isfig=1; end
    w = 2*pi*f;
    my = (max(y)+min(y))/2;
    for k=1:length(y)-1
        if y(k)<=my&&y(k+1)>my, t0=t(k); break; end
    end
    % myfig, plot(t,y,'-',t(k),y(k),'o'); grid on;
    for n=1:3
        wt = w*(t-t0);
        H = [sin(wt), -cos(wt), ones(size(wt))];  % y=A*sin(wt)*cos(w*dt0)-A*cos(wt)*sin(w*dt0)+y0*1
        X = lscov(H, y);
        A = sign(X(1))*norm(X(1:2));
        dt0 = asin(X(2)/A)/w;
        t0 = t0+dt0;
    end
    y0 = X(3);
    yest = A*sin(w*(t-t0))+y0;
    yerr = y-yest;
    if isfig==1
        myfig
        subplot(211); plot(t, [y,yest]); xygo(' y '); plot(t0,y0,'o','linewidth',2);
        title(sprintf('A=%.3e, t0=%.3e, y0=%.3e', A,t0,y0));
        subplot(212); plot(t, yerr); xygo('err');
    end
