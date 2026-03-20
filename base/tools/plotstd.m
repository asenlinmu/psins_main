function [sigma, mn, x] = plotstd(nSigma, meanLineWidth)
% Add +-sigma lines to current figure.
%
% Prototype:  plotstd(nSigma, meanLineWidth)
% Inputs: nSigma - = 1,2,3 sigma lines
%         meanLineWidth - >0 for plot mean line, =0 for no plot
% Outputs: sigma, mn - std & mean of the lines
% 
% Examples
%   myfig, plot(randn(20,5)); plotstd();
%   myfig, plot(randn(20,10)); plotstd(3,2);
%
% See also  ladd, lmc.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 31/08/2025
    if size(nSigma,1)>1  % plotstd(data, ystr)
        data = nSigma;
        if nargin>1, t=data(:,end); data(:,end)=[]; xstr='t'; ystr=meanLineWidth;
        else, t=lent(data); xstr='k'; ystr='Val'; end
        sig = std(data);  rmss = rms(data);  maxs = max(abs(data));
        myfig, plot(t, data);  xygo(xstr, ystr);
        title({['STD = ', sprintf('%.4f; ',sig)], ['RMS = ', sprintf('%.4f; ',rmss)], ['MAX = ', sprintf('%.4f; ',maxs)]});
        return;
    end
    if nargin<2, meanLineWidth=0; end
    if nargin<1, nSigma=3; end
    ls = findobj(gca(), 'type', 'line');
    Mls = length(ls);  if Mls<3, error('Too few lines found in this figure!'); end
    N = 1000;  for k=1:Mls, N=min(N,length(ls(k).XData)); end  % max points
    sigma = zeros(N,1);  mn = sigma;
    for n=1:N
        s = zeros(Mls,1);
        for k=1:Mls, s(k) = ls(k).YData(n); end
        sigma(n) = std(s);  mn(n) = mean(s);
    end
    x = ls(2).XData(1:N);
    hold on, plot(x, nSigma*[sigma,-sigma]+mn, '--');
    title(sprintf('\\itN\\rm(%.4f, %.4f) @ %.2f',mn(end),sigma(end),ls(2).XData(N)));
    if meanLineWidth>0 
        plot(x, mn, '--', 'linewidth', meanLineWidth);
    end
