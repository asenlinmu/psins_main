function gylim(y0, y1, fign)
% In current SIMU figure (or fign), set all the gyro Y-axes limits to [y0, y1];
%
% Prototype: gylim(y0, y1, fign)
% Inputs: y0, y1 - ylim value.
%         fign - figure NO.
%
% See also  ylimall, xlimall.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/10/2025
    if nargin<3, h=gcf; else, h=figure(fign); end
    if nargin<2,
        if nargin<1, y0=[-15; 15]; end
        if length(y0)<2, y1=y0;      y0=0;
        else,            y1=y0(2);   y0=y0(1);   end
    end
    ax = findall(h, 'type', 'axes');
    for k=1:2:length(ax)  % only for left column axis
        if isempty(get(ax(k),'Tag')), ylim(ax(k), [y0, y1]); end  % not for legend
    end
